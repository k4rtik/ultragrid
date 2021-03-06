/**
 * @file   video_compress.c
 * @author Martin Pulec     <pulec@cesnet.cz>
 * @ingroup video_compress
 *
 * @brief Video compress functions.
 */
/*
 * Copyright (c) 2011-2013 CESNET z.s.p.o.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, is permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of CESNET nor the names of its contributors may be
 *    used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESSED OR IMPLIED WARRANTIES, INCLUDING,
 * BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifdef HAVE_CONFIG_H
#include "config.h"
#include "config_unix.h"
#include "config_win32.h"
#endif // HAVE_CONFIG_H

#include <list>
#include <memory>
#include <stdio.h>
#include <string>
#include <string.h>
#include <tuple>
#include <vector>

#include "messaging.h"
#include "module.h"
#include "utils/synchronized_queue.h"
#include "utils/vf_split.h"
#include "utils/worker.h"
#include "video.h"
#include "video_compress.h"
#include "lib_common.h"

using namespace std;

namespace {
/**
 * @brief This structure represents real internal compress state
 */
struct compress_state_real {
        const video_compress_info    *funcs;            ///< handle for the driver
        struct module     **state;                  ///< driver internal states
        unsigned int        state_count;            ///< count of compress states (equal to tiles' count)
        char                compress_options[1024]; ///< compress options (for reconfiguration)
};
}

/**
 * @brief Video compress state.
 *
 * This structure represents external video compress state. This is basically a proxy for real
 * state. The point of doing this is to allow dynamic reconfiguration of the real state.
 */
struct compress_state {
        struct module mod;               ///< compress module data
        struct compress_state_real *ptr; ///< pointer to real compress state
        synchronized_queue<shared_ptr<video_frame>, 1> queue;
};

typedef struct compress_state compress_state_proxy; ///< Used to emphasize that the state is actually a proxy.

/**
 * This is placeholder state returned by compression module meaning that the initialization was
 * successful but no state was create. This is the case eg. when the module only displayed help.
 */
struct module compress_init_noerr;

static shared_ptr<video_frame> compress_frame_tiles(struct compress_state_real *s,
                shared_ptr<video_frame> frame, struct module *parent);
static int compress_init_real(struct module *parent, const char *config_string,
                struct compress_state_real **state);
static void compress_done_real(struct compress_state_real *s);
static void compress_done(struct module *mod);

/// @brief Displays list of available compressions.
void show_compress_help()
{
        printf("Possible compression modules (see '-c <module>:help' for options):\n");
        list_modules(LIBRARY_CLASS_VIDEO_COMPRESS, VIDEO_COMPRESS_ABI_VERSION);
}

list<compress_preset> get_compress_capabilities()
{
        list<compress_preset> ret;

        auto compressions = get_libraries_for_class(LIBRARY_CLASS_VIDEO_COMPRESS, VIDEO_COMPRESS_ABI_VERSION);

        for (auto it : compressions) {
                auto vci = static_cast<const struct video_compress_info *>(it.second);
                auto presets = vci->get_presets();
                for (auto const & it : presets) {
                        auto new_elem = it;
                        new_elem.name = string(vci->name)
                                + ":" + it.name;
                        ret.push_back(new_elem);
                }
        }

        return ret;
}

/**
 * @brief Processes message.
 *
 * This function is a callback called from control thread to change some parameters of
 * compression.
 *
 * @param[in] receiver pointer to the compress module
 * @param[in] msg      message to process
 */
static void compress_process_message(compress_state_proxy *proxy, struct msg_change_compress_data *data)
{
        struct response *r = NULL;
        /* In this case we are only changing some parameter of compression.
         * This means that we pass the parameter to compress driver. */
        if(data->what == CHANGE_PARAMS) {
                assert(proxy->ptr->state_count > 1);
                for(unsigned int i = 0; i < proxy->ptr->state_count; ++i) {
                        struct msg_change_compress_data *tmp_data =
                                (struct msg_change_compress_data *)
                                new_message(sizeof(struct msg_change_compress_data));
                        tmp_data->what = data->what;
                        strncpy(tmp_data->config_string, data->config_string,
                                        sizeof(tmp_data->config_string) - 1);
                        struct response *resp = send_message_to_receiver(proxy->ptr->state[i],
                                        (struct message *) tmp_data);
                        /// @todo
                        /// Handle responses more inteligently (eg. aggregate).
                        free_response(r); // frees previous response
                        r = resp;
                }

        } else {
                struct compress_state_real *new_state;
                char config[1024];
                strncpy(config, data->config_string, sizeof(config));

                int ret = compress_init_real(&proxy->mod, config, &new_state);
                if(ret == 0) {
                        struct compress_state_real *old = proxy->ptr;
                        proxy->ptr = new_state;
                        compress_done_real(old);
                        r = new_response(RESPONSE_OK, NULL);
                } else {
                        r = new_response(RESPONSE_INT_SERV_ERR, NULL);
                }
        }

        free_message((struct message *) data, r);
}

/**
 * @brief This function initializes video compression.
 *
 * This function wrapps the call of compress_init_real().
 * @param[in] parent        parent module
 * @param[in] config_string configuration (in format <driver>:<options>)
 * @param[out] state        created state
 * @retval     0            if state created sucessfully
 * @retval    <0            if error occured
 * @retval    >0            finished successfully, no state created (eg. displayed help)
 */
int compress_init(struct module *parent, const char *config_string, struct compress_state **state) {
        struct compress_state_real *s;

        compress_state_proxy *proxy;
        proxy = new compress_state_proxy();

        module_init_default(&proxy->mod);
        proxy->mod.cls = MODULE_CLASS_COMPRESS;
        proxy->mod.priv_data = proxy;
        proxy->mod.deleter = compress_done;

        int ret = compress_init_real(&proxy->mod, config_string, &s);
        if(ret == 0) {
                proxy->ptr = s;

                *state = proxy;
                module_register(&proxy->mod, parent);
        } else {
                delete proxy;
        }

        return ret;
}

/**
 * @brief This is compression initialization function that really does the stuff.
 * @param[in] parent        parent module
 * @param[in] config_string configuration (in format <driver>:<options>)
 * @param[out] state        created state
 * @retval     0            if state created sucessfully
 * @retval    <0            if error occured
 * @retval    >0            finished successfully, no state created (eg. displayed help)
 */
static int compress_init_real(struct module *parent, const char *config_string,
                struct compress_state_real **state)
{
        struct compress_state_real *s;
        string compress_name;
        string compress_options;

        if(!config_string)
                return -1;

        if(strcmp(config_string, "help") == 0)
        {
                show_compress_help();
                return 1;
        }

        char *tmp = strdup(config_string);
        if (strchr(tmp, ':')) {
                char *opts = strchr(tmp, ':') + 1;
                *strchr(tmp, ':') = '\0';
                compress_options = opts;
        }
        compress_name = tmp;
        free(tmp);

        auto vci = static_cast<const struct video_compress_info *>(load_library(compress_name.c_str(), LIBRARY_CLASS_VIDEO_COMPRESS, VIDEO_COMPRESS_ABI_VERSION));
        if(!vci) {
                fprintf(stderr, "Unknown compression: %s\n", config_string);
                return -1;
        }

        s = (struct compress_state_real *) calloc(1, sizeof(struct compress_state_real));
        s->state_count = 1;

        s->funcs = vci;

        strncpy(s->compress_options, compress_options.c_str(), sizeof(s->compress_options) - 1);

        if(s->funcs->init_func) {
                s->state = (struct module **) calloc(1, sizeof(struct module *));
                s->state[0] = s->funcs->init_func(parent, s->compress_options);
                if(!s->state[0]) {
                        fprintf(stderr, "Compression initialization failed: %s\n", config_string);
                        free(s->state);
                        free(s);
                        return -1;
                }
                if(s->state[0] == &compress_init_noerr) {
                        free(s->state);
                        free(s);
                        return 1;
                }
        } else {
                free(s);
                return -1;
        }

        *state = s;
        return 0;
}

/**
 * @brief Returns name of compression module
 *
 * @param proxy compress state
 * @returns     compress name
 */
const char *get_compress_name(compress_state_proxy *proxy)
{
        if(proxy)
                return proxy->ptr->funcs->name;
        else
                return NULL;
}

/**
 * @brief Compressses frame
 *
 * @param proxy        compress state
 * @param frame        uncompressed frame to be compressed
 * @return             compressed frame, may be NULL if compression failed
 */
void compress_frame(compress_state_proxy *proxy, shared_ptr<video_frame> frame)
{
        if (!frame) { // pass poisoned pill
                if (proxy)
                        proxy->queue.push(shared_ptr<video_frame>());
                return;
        }

        if (!proxy)
                abort();

        struct msg_change_compress_data *msg = NULL;
        while ((msg = (struct msg_change_compress_data *) check_message(&proxy->mod))) {
                compress_process_message(proxy, msg);
        }

        struct compress_state_real *s = proxy->ptr;

        shared_ptr<video_frame> sync_api_frame;
        if (s->funcs->compress_frame_func) {
                sync_api_frame = s->funcs->compress_frame_func(s->state[0], frame);
        } else if(s->funcs->compress_tile_func) {
                sync_api_frame = compress_frame_tiles(s, frame, &proxy->mod);
        } else {
                sync_api_frame = {};
        }

        // empty return value here represents error, but we don't want to pass it to queue, since it would
        // be interpreted as poisoned pill
        if (!sync_api_frame) {
                return;
        }

        proxy->queue.push(sync_api_frame);
}

/**
 * @name Tile API Routines
 * The worker callbacks here are optimization - all tiles are processed concurrently.
 * @{
 */
/**
 * @brief Auxiliary structure passed to worker thread.
 */
struct compress_worker_data {
        struct module *state;      ///< compress driver status
        shared_ptr<video_frame> frame; ///< uncompressed tile to be compressed

        compress_tile_t callback;  ///< tile compress callback
        shared_ptr<video_frame> ret; ///< OUT - returned compressed tile, NULL if failed
};

/**
 * @brief This function is callback passed to a "thread pool"
 * @param arg @ref compress_worker_data
 * @return @ref compress_worker_data (same as input)
 */
static void *compress_tile_callback(void *arg) {
        compress_worker_data *s = (compress_worker_data *) arg;

        s->ret = s->callback(s->state, s->frame);

        return s;
}

/**
 * Compresses video frame with tiles API
 *
 * @param[in]     s             compress state
 * @param[in]     frame         uncompressed frame
 * @param         parent        parent module (for the case when there is a need to reconfigure)
 * @return                      compressed video frame, may be NULL if compression failed
 */
static shared_ptr<video_frame> compress_frame_tiles(struct compress_state_real *s,
                shared_ptr<video_frame> frame, struct module *parent)
{
        if(frame->tile_count != s->state_count) {
                s->state = (struct module **) realloc(s->state, frame->tile_count * sizeof(struct module *));
                for(unsigned int i = s->state_count; i < frame->tile_count; ++i) {
                        s->state[i] = s->funcs->init_func(parent, s->compress_options);
                        if(!s->state[i]) {
                                fprintf(stderr, "Compression initialization failed\n");
                                return NULL;
                        }
                }
                s->state_count = frame->tile_count;
        }

        vector<shared_ptr<video_frame>> separate_tiles = vf_separate_tiles(frame);
        // frame pointer may no longer be valid
        frame = NULL;

        vector<task_result_handle_t> task_handle(separate_tiles.size());

        vector <compress_worker_data> data_tile(separate_tiles.size());
        for(unsigned int i = 0; i < separate_tiles.size(); ++i) {
                struct compress_worker_data *data = &data_tile[i];
                data->state = s->state[i];
                data->frame = separate_tiles[i];
                data->callback = s->funcs->compress_tile_func;

                task_handle[i] = task_run_async(compress_tile_callback, data);
        }

        vector<shared_ptr<video_frame>> compressed_tiles(separate_tiles.size(), nullptr);

        bool failed = false;
        for(unsigned int i = 0; i < separate_tiles.size(); ++i) {
                struct compress_worker_data *data = (struct compress_worker_data *)
                        wait_task(task_handle[i]);

                if(!data->ret) {
                        failed = true;
                }

                compressed_tiles[i] = data->ret;
        }

        if (failed) {
                return NULL;
        }

        return vf_merge_tiles(compressed_tiles);
}
/**
 * @}
 */

/**
 * @brief Video compression cleanup function.
 * @param mod video compress module
 */
static void compress_done(struct module *mod)
{
        if(!mod)
                return;

        compress_state_proxy *proxy = (compress_state_proxy *) mod->priv_data;
        struct compress_state_real *s = proxy->ptr;
        compress_done_real(s);

        delete proxy;
}

/**
 * Video compression cleanup function.
 * This destroys contained real video compress state.
 * @param mod video compress module
 */
static void compress_done_real(struct compress_state_real *s)
{
        if(!s)
                return;

        for(unsigned int i = 0; i < s->state_count; ++i) {
                module_done(s->state[i]);
        }
        free(s->state);
        free(s);
}

shared_ptr<video_frame> compress_pop(compress_state_proxy *proxy)
{
        if(!proxy)
                return NULL;

        return proxy->queue.pop();
}

