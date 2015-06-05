/**
 * @file   capture_filter/anonymize.cpp
 * @author Martin Pulec     <pulec@cesnet.cz>
 */
/*
 * Copyright (c) 2014, CESNET z. s. p. o.
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
#endif /* HAVE_CONFIG_H */

#include "capture_filter.h"

#include "debug.h"
#include "lib_common.h"
#include "module.h"

#include "video.h"
#include "video_codec.h"

#include <iostream>
#include <memory>
#include <map>
#include <string>
#include <sstream>
#include <vector>
#include <tesseract/baseapi.h>
#include <leptonica/allheaders.h>
#include <tre/tre.h>
#include <opencv/cv.hpp>
#include <opencv/cv.h>


using namespace std;

struct module;

static int init(struct module *parent, const char *cfg, void **state);
static void done(void *state);
static struct video_frame *filter(void *state, struct video_frame *in);

struct word_type {
        word_type(shared_ptr<BOX> box, int index, int conf, string &&s) :
                m_box(box), m_index(index), m_confidence(conf), m_str(s)
        {
        }
        shared_ptr<BOX> m_box;
        int m_index;
        int m_confidence;
        string m_str;
};

class state_anonymize {
public:
        state_anonymize(const char *);
        ~state_anonymize();
        struct video_frame *filter(struct video_frame *in);
        typedef vector<word_type> word_ret;
        word_ret get_words(Boxa* boxes);

        vector<string> match_paragraphs(word_ret const & word_list);
private:
        tesseract::TessBaseAPI m_api;
        regex_t m_regex;
        bool m_regex_in_use;
};

state_anonymize::state_anonymize(const char *cfg)
{
        if (m_api.Init(NULL, "ces")) {
                throw string("Could not initialize tesseract");
        }

        if (cfg && strlen(cfg) > 0) {
                const char *regex = cfg;
                wchar_t *regex_w = (wchar_t *) malloc((mbstowcs(NULL,regex,0) + 1) * sizeof(wchar_t));
                size_t len = mbstowcs(regex_w, regex, mbstowcs(NULL,regex,0) + 1);
                int err = tre_regwncomp(&m_regex, regex_w, len, REG_EXTENDED);
                free(regex_w);
                if (err != 0) {
                        throw string("Unable to parse regex!");
                }
                m_regex_in_use = true;
        } else {
                m_regex_in_use = false;
        }
}

state_anonymize::~state_anonymize()
{
        m_api.End();
        if (m_regex_in_use)
                tre_regfree(&m_regex);
}

static void destroy_box(BOX *b) {
        boxDestroy(&b);
}

state_anonymize::word_ret state_anonymize::get_words(Boxa* boxes)
{
        word_ret ret;
        for (int i = 0; i < boxes->n; i++) {
                BOX* box = boxaGetBox(boxes, i, L_CLONE);
                m_api.SetRectangle(box->x, box->y, box->w, box->h);
                char* ocrResult = m_api.GetUTF8Text();
                int conf = m_api.MeanTextConf();
                ret.push_back(word_type(shared_ptr<BOX>(box, destroy_box), i, conf, string(ocrResult)));
                delete [] ocrResult;
        }
        return ret;
}

tuple<string, int> process_a_line(state_anonymize::word_ret const & word_list, vector<bool> & used,
                multimap<int, int> &map_x,
                int first_word_index)
{
        int ret_int = 1;
        ostringstream oss;
        string str = word_list[first_word_index].m_str;
        if (str.size() > 1) {
                str.pop_back();
                str.pop_back();
        }
        oss << str;
        used[first_word_index] = true;

        // process a line
        word_type const * last = &word_list[first_word_index];
        int next_x = last->m_box->x + last->m_box->w;
        for (auto it = map_x.lower_bound(next_x); it != map_x.end(); ++it) {
                if (used[it->second]) {
                        continue;
                }

                if (it->first > next_x + last->m_box->h) {
                        // finish here, next word is too far
                        break;
                }

                if (abs(word_list[it->second].m_box->y - last->m_box->y) < last->m_box->h / 2) {
                        assert(word_list[it->second].m_index == it->second);
                        used[it->second] = true;

                        last = &word_list[it->second];
                        next_x = last->m_box->x + last->m_box->w;
                        ret_int += 1;
                        string str = word_list[it->second].m_str;
                        if (str.size() > 1) {
                                str.pop_back();
                                str.pop_back();
                        }
                        oss << " " << str;
                        // reset iterator
                        //it = map_x.lower_bound(next_x);
                }
        }

        return tuple<string, int>(oss.str(), ret_int);
}

constexpr int MIN_CONFIDENCE = 20;

tuple<string, int>
find_group(state_anonymize::word_ret const & word_list, vector<bool> & used, multimap<int, int> &map_x, multimap<int, int> &map_y) {
        // pick pivot word
        ssize_t line_pivot = -1;
        for (unsigned int i = 0; i < used.size(); ++i) {
                if (!used[i]) {
                        line_pivot = i;
                        break;
                }
        }

        assert(line_pivot != -1);

        int total_processed_words = 0;
        ostringstream oss;

        auto t = process_a_line(word_list, used, map_x, line_pivot);
        total_processed_words += get<1>(t);
        oss << get<0>(t);
        if (word_list[line_pivot].m_confidence < MIN_CONFIDENCE) {
                return tuple<string, int>({}, total_processed_words);
        }

        word_type const * last = &word_list[line_pivot];
        int next_y = last->m_box->y + last->m_box->h;
        for (auto it = map_y.lower_bound(next_y); it != map_y.end(); ++it) {
                if (used[it->second]) {
                        continue;
                }
                if (it->first > next_y + last->m_box->h * 2 / 3) {
                        // finish here, next word is too far
                        break;
                }

                if (abs(word_list[it->second].m_box->x - last->m_box->x) < last->m_box->h / 3) {
                        line_pivot = it->second;
                        last = &word_list[line_pivot];
                        next_y = last->m_box->y + last->m_box->h;

                        auto t = process_a_line(word_list, used, map_x, line_pivot);
                        total_processed_words += get<1>(t);
                        oss << "\n" << get<0>(t);
                }
        }

        return tuple<string, int>(oss.str(), total_processed_words);
}

vector<string> state_anonymize::match_paragraphs(state_anonymize::word_ret const & word_list)
{
        vector<string> ret;

        multimap<int, int> map_x;
        multimap<int, int> map_y;
        vector<bool> used(word_list.size());
        size_t total_used = 0;

        int index = 0;
        for (auto &t : word_list) {
                map_x.emplace(t.m_box->x, index);
                map_y.emplace(t.m_box->y, index);
                index++;
        }

        int i = 0;
        while (total_used < word_list.size()) {
                tuple<string, int> rc =
                        find_group(word_list, used, map_x, map_y);
                total_used += get<1>(rc);
                if (!get<0>(rc).empty()) {
                        i++;
                        ret.push_back(get<0>(rc));
                }
        }

        return ret;
}

struct video_frame *state_anonymize::filter(struct video_frame *in)
{
        // Open input image with leptonica library
        //api->SetImage(image);
        //unsigned char *data = new unsigned char[in->tiles[0].height * vc_get_linesize(in->tiles[0].width, RGB)];
        unsigned char *data;

        //Pix *image = pixReadMem(data, in->tiles[0].data_len / 2 * 3);
        //abort();
        size_t width = in->tiles[0].width;
        size_t height = in->tiles[0].height;

        Pix *image = pixCreateNoInit(in->tiles[0].width, in->tiles[0].height, 8);
        vc_copylineUYVYtoGrayscale((unsigned char *) image->data, (unsigned char *) in->tiles[0].data, in->tiles[0].data_len / 2);
        //vc_copylineUYVYtoRGB((unsigned char *) data, (unsigned char *) in->tiles[0].data, in->tiles[0].data_len / 2 * 3);
        //m_api.SetImage(data, in->tiles[0].width, in->tiles[0].height, 3, vc_get_linesize(in->tiles[0].width, RGB));
        //m_api.SetImage(image);
        //

        unsigned char *d = (unsigned char *) image->data;
        for (unsigned int i = 0; i < in->tiles[0].width * in->tiles[0].height; ++i) {
                if (d[i] < 192) d[i] = 0;
        }

        data = (unsigned char *) image->data;

#if 0
        cv::Mat input;
        cv::Mat output;
        input.create(height, width, CV_8UC1);
        input.data = (unsigned char *) image->data;

        size_t dilation_size = 1;
        cv::Mat element = getStructuringElement( cv::MORPH_RECT,
                        cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                        cv::Point( dilation_size, dilation_size ) );
        /// Apply the dilation operation
        cv::dilate( input, output, element );

        data = output.data;
#endif

        static int i = 0;
        if (i++ == 0) {
                FILE *in_file = fopen("in.yuv", "w+");
                fwrite(in->tiles[0].data, width * height * 2, 1, in_file);
                fclose(in_file);

                FILE *out = fopen("out.pnm", "w+");
                fprintf(out, "P5\n%zu %zu\n255\n", width, height);
                fwrite(data, width * height, 1, out);
                fclose(out);
                sync();
        }

        m_api.SetImage((unsigned char *) data, width, height, 1, width);

        // Get OCR result
        //outText = api->GetUTF8Text();
        //printf("OCR output:\n%s", outText);
        Boxa* boxes = m_api.GetComponentImages(tesseract::RIL_WORD, true, NULL, NULL);
        if (boxes) {
                printf("Found %d textline image components.\n", boxes->n);
                auto words = get_words(boxes);
                for (auto &t : words) {
                        shared_ptr<BOX> box = t.m_box;
                        int i = t.m_index;
                        int conf = t.m_confidence;
                        string s = t.m_str;
                        fprintf(stdout, "Box[%d]: x=%d, y=%d, w=%d, h=%d, confidence: %d, text: %s",
                                        i, box->x, box->y, box->w, box->h, conf, s.c_str());
                }
                auto ret = match_paragraphs(words);
                cout << "Paragraphs:\n";
                for (auto & s : ret) {
                        cout << "\"" << s << "\"" <<  "\n";
                        if (m_regex_in_use) {
                                wchar_t *regex_w = (wchar_t *) malloc((mbstowcs(NULL,s.c_str(),0) + 1) * sizeof(wchar_t));
                                size_t len = mbstowcs(regex_w, s.c_str(), mbstowcs(NULL,s.c_str(),0) + 1);
                                regamatch_t match;
                                regaparams_t params;
                                tre_regaparams_default(&params);
                                match.nmatch = 0;
                                params.max_cost = 0;
                                if (tre_regawexec(&m_regex, regex_w, &match, params, 0) == 0)
                                //if (tre_regwexec(&m_regex, regex_w, 0, NULL, 0) == 0)
                                        cerr << "MATCH" << endl;
                                free(regex_w);
                        }
                }
        }

        // Destroy used object and release memory
        pixDestroy(&image);
        return in;
}

static int init(struct module *parent, const char *cfg, void **state)
{
        UNUSED(parent);
        *state = new state_anonymize(cfg);
        return 0;
}

static void done(void *state)
{
        delete (state_anonymize *) state;
}

static struct video_frame *filter(void *state, struct video_frame *in)
{
        return ((state_anonymize *) state)->filter(in);
}

static struct capture_filter_info capture_filter_anonymize = {
    "anonymize",
    init,
    done,
    filter,
};

static void mod_reg(void)  __attribute__((constructor));

static void mod_reg(void)
{
        register_library("capture_filter_anonymize", &capture_filter_anonymize, LIBRARY_CLASS_CAPTURE_FILTER, CAPTURE_FILTER_ABI_VERSION);
}

