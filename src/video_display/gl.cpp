/**
 * @file   video_display/gl.cpp
 * @author Lukas Hejtmanek  <xhejtman@ics.muni.cz>
 * @author Milos Liska      <xliska@fi.muni.cz>
 * @author Martin Pulec     <pulec@cesnet.cz>
 */
/*
 * Copyright (c) 2010-2014 CESNET, z. s. p. o.
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
#endif

#include <assert.h>
#ifdef HAVE_MACOSX
#include <OpenGL/gl.h>
#include <OpenGL/OpenGL.h> // CGL
#include <OpenGL/glext.h>
#include <GLUT/glut.h>
#elif defined HAVE_LINUX
#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glx.h>
#include <GL/glut.h>
#include "x11_common.h"
#else // WIN32
#include <GL/glew.h>
#include <GL/glut.h>
#endif /* HAVE_MACOSX */

#ifdef FREEGLUT
#include <GL/freeglut_ext.h>
#endif /* FREEGLUT */

#include <condition_variable>
#include <mutex>
#include <queue>

#include "debug.h"
#include "gl_context.h"
#include "host.h"
#include "video.h"
#include "video_display.h"
#include "video_display/gl.h"
#include "video_display/splashscreen.h"
#include "tv.h"

#define MAGIC_GL         DISPLAY_GL_ID
#define DEFAULT_WIN_NAME "Ultragrid - OpenGL Display"

#define STRINGIFY(A) #A

#define MAX_BUFFER_SIZE 1

using namespace std;

static const char * yuv422_to_rgb_fp = STRINGIFY(
uniform sampler2D image;
uniform float imageWidth;
void main()
{
        vec4 yuv;
        yuv.rgba  = texture2D(image, gl_TexCoord[0].xy).grba;
        if(gl_TexCoord[0].x * imageWidth / 2.0 - floor(gl_TexCoord[0].x * imageWidth / 2.0) > 0.5)
                yuv.r = yuv.a;
        yuv.r = 1.1643 * (yuv.r - 0.0625);
        yuv.g = yuv.g - 0.5;
        yuv.b = yuv.b - 0.5;
        gl_FragColor.r = yuv.r + 1.7926 * yuv.b;
        gl_FragColor.g = yuv.r - 0.2132 * yuv.g - 0.5328 * yuv.b;
        gl_FragColor.b = yuv.r + 2.1124 * yuv.g;
});

/* DXT YUV (FastDXT) related */
static const char *fp_display_dxt1 = STRINGIFY(
        uniform sampler2D yuvtex;

        void main(void) {
        vec4 col = texture2D(yuvtex, gl_TexCoord[0].st);

        float Y = 1.1643 * (col[0] - 0.0625);
        float U = (col[1] - 0.5);
        float V = (col[2] - 0.5);

        float R = Y + 1.7926 * V;
        float G = Y - 0.2132 * U - 0.5328 * V;
        float B = Y + 2.1124 * U;

        gl_FragColor=vec4(R,G,B,1.0);
}
);

static const char * vert = STRINGIFY(
void main() {
        gl_TexCoord[0] = gl_MultiTexCoord0;
        gl_Position = ftransform();}
);

static const char fp_display_dxt5ycocg[] = STRINGIFY(
uniform sampler2D image;
void main()
{
        vec4 color;
        float Co;
        float Cg;
        float Y;
        float scale;
        color = texture2D(image, gl_TexCoord[0].xy);
        scale = (color.z * ( 255.0 / 8.0 )) + 1.0;
        Co = (color.x - (0.5 * 256.0 / 255.0)) / scale;
        Cg = (color.y - (0.5 * 256.0 / 255.0)) / scale;
        Y = color.w;
        gl_FragColor = vec4(Y + Co - Cg, Y + Cg, Y - Co - Cg, 1.0);
} // main end
);

struct state_gl {
        GLuint          PHandle_uyvy, PHandle_dxt, PHandle_dxt5;

        // Framebuffer
        GLuint fbo_id;
	GLuint		texture_display;
	GLuint		texture_uyvy;

        /* For debugging... */
        uint32_t	magic;

        int             window;

	bool            fs;
        bool            deinterlace;

        struct video_frame *current_frame;

        queue<struct video_frame *> frame_queue;
        queue<struct video_frame *> free_frame_queue;
        struct video_desc current_desc;
        struct video_desc current_display_desc;
        mutex           lock;
        condition_variable frame_consumed_cv;

        double          aspect;
        double          video_aspect;
        unsigned long int frames;
        
        int             dxt_height;

        struct timeval  tv;

        bool            sync_on_vblank;
        bool            paused;
        bool            show_cursor;

        bool should_exit_main_loop;

        double          window_size_factor;

        state_gl() : PHandle_uyvy(0), PHandle_dxt(0), PHandle_dxt5(0),
                fbo_id(0), texture_display(0), texture_uyvy(0),
                magic(MAGIC_GL), window(-1), fs(false), deinterlace(false), current_frame(nullptr),
                aspect(0.0), video_aspect(0.0), frames(0ul), dxt_height(0),
                sync_on_vblank(true), paused(false), show_cursor(false), 
                should_exit_main_loop(false), window_size_factor(1.0)
        {
                gettimeofday(&tv, NULL);
                memset(&current_desc, 0, sizeof(current_desc));
                memset(&current_display_desc, 0, sizeof(current_display_desc));
        }
};

static struct state_gl *gl;

/* Prototyping */
static void gl_draw(double ratio, double bottom_offset);
static void gl_show_help(void);

static void gl_resize(int width, int height);
static void gl_render_uyvy(struct state_gl *s, char *data);
static void gl_reconfigure_screen(struct state_gl *s, struct video_desc desc);
static void glut_idle_callback(void);
static void glut_key_callback(unsigned char key, int x, int y);
static void glut_close_callback(void);
static void glut_resize_window(bool fs, int height, double aspect, double window_size_factor);
static void display_gl_set_sync_on_vblank(int value);
static void screenshot(struct video_frame *frame);

#ifdef HAVE_MACOSX
extern "C" void NSApplicationLoad(void);
#endif

/**
 * Show help
 */
static void gl_show_help(void) {
        printf("GL options:\n");
        printf("\t-d gl[:d|:fs|:aspect=<v>/<h>|:cursor:|size=X%%]* | help\n\n");
        printf("\t\td\t\tdeinterlace\n");
        printf("\t\tfs\t\tfullscreen\n");
        printf("\t\nnovsync\t\tdo not turn sync on VBlank\n");
        printf("\t\taspect=<w>/<h>\trequested video aspect (eg. 16/9). Leave unset if PAR = 1.\n");
        printf("\t\tcursor\t\tshow visible cursor\n");
        printf("\t\tsize\t\tspecifies desired size of window compared "
                        " to native resolution (in percents)\n");

        printf("\n\nKeyboard shortcuts:\n");
        printf("\t\t'f'\t\ttoggle fullscreen\n");
        printf("\t\t'q'\t\tquit\n");
        printf("\t\t'd'\t\ttoggle deinterlace\n");
        printf("\t\t' '\t\tpause video\n");
        printf("\t\t's'\t\tscreenshot\n");
        printf("\t\t'm'\t\tshow/hide cursor\n");
        printf("\t\t'+'\t\tmake window smaller by factor 50%%\n");
        printf("\t\t'-'\t\tmake window twice as bigger\n");
}

static void gl_load_splashscreen(struct state_gl *s)
{
        struct video_desc desc;

        desc.width = 512;
        desc.height = 512;
        desc.color_spec = RGBA;
        desc.interlacing = PROGRESSIVE;
        desc.fps = 1;
        desc.tile_count = 1;

        display_gl_reconfigure(s, desc);

        struct video_frame *frame = vf_alloc_desc_data(desc);

        const char *data = splash_data;
        memset(frame->tiles[0].data, 0, frame->tiles[0].data_len);
        for (unsigned int y = 0; y < splash_height; ++y) {
                char *line = frame->tiles[0].data;
                line += vc_get_linesize(frame->tiles[0].width,
                                frame->color_spec) *
                        (((frame->tiles[0].height - splash_height) / 2) + y);
                line += vc_get_linesize(
                                (frame->tiles[0].width - splash_width)/2,
                                frame->color_spec);
                for (unsigned int x = 0; x < splash_width; ++x) {
                        HEADER_PIXEL(data,line);
                        line += 4;
                }
        }

        display_gl_putf(s, frame, PUTF_BLOCKING);
}

void * display_gl_init(const char *fmt, unsigned int flags) {
        UNUSED(flags);
	struct state_gl *s = new state_gl;
        
        /* GLUT callbacks take only some arguments so we need static variable */
        gl = s;

	// parse parameters
	if (fmt != NULL) {
		if (strcmp(fmt, "help") == 0) {
			gl_show_help();
			free(s);
			return &display_init_noerr;
		}

		char *tmp = strdup(fmt);
		char *tok, *save_ptr = NULL;
		
		while((tok = strtok_r(tmp, ":", &save_ptr)) != NULL) {
                        if(!strcmp(tok, "d")) {
                                s->deinterlace = true;
                        } else if(!strcmp(tok, "fs")) {
                                s->fs = true;
                        } else if(!strncmp(tok, "aspect=", strlen("aspect="))) {
                                s->video_aspect = atof(tok + strlen("aspect="));
                                char *pos = strchr(tok,'/');
                                if(pos) s->video_aspect /= atof(pos + 1);
                        } else if(!strcasecmp(tok, "novsync")) {
                                s->sync_on_vblank = false;
                        } else if (!strcasecmp(tok, "cursor")) {
                                s->show_cursor = true;
                        } else if(!strncmp(tok, "size=",
                                                strlen("size="))) {
                                s->window_size_factor =
                                        atof(tok + strlen("size=")) / 100.0;
                        } else {
                                fprintf(stderr, "[GL] Unknown option: %s\n", tok);
                        }
                        tmp = NULL;
                }

		free(tmp);
	}

        fprintf(stdout,"GL setup: fullscreen: %s, deinterlace: %s\n",
                        s->fs ? "ON" : "OFF", s->deinterlace ? "ON" : "OFF");

        gl_load_splashscreen(s);

        return (void*)s;
}

/**
 * This function just sets new video description.
 */
int display_gl_reconfigure(void *state, struct video_desc desc)
{
        struct state_gl	*s = (struct state_gl *) state;

        assert (desc.color_spec == RGBA ||
                        desc.color_spec == RGB  ||
                        desc.color_spec == UYVY ||
                        desc.color_spec == DXT1 ||
                        desc.color_spec == DXT1_YUV ||
                        desc.color_spec == DXT5);

        s->current_desc = desc;

        return TRUE;
}

static void glut_resize_window(bool fs, int height, double aspect, double window_size_factor)
{
        if (fs) {
                glutReshapeWindow(glutGet(GLUT_SCREEN_WIDTH),
                               glutGet(GLUT_SCREEN_HEIGHT));
                glutFullScreen();
        } else {
                glutReshapeWindow(window_size_factor *
                                height * aspect,
                                window_size_factor *
                                height);
        }
}

/*
 * Please note, that setting the value of 0 to GLX function is invalid according to
 * documentation. However. reportedly NVidia driver does unset VSync.
 */
static void display_gl_set_sync_on_vblank(int value) {
#ifdef HAVE_MACOSX
        int swap_interval = value;
        CGLContextObj cgl_context = CGLGetCurrentContext();
        CGLSetParameter(cgl_context, kCGLCPSwapInterval, &swap_interval);
#elif HAVE_LINUX
        /* using GLX_SGI_swap_control
         *
         * Also it is worth considering to use GLX_EXT_swap_control (instead?).
         * But we would need both Display and GLXDrawable variables which we do not currently have
         */
        int (*glXSwapIntervalSGIProc)(int interval) = 0;

        glXSwapIntervalSGIProc = (int (*)(int))
                glXGetProcAddressARB( (const GLubyte *) "glXSwapIntervalSGI");

        if(glXSwapIntervalSGIProc) {
                glXSwapIntervalSGIProc(value);
        } else {
                fprintf(stderr, "[GL display] GLX_SGI_swap_control is presumably not supported. Unable to set sync-on-VBlank.\n");
        }
#endif
}

static void screenshot(struct video_frame *frame)
{
        char name[128];
        time_t t;
        struct tm time_tmp;

        t = time(NULL);
        localtime_r(&t, &time_tmp);

        strftime(name, sizeof(name), "screenshot-%a, %d %b %Y %T %z.pnm",
                                               &time_tmp);
        save_video_frame_as_pnm(frame, name);
}

/**
 * This function does the actual reconfiguration of GL state.
 *
 * This function must be called only from GL thread.
 */
static void gl_reconfigure_screen(struct state_gl *s, struct video_desc desc)
{
        assert(s->magic == MAGIC_GL);

        if(desc.color_spec == DXT1 || desc.color_spec == DXT1_YUV || desc.color_spec == DXT5) {
                s->dxt_height = (desc.height + 3) / 4 * 4;
        } else {
                s->dxt_height = desc.height;
        }

        if(!s->video_aspect)
                s->aspect = (double) desc.width / desc.height;
        else
                s->aspect = s->video_aspect;

        fprintf(stdout,"Setting GL window size %dx%d (%dx%d).\n", (int)(s->aspect * desc.height),
                        desc.height, desc.width, desc.height);
        glutShowWindow();
        glut_resize_window(s->fs, desc.height, s->aspect, s->window_size_factor);

        glUseProgram(0);

        gl_check_error();

        if(desc.color_spec == DXT1 || desc.color_spec == DXT1_YUV) {
                glBindTexture(GL_TEXTURE_2D,s->texture_display);
                size_t data_len = ((desc.width + 3) / 4 * 4* s->dxt_height)/2;
                char *buffer = (char *) malloc(data_len);
                glCompressedTexImage2D(GL_TEXTURE_2D, 0,
                                GL_COMPRESSED_RGBA_S3TC_DXT1_EXT,
                                (desc.width + 3) / 4 * 4, s->dxt_height, 0, data_len,
                                /* passing NULL here isn't allowed and some rigid implementations
                                 * will crash here. We just pass some egliable buffer to fulfil
                                 * this requirement. glCompressedSubTexImage2D works as expected.
                                 */
                                buffer);
                free(buffer);
                if(desc.color_spec == DXT1_YUV) {
                        glBindTexture(GL_TEXTURE_2D,s->texture_display);
                        glUseProgram(s->PHandle_dxt);
                }
        } else if (desc.color_spec == UYVY) {
                glActiveTexture(GL_TEXTURE0 + 2);
                glBindTexture(GL_TEXTURE_2D,s->texture_uyvy);
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA,
                                desc.width / 2, desc.height, 0,
                                GL_RGBA, GL_UNSIGNED_BYTE,
                                NULL);
                glUseProgram(s->PHandle_uyvy);
                glUniform1i(glGetUniformLocation(s->PHandle_uyvy, "image"), 2);
                glUniform1f(glGetUniformLocation(s->PHandle_uyvy, "imageWidth"),
                                (GLfloat) desc.width);
                glUseProgram(0);
                glActiveTexture(GL_TEXTURE0 + 0);
                glBindTexture(GL_TEXTURE_2D,s->texture_display);
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA,
                                desc.width, desc.height, 0,
                                GL_RGBA, GL_UNSIGNED_BYTE,
                                NULL);
        } else if (desc.color_spec == RGBA) {
                glBindTexture(GL_TEXTURE_2D,s->texture_display);
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA,
                                desc.width, desc.height, 0,
                                GL_RGBA, GL_UNSIGNED_BYTE,
                                NULL);
        } else if (desc.color_spec == RGB) {
                glBindTexture(GL_TEXTURE_2D,s->texture_display);
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB,
                                desc.width, desc.height, 0,
                                GL_RGB, GL_UNSIGNED_BYTE,
                                NULL);
        } else if (desc.color_spec == DXT5) {
                glUseProgram(s->PHandle_dxt5);

                glBindTexture(GL_TEXTURE_2D,s->texture_display);
                glCompressedTexImage2D(GL_TEXTURE_2D, 0,
                                GL_COMPRESSED_RGBA_S3TC_DXT5_EXT,
                                (desc.width + 3) / 4 * 4, s->dxt_height, 0,
                                (desc.width + 3) / 4 * 4 * s->dxt_height,
                                NULL);
        }

        gl_check_error();

        display_gl_set_sync_on_vblank(s->sync_on_vblank ? 1 : 0);
        gl_check_error();

        s->current_display_desc = desc;
}

static void gl_render(struct state_gl *s, char *data)
{
        /* for DXT, deinterlacing doesn't make sense since it is
         * always deinterlaced before comrpression */
        if(s->deinterlace && (s->current_display_desc.color_spec == RGBA || s->current_display_desc.color_spec == UYVY))
                vc_deinterlace((unsigned char *) data,
                                vc_get_linesize(s->current_display_desc.width, s->current_display_desc.color_spec),
                                s->current_display_desc.height);

        gl_check_error();

        switch(s->current_display_desc.color_spec) {
                case DXT1:
                        glCompressedTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0,
                                        (s->current_display_desc.width + 3) / 4 * 4, s->dxt_height,
                                        GL_COMPRESSED_RGBA_S3TC_DXT1_EXT,
                                        ((s->current_display_desc.width + 3) / 4 * 4 * s->dxt_height)/2,
                                        data);
                case DXT1_YUV:
                        glCompressedTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0,
                                        s->current_display_desc.width, s->current_display_desc.height,
                                        GL_COMPRESSED_RGBA_S3TC_DXT1_EXT,
                                        (s->current_display_desc.width * s->current_display_desc.height/16)*8,
                                        data);
                        break;
                case UYVY:
                        gl_render_uyvy(s, data);
                        break;
                case RGBA:
                        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0,
                                        s->current_display_desc.width, s->current_display_desc.height,
                                        GL_RGBA, GL_UNSIGNED_BYTE,
                                        data);
                        break;
                case RGB:
                        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0,
                                        s->current_display_desc.width, s->current_display_desc.height,
                                        GL_RGB, GL_UNSIGNED_BYTE,
                                        data);
                        break;
                case DXT5:                        
                        glCompressedTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0,
                                        (s->current_display_desc.width + 3) / 4 * 4, s->dxt_height,
                                        GL_COMPRESSED_RGBA_S3TC_DXT5_EXT,
                                        (s->current_display_desc.width + 3) / 4 * 4 * s->dxt_height,
                                        data);
                        break;
                default:
                        fprintf(stderr, "[GL] Fatal error - received unsupported codec.\n");
                        exit_uv(128);
                        return;

        }

        gl_check_error();
}

static void glut_idle_callback(void)
{
        struct state_gl *s = gl;
        struct timeval tv;
        double seconds;
        struct video_frame *frame;

        s->lock.lock();
        if (s->frame_queue.size() == 0) {
                s->lock.unlock();
                return;
        }
        frame = s->frame_queue.front();
        s->frame_queue.pop();
        s->lock.unlock();
        s->frame_consumed_cv.notify_one();

        if (s->paused)
                return;

        if (s->current_frame) {
                s->lock.lock();
                s->free_frame_queue.push(s->current_frame);
                s->lock.unlock();
        }
        s->current_frame = frame;

        if (!video_desc_eq(video_desc_from_frame(frame), s->current_display_desc)) {
                gl_reconfigure_screen(s, video_desc_from_frame(frame));
        }

        gl_render(s, frame->tiles[0].data);
        gl_draw(s->aspect, (gl->dxt_height - gl->current_display_desc.height) / (float) gl->dxt_height * 2);
        glutPostRedisplay();

        /* FPS Data, this is pretty ghetto though.... */
        s->frames++;
        gettimeofday(&tv, NULL);
        seconds = tv_diff(tv, s->tv);

        if (seconds > 5) {
                double fps = s->frames / seconds;
                fprintf(stderr, "[GL] %lu frames in %g seconds = %g FPS\n", s->frames, seconds, fps);
                s->frames = 0;
                s->tv = tv;
        }
}

static void glut_key_callback(unsigned char key, int x, int y)
{
        UNUSED(x);
        UNUSED(y);

        switch(key) {
                case 'f':
                        gl->fs = !gl->fs;
                        glut_resize_window(gl->fs, gl->current_display_desc.height, gl->aspect,
                                        gl->window_size_factor);
                        break;
                case 'q':
#if defined FREEGLUT || defined HAVE_MACOSX
                        exit_uv(0);
#else
			glutDestroyWindow(gl->window);
			exit(1);
#endif
                        break;
                case 'd':
                        gl->deinterlace = !gl->deinterlace;
                        printf("Deinterlacing: %s\n", gl->deinterlace ? "ON" : "OFF");
                        break;
                case ' ':
                        gl->paused = !gl->paused;
                        break;
                case 's':
                        screenshot(gl->current_frame);
                        break;
                case 'm':
                        gl->show_cursor = !gl->show_cursor;
                        glutSetCursor(gl->show_cursor ? GLUT_CURSOR_CROSSHAIR : GLUT_CURSOR_NONE);
                        break;
                case '+':
                        gl->window_size_factor *= 2;
                        glut_resize_window(gl->fs, gl->current_display_desc.height, gl->aspect,
                                        gl->window_size_factor);
                        break;
                case '-':
                        gl->window_size_factor /= 2;
                        glut_resize_window(gl->fs, gl->current_display_desc.height, gl->aspect,
                                        gl->window_size_factor);
                        break;
        }
}

static bool display_gl_init_opengl(struct state_gl *s)
{
        char *tmp, *gl_ver_major;
        char *save_ptr = NULL;
#if defined HAVE_LINUX || defined WIN32
        GLenum err;
#endif // HAVE_LINUX

#ifdef HAVE_LINUX
        x11_enter_thread();
#endif

        glutInit(&uv_argc, uv_argv);
        glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);

#ifdef HAVE_MACOSX
        /* Startup function to call when running Cocoa code from a Carbon application. Whatever the fuck that means. */
        /* Avoids uncaught exception (1002)  when creating CGSWindow */
        NSApplicationLoad();
#endif

#ifdef FREEGLUT
        glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_CONTINUE_EXECUTION);
#endif
        glutIdleFunc(glut_idle_callback);
	s->window = glutCreateWindow(window_title != NULL ? window_title : DEFAULT_WIN_NAME);
        glutSetCursor(s->show_cursor ? GLUT_CURSOR_CROSSHAIR : GLUT_CURSOR_NONE);
        //glutHideWindow();
	glutKeyboardFunc(glut_key_callback);
	glutDisplayFunc((void (*)())glutSwapBuffers); // cast is needed because glutSwapBuffers is stdcall on MSW
#ifdef HAVE_MACOSX
        glutWMCloseFunc(glut_close_callback);
#elif FREEGLUT
        glutCloseFunc(glut_close_callback);
#endif
	glutReshapeFunc(gl_resize);

        tmp = strdup((const char *)glGetString(GL_VERSION));
        gl_ver_major = strtok_r(tmp, ".", &save_ptr);
        if(atoi(gl_ver_major) >= 2) {
                fprintf(stderr, "OpenGL 2.0 is supported...\n");
        } else {
                fprintf(stderr, "ERROR: OpenGL 2.0 is not supported, try updating your drivers...\n");
                free(tmp);
                return false;
        }
        free(tmp);

#if defined HAVE_LINUX || defined WIN32
        err = glewInit();
        if (GLEW_OK != err)
        {
                /* Problem: glewInit failed, something is seriously wrong. */
                fprintf(stderr, "GLEW Error: %d\n", err);
                return false;
        }
#endif /* HAVE_LINUX */

        glClearColor( 0.0f, 0.0f, 0.0f, 1.0f );
        glEnable( GL_TEXTURE_2D );

        glGenTextures(1, &s->texture_display);
        glBindTexture(GL_TEXTURE_2D, s->texture_display);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

        glGenTextures(1, &s->texture_uyvy);
        glBindTexture(GL_TEXTURE_2D, s->texture_uyvy);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

        s->PHandle_uyvy = glsl_compile_link(vert, yuv422_to_rgb_fp);
        // Create fbo
        glGenFramebuffersEXT(1, &s->fbo_id);
        s->PHandle_dxt = glsl_compile_link(vert, fp_display_dxt1);
        glUseProgram(s->PHandle_dxt);
        glUniform1i(glGetUniformLocation(s->PHandle_dxt,"yuvtex"),0);
        glUseProgram(0);
        s->PHandle_dxt5 = glsl_compile_link(vert, fp_display_dxt5ycocg);
        /*if (pthread_create(&(s->thread_id), NULL, display_thread_gl, (void *) s) != 0) {
          perror("Unable to create display thread\n");
          return NULL;
          }*/

        return true;
}

void display_gl_run(void *arg)
{
        struct state_gl *s = 
                (struct state_gl *) arg;

        if (!display_gl_init_opengl(s)) {
                exit_uv(1);
                return;
        }

#if defined HAVE_MACOSX || defined FREEGLUT
        while(!s->should_exit_main_loop) {
                usleep(1000);
                glut_idle_callback();
#ifndef HAVE_MACOSX
                glutMainLoopEvent();
#else
                glutCheckLoop();
#endif
        }
#else /* defined HAVE_MACOSX || defined FREEGLUT */
	glutMainLoop();
#endif
}


static void gl_resize(int width, int height)
{
        glViewport( 0, 0, ( GLint )width, ( GLint )height );
        glMatrixMode( GL_PROJECTION );
        glLoadIdentity( );

        double screen_ratio;
        double x = 1.0,
               y = 1.0;

        debug_msg("Resized to: %dx%d\n", width, height);

        screen_ratio = (double) width / height;
        if(screen_ratio > gl->aspect) {
                x = (double) height * gl->aspect / width;
        } else {
                y = (double) width / (height * gl->aspect);
        }
        glScalef(x, y, 1);

        glOrtho(-1,1,-1/gl->aspect,1/gl->aspect,10,-10);

        glMatrixMode( GL_MODELVIEW );

        glLoadIdentity( );

        if (gl->current_frame) {
                // redraw last frame
                for (int i = 0; i < 2; ++i) {
                        gl_render(gl, gl->current_frame->tiles[0].data);
                        gl_draw(gl->aspect, (gl->dxt_height - gl->current_display_desc.height) / (float) gl->dxt_height * 2);
                        glutSwapBuffers();
                }
                glutPostRedisplay();
        }
}

static void gl_render_uyvy(struct state_gl *s, char *data)
{
        int status;
        glBindTexture(GL_TEXTURE_2D, 0);
        glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, s->fbo_id);
        gl_check_error();
        glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_2D, s->texture_display, 0);
        status = glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT);
        assert(status == GL_FRAMEBUFFER_COMPLETE_EXT);
        glActiveTexture(GL_TEXTURE0 + 2);
        glBindTexture(GL_TEXTURE_2D, s->texture_uyvy);

        glMatrixMode( GL_PROJECTION );
        glPushMatrix();
        glLoadIdentity( );

        glMatrixMode( GL_MODELVIEW );
        glPushMatrix();
        glLoadIdentity( );

        glPushAttrib(GL_VIEWPORT_BIT);

        glViewport( 0, 0, s->current_display_desc.width, s->current_display_desc.height);

        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, s->current_display_desc.width / 2, s->current_display_desc.height,  GL_RGBA, GL_UNSIGNED_BYTE, data);
        glUseProgram(s->PHandle_uyvy);

        glDrawBuffer(GL_COLOR_ATTACHMENT0_EXT);
        gl_check_error();

        glBegin(GL_QUADS);
        glTexCoord2f(0.0, 0.0); glVertex2f(-1.0, -1.0);
        glTexCoord2f(1.0, 0.0); glVertex2f(1.0, -1.0);
        glTexCoord2f(1.0, 1.0); glVertex2f(1.0, 1.0);
        glTexCoord2f(0.0, 1.0); glVertex2f(-1.0, 1.0);
        glEnd();

        glPopAttrib();

        glMatrixMode( GL_PROJECTION );
        glPopMatrix();
        glMatrixMode( GL_MODELVIEW );
        glPopMatrix();

        glUseProgram(0);
        glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
        glActiveTexture(GL_TEXTURE0 + 0);
        glBindTexture(GL_TEXTURE_2D, s->texture_display);
}    

static void gl_draw(double ratio, double bottom_offset)
{
        float bottom;
        gl_check_error();

        /* Clear the screen */
        glClear(GL_COLOR_BUFFER_BIT);

        glLoadIdentity( );
        glTranslatef( 0.0f, 0.0f, -1.35f );

        /* Reflect that we may have higher texture than actual data
         * if we use DXT and source height was not divisible by 4 
         * In normal case, there would be 1.0 */
        bottom = 1.0f - bottom_offset;

        gl_check_error();
        glBegin(GL_QUADS);
        /* Front Face */
        /* Bottom Left Of The Texture and Quad */
        glTexCoord2f( 0.0f, bottom ); glVertex2f( -1.0f, -1/ratio);
        /* Bottom Right Of The Texture and Quad */
        glTexCoord2f( 1.0f, bottom ); glVertex2f(  1.0f, -1/ratio);
        /* Top Right Of The Texture and Quad */
        glTexCoord2f( 1.0f, 0.0f ); glVertex2f(  1.0f,  1/ratio);
        /* Top Left Of The Texture and Quad */
        glTexCoord2f( 0.0f, 0.0f ); glVertex2f( -1.0f,  1/ratio);
        glEnd( );

        gl_check_error();
}

static void glut_close_callback(void)
{
        gl->should_exit_main_loop = true;
        exit_uv(0);
}

display_type_t *display_gl_probe(void)
{
        display_type_t          *dt;

        dt = (display_type_t *) malloc(sizeof(display_type_t));
        if (dt != NULL) {
                dt->id          = DISPLAY_GL_ID;
                dt->name        = "gl";
                dt->description = "OpenGL";
        }
        return dt;
}

int display_gl_get_property(void *state, int property, void *val, size_t *len)
{
        UNUSED(state);
        codec_t codecs[] = {UYVY, RGBA, RGB, DXT1, DXT1_YUV, DXT5};
        enum interlacing_t supported_il_modes[] = {PROGRESSIVE, INTERLACED_MERGED, SEGMENTED_FRAME};
        int rgb_shift[] = {0, 8, 16};

        switch (property) {
                case DISPLAY_PROPERTY_CODECS:
                        if(sizeof(codecs) <= *len) {
                                memcpy(val, codecs, sizeof(codecs));
                        } else {
                                return FALSE;
                        }

                        *len = sizeof(codecs);
                        break;
                case DISPLAY_PROPERTY_RGB_SHIFT:
                        if(sizeof(rgb_shift) > *len) {
                                return FALSE;
                        }
                        memcpy(val, rgb_shift, sizeof(rgb_shift));
                        *len = sizeof(rgb_shift);
                        break;
                case DISPLAY_PROPERTY_BUF_PITCH:
                        *(int *) val = PITCH_DEFAULT;
                        *len = sizeof(int);
                        break;
                case DISPLAY_PROPERTY_SUPPORTED_IL_MODES:
                        if(sizeof(supported_il_modes) <= *len) {
                                memcpy(val, supported_il_modes, sizeof(supported_il_modes));
                        } else {
                                return FALSE;
                        }
                        *len = sizeof(supported_il_modes);
                        break;
                default:
                        return FALSE;
        }
        return TRUE;
}

void display_gl_done(void *state)
{
        struct state_gl *s = (struct state_gl *) state;

        assert(s->magic == MAGIC_GL);

        //pthread_join(s->thread_id, NULL);
        if(s->window != -1) {
                glutDestroyWindow(s->window);
        }
        if (s->PHandle_uyvy)
                glDeleteProgram(s->PHandle_uyvy);
        if (s->PHandle_dxt)
                glDeleteProgram(s->PHandle_dxt);
        if (s->PHandle_dxt5)
                glDeleteProgram(s->PHandle_dxt5);

        while (s->free_frame_queue.size() > 0) {
                struct video_frame *buffer = s->free_frame_queue.front();
                s->free_frame_queue.pop();
                vf_free(buffer);
        }

        while (s->frame_queue.size() > 0) {
                struct video_frame *buffer = s->frame_queue.front();
                s->frame_queue.pop();
                vf_free(buffer);
        }

        vf_free(s->current_frame);
        
        delete s;
}

struct video_frame * display_gl_getf(void *state)
{
        struct state_gl *s = (struct state_gl *) state;
        assert(s->magic == MAGIC_GL);

        lock_guard<mutex> lock(s->lock);

        while (s->free_frame_queue.size() > 0) {
                struct video_frame *buffer = s->free_frame_queue.front();
                s->free_frame_queue.pop();
                if (video_desc_eq(video_desc_from_frame(buffer), s->current_desc)) {
                        return buffer;
                } else {
                        vf_free(buffer);
                }
        }

        struct video_frame *buffer = vf_alloc_desc_data(s->current_desc);
        clear_video_buffer(reinterpret_cast<unsigned char *>(buffer->tiles[0].data),
                        vc_get_linesize(buffer->tiles[0].height, buffer->color_spec),
                        vc_get_linesize(buffer->tiles[0].height, buffer->color_spec),
                        buffer->tiles[0].height,
                        buffer->color_spec);
        return buffer;
}

int display_gl_putf(void *state, struct video_frame *frame, int nonblock)
{
        struct state_gl *s = (struct state_gl *) state;

        assert(s->magic == MAGIC_GL);

        if(!frame) {
                s->should_exit_main_loop = true;
                return 0;
        }

        std::unique_lock<std::mutex> lk(s->lock);
        if (s->frame_queue.size() >= MAX_BUFFER_SIZE && nonblock == PUTF_NONBLOCK) {
                s->free_frame_queue.push(frame);
                return 1;
        }
        s->frame_consumed_cv.wait(lk, [s]{return s->frame_queue.size() < MAX_BUFFER_SIZE;});
        s->frame_queue.push(frame);

        return 0;
}

void display_gl_put_audio_frame(void *state, struct audio_frame *frame)
{
        UNUSED(state);
        UNUSED(frame);
}

int display_gl_reconfigure_audio(void *state, int quant_samples, int channels,
                int sample_rate)
{
        UNUSED(state);
        UNUSED(quant_samples);
        UNUSED(channels);
        UNUSED(sample_rate);

        return FALSE;
}


