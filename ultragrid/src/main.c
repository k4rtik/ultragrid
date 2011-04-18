/*
 * FILE:    main.c
 * AUTHORS: Colin Perkins    <csp@csperkins.org>
 *          Ladan Gharai     <ladan@isi.edu>
 *          Martin Benes     <martinbenesh@gmail.com>
 *          Lukas Hejtmanek  <xhejtman@ics.muni.cz>
 *          Petr Holub       <hopet@ics.muni.cz>
 *          Milos Liska      <xliska@fi.muni.cz>
 *          Jiri Matela      <matela@ics.muni.cz>
 *          Dalibor Matura   <255899@mail.muni.cz>
 *          Ian Wesley-Smith <iwsmith@cct.lsu.edu>
 *
 * Copyright (c) 2005-2010 CESNET z.s.p.o.
 * Copyright (c) 2001-2004 University of Southern California
 * Copyright (c) 2003-2004 University of Glasgow
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
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 * 
 *      This product includes software developed by the University of Southern
 *      California Information Sciences Institute. This product also includes
 *      software developed by CESNET z.s.p.o.
 * 
 * 4. Neither the name of the University nor of the Institute may be used
 *    to endorse or promote products derived from this software without
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
 *
 * $Revision: 1.30 $
 * $Date: 2010/02/05 14:06:17 $
 *
 */

#include <string.h>
#include <getopt.h>
#include "config.h"
#include "config_unix.h"
#include "config_win32.h"
#include "debug.h"
#include "rtp/rtp.h"
#include "rtp/rtp_callback.h"
#include "rtp/pbuf.h"
#include "video_types.h"
#include "video_codec.h"
#include "video_capture.h"
#include "video_display.h"
#include "video_display/sdl.h"
#include "video_compress.h"
#include "pdb.h"
#include "tv.h"
#include "transmit.h"
#include "tfrc.h"
#include "version.h"
#include "ihdtv/ihdtv.h"
#include "compat/platform_semaphore.h"
#include "audio/audio.h"

#define EXIT_FAIL_USAGE		1
#define EXIT_FAIL_UI   		2
#define EXIT_FAIL_DISPLAY	3
#define EXIT_FAIL_CAPTURE	4
#define EXIT_FAIL_NETWORK	5
#define EXIT_FAIL_TRANSMIT	6


struct state_uv {
    struct rtp		*network_device;
	struct rtp		*audio_network_device;
	struct vidcap		*capture_device;
	struct timeval		 start_time, curr_time;
    struct pdb		*participants;
	struct pdb		*audio_participants;
    uint32_t		 ts;
	int			 fps;
    struct video_tx		*tx;
	struct display		*display_device;
	struct video_compress	*compression;
	const char		*requested_display;
	const char		*requested_capture;
	int			 requested_compression;
	int			 dxt_display;
	unsigned 		 requested_mtu;

	int	use_ihdtv_protocol;

	int	audio_capture_device;
	int	audio_playback_device;
};

long        packet_rate = 13600;
int	        should_exit = FALSE;
uint32_t  	RTT = 0;    /* this is computed by handle_rr in rtp_callback */
char		*frame_buffer = NULL;
uint32_t        hd_size_x = 2048;
uint32_t	hd_size_y = 1080;
uint32_t	hd_color_bpp = 4;
uint32_t	bitdepth = 8;
uint32_t	progressive = 0;

#ifdef HAVE_HDSTATION
#include <dvs_clib.h>
uint32_t	hd_video_mode=SV_MODE_SMPTE274_29I | SV_MODE_NBIT_10BDVS | SV_MODE_COLOR_YUV422 | SV_MODE_ACTIVE_STREAMER;
//uint32_t	hd_video_mode=SV_MODE_SMPTE274_25P | SV_MODE_NBIT_10BDVS | SV_MODE_COLOR_YUV422 | SV_MODE_ACTIVE_STREAMER;
#else
uint32_t	hd_video_mode=0;
#endif /* HAVE_HDSTATION */

long frame_begin[2];

#ifndef WIN32
static void
signal_handler(int signal)
{
	debug_msg("Caught signal %d\n", signal);
	should_exit = TRUE;
	return;
}
#endif /* WIN32 */

static void 
usage(void) 
{
    /* TODO -c -p -b are deprecated options */
	printf("Usage: uv [-d <display_device>] [-t <capture_device>] [-g <cfg>] [-m <mtu>] [-f <framerate>] [-c] [-p] [-i] [-b <8|10>] address\n\n");
    printf("\t-d <display_device>\tselect display device, use -d help to get\n");
    printf("\t                   \tlist of devices availabe\n");
    printf("\t-t <capture_device>\tselect capture device, use -t help to get\n");
    printf("\t                   \tlist of devices availabe\n");
    printf("\t-g <cfg>           \tconfigure capture/display device,\n");
    printf("\t                   \tuse -g help with a device to get info about\n");
    printf("\t                   \tsupported capture/display modes\n");
    printf("\t-i                 \tiHDTV compatibility mode\n");
}

static void
initialize_video_codecs(void)
{
	int		nc, i;

	vcodec_init();
	nc = vcodec_get_num_codecs();
	for (i = 0; i < nc; i++) {
		if (vcodec_can_encode(i)) {
			printf("Video encoder : %s\n", vcodec_get_description(i));
		}
		if (vcodec_can_decode(i)) {
			printf("Video decoder : %s\n", vcodec_get_description(i));
		}

		/* Map static and "well-known" dynamic payload types */
		//TODO: Is this where I would list DXT payload?
		if (strcmp(vcodec_get_name(i), "uv_yuv") == 0) {
			vcodec_map_payload(96, i);	/*  96 : Uncompressed YUV */
		}
	}
}

void
list_video_display_devices()
{
    int i;
	display_type_t		*dt;

    printf("Available display devices:\n");
	display_init_devices();
	for (i = 0; i < display_get_device_count(); i++) {
		dt = display_get_device_details(i);
        printf("\t%s\n", dt->name); 
	}
	display_free_devices();
}

static struct display *
initialize_video_display(const char *requested_display, char *fmt)
{
	struct display		*d;
	display_type_t		*dt;
	display_id_t		 id = display_get_null_device_id();
	int			 i;

	if (display_init_devices() != 0) {
		printf("Unable to initialise devices\n");
		abort();
	} else {
		debug_msg("Found %d display devices\n", display_get_device_count());
	}
	for (i = 0; i < display_get_device_count(); i++) {
		dt = display_get_device_details(i);
		if (strcmp(requested_display, dt->name) == 0) {
			id = dt->id;
			debug_msg("Found device\n");
		} else {
			debug_msg("Device %s does not match %s\n", dt->name, requested_display);
		}
	}
	display_free_devices();

	d = display_init(id, fmt);
	if (d != NULL) {
		frame_buffer = display_get_frame(d);
	}
	return d;
}

void
list_video_capture_devices()
{
    int i;
	struct vidcap_type	*vt;

    printf("Available capture devices:\n");
	vidcap_init_devices();
	for (i = 0; i < vidcap_get_device_count(); i++) {
		vt = vidcap_get_device_details(i);
        printf("\t%s\n", vt->name); 
	}
	vidcap_free_devices();
}

static struct vidcap *
initialize_video_capture(const char *requested_capture, char *fmt)
{
	struct vidcap_type	*vt;
	vidcap_id_t		 id = vidcap_get_null_device_id();
	int			 i;

	vidcap_init_devices();
	for (i = 0; i < vidcap_get_device_count(); i++) {
		vt = vidcap_get_device_details(i);
		if (strcmp(vt->name, requested_capture) == 0) {
			id = vt->id;
		}
	}
	vidcap_free_devices();

	return vidcap_init(id, fmt);
}

static struct rtp *
initialize_network(char *addr, struct pdb *participants)
{
	struct rtp 	*r;
	double		 rtcp_bw = 5 * 1024 * 1024;	/* FIXME */
	
	char *tmp;
    tmp=strtok(addr,"/");
    tmp=strtok(NULL,"/");
    
    r = rtp_init(addr, atoi(tmp), atoi(tmp), 255, rtcp_bw, FALSE, rtp_recv_callback, (void *) participants);
	
    if (r != NULL) {
		pdb_add(participants, rtp_my_ssrc(r));
		rtp_set_option(r, RTP_OPT_WEAK_VALIDATION,   TRUE);
		rtp_set_sdes(r, rtp_my_ssrc(r), RTCP_SDES_TOOL, ULTRAGRID_VERSION, strlen(ULTRAGRID_VERSION));
	}
	return r;
}

static struct video_tx *
initialize_transmit(unsigned requested_mtu)
{
	/* Currently this is trivial. It'll get more complex once we */
	/* have multiple codecs and/or error correction.             */
	return tx_init(requested_mtu);
}

static void *
ihdtv_reciever_thread(void *arg)
{
	ihdtv_connection *connection = (ihdtv_connection*) ((void**)arg)[0];
	struct display *display_device = (struct display*) ((void**)arg)[1];

	while(!should_exit)
	{
		if(ihdtv_recieve(connection, frame_buffer, hd_size_x * hd_size_y * 3))
			return 0;       // we've got some error. probably empty buffer
		display_put_frame(display_device, frame_buffer);
		frame_buffer = display_get_frame(display_device);
	}
	return 0;
}


static void *
ihdtv_sender_thread(void *arg)
{
	ihdtv_connection *connection = (ihdtv_connection*) ((void**)arg)[0];
	struct vidcap           *capture_device = (struct vidcap*) ((void**)arg)[1];
	struct video_frame *tx_frame;

	while(!should_exit){
	       if((tx_frame = vidcap_grab(capture_device)) != NULL)
	       {
		ihdtv_send(connection, tx_frame, 9000000); // FIXME: fix the use of frame size!!
		free(tx_frame);
	       }
	       else
	       {
		fprintf(stderr,"Error recieving frame from capture device\n");
		return 0;
	       }
	}

	return 0;
}

#ifdef HAVE_AUDIO
static struct rtp *
initialize_audio_network(char *addr, struct pdb *participants)	// GiX
{
	struct rtp *r;
	double	rtcp_bw = 1024*512;	// FIXME:  something about 5% for rtcp is said in rfc

	r = rtp_init(addr, 5006, 5006, 255, rtcp_bw, FALSE, rtp_recv_callback, (void *) participants);
	if(r != NULL)
	{
		pdb_add(participants, rtp_my_ssrc(r));
		rtp_set_option(r, RTP_OPT_WEAK_VALIDATION,   TRUE);
		rtp_set_sdes(r, rtp_my_ssrc(r), RTCP_SDES_TOOL, ULTRAGRID_VERSION, strlen(ULTRAGRID_VERSION));
	}

	return r;
}

static void *
audio_thread(void *arg)
{
	struct state_uv *uv = arg;
	audio_init(uv->audio_playback_device, uv->audio_capture_device);

	// we act as a receiver
	if(uv->audio_playback_device != -2)
	{
		audio_frame_buffer buffer;
		int size_of_frame_buffer = 5;
		init_audio_frame_buffer(&buffer, size_of_frame_buffer);
//		int buffer_underrun = 1;	// so far we only use buffer at the beginning

		// rtp variables
		struct timeval timeout, curr_time;
		uint32_t ts;
		struct pdb_e	*cp;


		audio_start_stream();
		while(!should_exit)
		{
			timeout.tv_sec = 0;
			timeout.tv_usec = 999999/((48000/audio_samples_per_frame)*2);
			gettimeofday(&curr_time, NULL);
			ts = tv_diff(curr_time, uv->start_time) * 90000;	// What is this?
			rtp_update(uv->audio_network_device, curr_time);	// this is just some internal rtp housekeeping...nothing to worry about
			rtp_send_ctrl(uv->audio_network_device, ts, 0, curr_time);	// strange..

			// initially we wait to fill the buffer -- probably should be completely deleted..
			/*
			if(buffer_underrun)
			{
				while(!audio_buffer_full(&buffer))
				{
					gettimeofday(&curr_time, NULL);
					ts = tv_diff(curr_time, uv->start_time) * 90000;
					rtp_update(uv->audio_network_device, curr_time);
					rtp_send_ctrl(uv->audio_network_device, ts, 0, curr_time);

					timeout.tv_sec = 0;
					timeout.tv_usec = 999999/((48000/audio_samples_per_frame)*2);
					rtp_recv(uv->audio_network_device, &timeout, ts);
					cp = pdb_iter_init(uv->audio_participants);

					while((cp != NULL) && (!audio_buffer_full(&buffer)))
					{
				    	audio_frame *frame = audio_buffer_get_empty_frame(&buffer);
						if(audio_pbuf_decode(cp->playout_buffer, curr_time, frame))
							audio_buffer_mark_last_frame_full(&buffer);
						pbuf_remove(cp->playout_buffer, curr_time);
						cp = pdb_iter_next(uv->audio_participants);
					}

					pdb_iter_done(uv->audio_participants);
				}

				buffer_underrun = 0;
				audio_start_stream();
			}
			*/

			rtp_recv(uv->audio_network_device, &timeout, ts);
			cp = pdb_iter_init(uv->audio_participants);

			while((cp != NULL) && (!audio_buffer_full(&buffer)))
			{
				audio_frame *frame = audio_buffer_get_empty_frame(&buffer);
				if(audio_pbuf_decode(cp->playout_buffer, curr_time, frame))
					audio_buffer_mark_last_frame_full(&buffer);

				pbuf_remove(cp->playout_buffer, curr_time);
				cp = pdb_iter_next(uv->audio_participants);
			}
			pdb_iter_done(uv->audio_participants);

			while(audio_ready_to_write() >= audio_samples_per_frame)
			{
				audio_frame *frame = audio_buffer_get_full_frame(&buffer);

				if(frame != NULL)
					audio_write(frame);

				else
					break;	// audio buffer is empty...nead to read some data
			}
		}
		free_audio_frame_buffer(&buffer);
	}

	else	// we act as a sender
	if(uv->audio_capture_device != -2)
	{
		audio_frame buffer;
		init_audio_frame(&buffer, audio_samples_per_frame);
		audio_start_stream();
		while(!should_exit)
		{
			audio_read(&buffer);
			audio_tx_send(uv->audio_network_device, &buffer);
		}
		free_audio_frame(&buffer);
	}


	audio_close();
	return NULL;
}
#endif /* HAVE_AUDIO */

static void *
receiver_thread(void *arg)
{
	struct state_uv *uv = (struct state_uv *) arg;

	struct pdb_e		*cp;
	struct timeval           timeout;
	int                      fr;
	int 			 i = 0;
	int                      ret;

	fr = 1;

	while (!should_exit) {
		/* Housekeeping and RTCP... */
		gettimeofday(&uv->curr_time, NULL);
		uv->ts = tv_diff(uv->curr_time, uv->start_time) * 90000;
		rtp_update(uv->network_device, uv->curr_time);
		rtp_send_ctrl(uv->network_device, uv->ts, 0, uv->curr_time);

		/* Receive packets from the network... The timeout is adjusted */
		/* to match the video capture rate, so the transmitter works.  */
		if(fr) {
			gettimeofday(&uv->curr_time, NULL);
			frame_begin[i] = uv->curr_time.tv_usec;
			fr=0;
		}

		timeout.tv_sec  = 0;
        timeout.tv_usec = 999999 / 59.94;
		ret = rtp_recv(uv->network_device, &timeout, uv->ts);
		
		/*
		if (ret == FALSE) {
			printf("Failed to receive data\n");
		}
		*/

		/* Decode and render for each participant in the conference... */
		cp = pdb_iter_init(uv->participants);
		while (cp != NULL) { 
			if (tfrc_feedback_is_due(cp->tfrc_state, uv->curr_time)) {
				debug_msg("tfrc rate %f\n", 
						tfrc_feedback_txrate(cp->tfrc_state, uv->curr_time));
			}

			/* Decode and render video... */
			if (pbuf_decode(cp->playout_buffer, uv->curr_time, frame_buffer, i, uv->dxt_display)) {
				gettimeofday(&uv->curr_time, NULL);
				fr = 1;
				display_put_frame(uv->display_device, frame_buffer);
				i = (i + 1) % 2;
				frame_buffer = display_get_frame(uv->display_device);
			}
			pbuf_remove(cp->playout_buffer, uv->curr_time);
			cp = pdb_iter_next(uv->participants);
		}
		pdb_iter_done(uv->participants);
	}

	return 0;
}

static void *
sender_thread(void *arg)
{
	struct state_uv *uv = (struct state_uv *) arg;

	struct video_frame	*tx_frame;

	while (!should_exit) {
		/* Capture and transmit video... */
		tx_frame = vidcap_grab(uv->capture_device);
		if (tx_frame != NULL) {
			//TODO: Unghetto this
			if(uv->requested_compression) {
#ifdef HAVE_FASTDXT
				compress_data(uv->compression,tx_frame);
				dxt_tx_send(uv->tx, tx_frame, uv->network_device);
#endif /* HAVE_FASTDXT */
			}else{
                tx_send(uv->tx, tx_frame, uv->network_device);
			}
			free(tx_frame);
		}
	}

	return 0;
}

int 
check_cfg(char* cfg)
{
    int  number=1;
    char *tmp=cfg;
    
    tmp++;
    while(tmp[0]==':')
    {    
        number++;
        tmp+=2;
    }
    return number;
}

int 
main(int argc, char *argv[])
{

#ifdef HAVE_SCHED_SETSCHEDULER
	struct sched_param	 sp;
#endif
	char                *cfg=NULL;
	struct state_uv     *uv[4];
    int			 ch;
    int          i; 
	int          number_of_devices;
    int          res;
    pthread_t	 receiver_thread_id;
    pthread_t    sender_thread_id[4];

	static struct option 	 getopt_options[] = {
		{"display", required_argument, 0, 'd'},
		{"cfg", required_argument, 0, 'g'},
		{"capture", required_argument, 0, 't'},
		{"mtu", required_argument, 0, 'm'},
		{"fps", required_argument, 0, 'f'},
		{"bitdepth", required_argument, 0, 'b'},
		{"version", no_argument, 0, 'v'},
		{"compress", no_argument, 0, 'c'},
		{"progressive", no_argument, 0, 'p'},
		{"ihdtv", no_argument, 0, 'i'},
		{"receive", required_argument, 0, 'r'},
		{"send", required_argument, 0, 's'},
		{0, 0, 0, 0}
	};
	int			 option_index = 0;

   for (i=0;i<4;i++)
   {
        uv[i] = (struct state_uv *) malloc(sizeof(struct state_uv));

        uv[i]->ts = 0;
        uv[i]->fps = 60;
        uv[i]->display_device = NULL;
        uv[i]->compression = NULL;
        uv[i]->requested_display = "none";
        uv[i]->requested_capture = "none";
        uv[i]->requested_compression = 0;
        uv[i]->requested_mtu = 0;
        uv[i]->use_ihdtv_protocol = 0;
        uv[i]->audio_capture_device = -2;	// no device
        uv[i]->audio_playback_device = -2;
        uv[i]->audio_participants = NULL;
        uv[i]->participants = NULL;
   }
#ifdef HAVE_AUDIO
	while ((ch = getopt_long(argc, argv, "d:g:t:m:f:b:r:s:vcpi", getopt_options, &option_index)) != -1) {
#else
	while ((ch = getopt_long(argc, argv, "d:g:t:m:f:b:vcpi", getopt_options, &option_index)) != -1) {
#endif /* HAVE_AUDIO */
		switch (ch) {
		case 'd' :
            for(i=0;i<4;i++)
			    uv[i]->requested_display = optarg;
            if(!strcmp(uv[0]->requested_display, "help")) {
                list_video_display_devices();
                return 0;
            }
			if(!strcmp(uv[0]->requested_display,"dxt")){
                for(i=0;i<4;i++)
                    uv[i]->dxt_display = 1;
            }    
			break;
		case 't' :
            for(i=0;i<4;i++)
			    uv[i]->requested_capture = optarg;
            if(!strcmp(uv[0]->requested_capture, "help")) {
                list_video_capture_devices();
                return 0;
            }
			break;
		case 'm' :
            for(i=0;i<4;i++)
                uv[i]->requested_mtu = atoi(optarg);
			break;
		case 'g' :
			cfg = strdup(optarg);
			break;
		case 'f' :
            for(i=0;i<4;i++)
                uv[i]->fps = atoi(optarg);
			break;
		case 'b' :
			bitdepth = atoi(optarg);
			if (bitdepth != 10 && bitdepth != 8) {
				usage();
				return EXIT_FAIL_USAGE;
			}
			if (bitdepth == 8) {
				hd_color_bpp = 2;
#ifdef HAVE_HDSTATION
			hd_video_mode=SV_MODE_SMPTE274_29I | SV_MODE_COLOR_YUV422 | SV_MODE_ACTIVE_STREAMER;
#endif /* HAVE_HDSTATION */
			}
			break;
		case 'v' :
			printf("%s\n", ULTRAGRID_VERSION);
			return EXIT_SUCCESS;
		case 'c' :
            for(i=0;i<4;i++)
			    uv[i]->requested_compression=1;
			break;
		case 'p' :
			progressive=1;
			break;
		case 'i':
            for(i=0;i<4;i++)
			    uv[i]->use_ihdtv_protocol = 1;
			printf("setting ihdtv protocol\n");
			break;
#ifdef HAVE_AUDIO
		case 'r':
			if(!strcmp("list", optarg)){
				print_available_devices();
				return EXIT_SUCCESS;
			}
            for(i=0;i<4;i++)
                uv[i]->audio_playback_device = atoi(optarg);
			break;
		case 's':
			if(!strcmp("list", optarg)){
				print_available_devices();
				return EXIT_SUCCESS;
			}
            for(i=0;i<4;i++)
                uv[i]->audio_capture_device = atoi(optarg);
			break;
#endif /* HAVE_AUDIO */
		case '?' :
			break;
		default :
			usage();
			return EXIT_FAIL_USAGE;
		}
	}

	if(cfg == NULL) {
        cfg=malloc(4);
		snprintf(cfg, 4, "%d", uv[0]->fps);
	}

    number_of_devices=1;

#ifdef HAVE_QUAD    
    number_of_devices=check_cfg(cfg);
#endif  

    for (i=3;i>=number_of_devices;i--)
        free(uv[i]);

    argc -= optind;
	argv += optind;
/*
	if(uv[0]->use_ihdtv_protocol)
	{
		if((argc != 0) && (argc != 1) && (argc != 2))
		{
			usage();
			return EXIT_FAIL_USAGE;
		}
	}
	else if (argc != number_of_devices  && strstr(cfg, "help")==0) {
		usage();
		return EXIT_FAIL_USAGE;
	}
*/
	printf("%s\n", ULTRAGRID_VERSION);
    printf("Display device: %s\n", uv[0]->requested_display);
	printf("Capture device: %s\n", uv[0]->requested_capture);
	printf("Frame rate    : %d\n", uv[0]->fps);
	printf("MTU           : %d\n", uv[0]->requested_mtu);
	if(uv[0]->requested_compression)
		printf("Compression   : DXT\n");
	else
		printf("Compression   : None\n");

	if(uv[0]->use_ihdtv_protocol)
		printf("Network protocol: ihdtv\n");
	else
		printf("Network protocol: ultragrid rtp\n");

    for(i=0;i<number_of_devices;i++)
    	gettimeofday(&(uv[i]->start_time), NULL);

	initialize_video_codecs();
	
    for(i=0;i<number_of_devices;i++)
        uv[i]->participants = pdb_init();

    
    char *temp;
    temp=NULL;

	if(temp == NULL) {
        temp=malloc(4);
	    memcpy(temp,cfg,strlen(cfg));
    }

    for (i=0;i<number_of_devices;i++)
    {
        if ((uv[i]->capture_device = initialize_video_capture(uv[i]->requested_capture, cfg)) == NULL) {
            printf("Unable to open capture device: %s number %d\n", uv[i]->requested_capture, atoi(cfg));
            return EXIT_FAIL_CAPTURE;
        }
	    printf("Video capture number %d initialized-%s\n", atoi(cfg), uv[i]->requested_capture);
        //temp=strtok(NULL,":");
        cfg+=2;
    }

	if ((uv[0]->display_device = initialize_video_display(uv[0]->requested_display, temp)) == NULL) {
		printf("Unable to open display device: %s\n", uv[0]->requested_display);
		return EXIT_FAIL_DISPLAY;
	}
	printf("Display initialized-%s\n", uv[0]->requested_display);

#ifdef HAVE_FASTDXT
	if (uv[0]->requested_compression) {
        for(i=0;i<4;i++)
            uv[i]->compression = initialize_video_compression();
	}
#endif /* HAVE_FASTDXT */

#ifndef WIN32
	signal(SIGINT, signal_handler);
	signal(SIGTERM, signal_handler);
	signal(SIGQUIT, signal_handler);
	signal(SIGHUP, signal_handler);
	signal(SIGABRT, signal_handler);
#endif

#ifdef HAVE_SCHED_SETSCHEDULER
	sp.sched_priority = sched_get_priority_max(SCHED_FIFO);
	if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0) {
		printf("WARNING: Unable to set real-time scheduling\n");
	}
#else
	printf("WARNING: System does not support real-time scheduling\n");
#endif /* HAVE_SCHED_SETSCHEDULER */


/*Edit HAVE_AUDIO to use more devices!!!*/    
#ifdef HAVE_AUDIO
	pthread_t audio_thread_id;
	if( (uv[0]->audio_playback_device != -2) || (uv[0]->audio_capture_device != -2))
	{
		uv[0]->audio_participants = pdb_init();
		if(( uv[0]->audio_network_device = initialize_audio_network(argv[0], uv[0]->audio_participants)) == NULL)
		{
			printf("Unable to open audio network\n");
			return EXIT_FAIL_NETWORK;
		}

		if(pthread_create(&audio_thread_id, NULL, audio_thread, (void *) uv) != 0)
		{
			fprintf(stderr, "Error creating audio thread. Quitting\n");
			return EXIT_FAILURE;
		}
	}
#endif /* HAVE_AUDIO */

    /*Edit ihdtv_protocol to use more devices!!!*/    
	if(uv[0]->use_ihdtv_protocol) {
		ihdtv_connection tx_connection, rx_connection;

		printf("Initializing ihdtv protocol\n");

		// we cannot act as both together, because parameter parsing whould have to be revamped
		if ((strcmp("none", uv[0]->requested_display) != 0) && (strcmp("none", uv[0]->requested_capture) != 0))
		{
			printf("Error: cannot act as both sender and reciever together in ihdtv mode\n");
			return -1;
		}

		void *rx_connection_and_display[] = {(void*)&rx_connection, (void*)uv[0]->display_device};
		void *tx_connection_and_display[] = {(void*)&tx_connection, (void*)uv[0]->capture_device};

		if(uv[0]->requested_mtu == 0)  // mtu not specified on command line
		{
			uv[0]->requested_mtu = 8112;   // not really a mtu, but a video-data payload per packet
		}

		if (strcmp("none", uv[0]->requested_display) != 0)
		{
			if(ihdtv_init_rx_session(&rx_connection, (argc==0)?NULL:argv[0], (argc==0)?NULL:( (argc==1)?argv[0]:argv[1] ), 3000, 3001, uv[0]->requested_mtu) != 0 )
			{
				fprintf(stderr, "Error initializing reciever session\n");
				return 1;
			}

			if (pthread_create(&receiver_thread_id, NULL, ihdtv_reciever_thread, rx_connection_and_display) != 0)
			{
				fprintf(stderr, "Error creating reciever thread. Quitting\n");
				return 1;
			}
		}

		if (strcmp("none", uv[0]->requested_capture) != 0)
		{
			if(argc == 0)
			{
				fprintf(stderr, "Error: specify the destination address\n");
				usage();
				return EXIT_FAIL_USAGE;
			}

			if(ihdtv_init_tx_session(&tx_connection, argv[0], (argc==2)?argv[1]:argv[0], uv[0]->requested_mtu) != 0)
			{
				fprintf(stderr, "Error initializing sender session\n");
				return 1;
			}

			if (pthread_create(&(sender_thread_id[0]), NULL, ihdtv_sender_thread, tx_connection_and_display) != 0)
			{
				fprintf(stderr, "Error creating sender thread. Quitting\n");
				return 1;
			}
		}

		while(!should_exit)
			sleep(1);

	}
	else {
        for(i=0;i<number_of_devices;i++)
        {
            if ((uv[i]->network_device = initialize_network(argv[i], uv[i]->participants)) == NULL) {
                printf("Unable to open network number %d\n",i);
                return EXIT_FAIL_NETWORK;
            }
		}
        
        for(i=0;i<number_of_devices;i++)
        {
            if(uv[i]->requested_mtu == 0)  // mtu wasn't specified on the command line
            {
                uv[i]->requested_mtu = 1500;   // the default value for rpt
            }
        }
        for(i=0;i<number_of_devices;i++)
        {
            if ((uv[i]->tx = initialize_transmit(uv[i]->requested_mtu)) == NULL) {
			printf("Unable to initialize transmitter number %d\n",i);
			return EXIT_FAIL_TRANSMIT;
            }
        }

		if (strcmp("none", uv[0]->requested_display) != 0) {
			if(pthread_create(&receiver_thread_id, NULL, receiver_thread, (void *) uv[0]) != 0) {
				perror("Unable to create display thread!\n");
				should_exit = TRUE;
			}
		}

        for(i=0;i<number_of_devices;i++)
        {
            if (strcmp("none", uv[i]->requested_capture) != 0) {
                if(pthread_create(&(sender_thread_id[i]), NULL, sender_thread, (void *) uv[i]) != 0) {
                    perror("Unable to create capture thread!\n");
                    should_exit = TRUE;
                }
            }
        }
		while (!should_exit) {
#ifndef X_DISPLAY_MISSING		
#ifdef HAVE_SDL
			if (strcmp(uv[0]->requested_display, "sdl") == 0) {
				display_sdl_handle_events(uv[0]->display_device);
			}
#endif /* HAVE_SDL */
#endif /* X_DISPLAY_MISSING */
			usleep(500000);

		}
	}

	if(strcmp("none", uv[0]->requested_display) != 0)
		pthread_join(receiver_thread_id, NULL);

    for(i=0;i<number_of_devices;i++)
    {
        if(strcmp("none", uv[i]->requested_capture) != 0)
	    	pthread_join(sender_thread_id[i], NULL);
    }

#ifdef HAVE_AUDIO
	if((uv[0]->audio_playback_device != -2) || (uv[0]->audio_capture_device != -2))
		pthread_join(audio_thread_id, NULL);
#endif /* HAVE_AUDIO */

    display_done(uv[0]->display_device);
    for(i=0;i<number_of_devices;i++)
    {
        tx_done(uv[i]->tx);
        rtp_done(uv[i]->network_device);
        vidcap_done(uv[i]->capture_device);
        if(uv[i]->participants != NULL)
            pdb_destroy(&(uv[i]->participants));
        if(uv[i]->audio_participants != NULL)
            pdb_destroy(&(uv[i]->audio_participants));
        free(uv[i]);
    }
	vcodec_done();
    free(temp);
    printf("Exit\n");
	return EXIT_SUCCESS;
}
