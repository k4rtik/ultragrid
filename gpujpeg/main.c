/**
 * Copyright (c) 2011, CESNET z.s.p.o
 * Copyright (c) 2011, Silicon Genome, LLC.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "libgpujpeg/gpujpeg.h"
#include "libgpujpeg/gpujpeg_util.h"

#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <string.h>
#include <strings.h>

void
print_help() 
{
    printf(
        "gpujpeg [options] input.rgb output.jpg [input2.rgb output2.jpg ...]\n"
        "   -h, --help\t\t\tprint help\n"
        "   -s, --size\t\t\tset image size in pixels, e.g. 1920x1080\n"
        "   -f, --sampling-factor\tset image sampling factor, e.g. 4:2:2\n"
        "   -q, --quality\t\tset quality level 1-100 (default 75)\n"
        "   -r, --restart\t\tset restart interval (default 8)\n"
        "   -e, --encode\t\t\tencode images\n"
        "   -d, --decode\t\t\tdecode images\n"
        "   -D, --device\t\t\tcuda device id (default 0)\n"
    );
}

int
main(int argc, char *argv[])
{       
    struct option longopts[] = {
        {"help",            no_argument,       0, 'h'},
        {"size",            required_argument, 0, 's'},
        {"sampling-factor", required_argument, 0, 'f'},
        {"quality",         required_argument, 0, 'q'},
        {"restart",         required_argument, 0, 'r'},
        {"encode",          no_argument,       0, 'e'},
        {"decode",          no_argument,       0, 'd'},
        {"device",          required_argument, 0, 'D'},
    };

    // Default image parameters
    struct gpujpeg_image_parameters param_image;
    gpujpeg_image_set_default_parameters(&param_image);
    
    // Default encoder parameters
    struct gpujpeg_encoder_parameters param_encoder;
    gpujpeg_encoder_set_default_parameters(&param_encoder);   
    
    // Other parameters
    int encode = 0;
    int decode = 0;
    int device_id = 0;
    
    // Parse command line
    char ch = '\0';
    int optindex = 0;
    char* pos = 0;
    while ( (ch = getopt_long(argc, argv, "hs:q:r:ed", longopts, &optindex)) != -1 ) {
        switch (ch) {
        case 'h':
            print_help();
            return 0;
        case 's':
            param_image.width = atoi(optarg);
            pos = strstr(optarg, "x");
            if ( pos == NULL || param_image.width == 0 || (strlen(pos) >= strlen(optarg)) ) {
                print_help();
                return -1;
            }
            param_image.height = atoi(pos + 1);
            break;
        case 'f':
            if ( strcmp(optarg, "4:4:4") == 0 )
                param_image.sampling_factor = GPUJPEG_4_4_4;
            else if ( strcmp(optarg, "4:2:2") == 0 )
                param_image.sampling_factor = GPUJPEG_4_2_2;
            else
                fprintf(stderr, "Sampling factor '%s' is not available!\n", optarg);
            break;
        case 'q':
            param_encoder.quality = atoi(optarg);
            if ( param_encoder.quality <= 0 )
                param_encoder.quality = 1;
            if ( param_encoder.quality > 100 )
                param_encoder.quality = 100;
            break;
        case 'r':
            param_encoder.restart_interval = atoi(optarg);
            if ( param_encoder.restart_interval < 0 )
                param_encoder.restart_interval = 0;
            break;
        case 'e':
            encode = 1;
            break;
        case 'd':
            decode = 1;
            break;
        case 'D':
            device_id = atoi(optarg);
            break;
        case '?':
            return -1;
        default:
            print_help();
            return -1;
        }
    }
	argc -= optind;
	argv += optind;
    
    // Source image and target image must be presented
    if ( argc < 2 ) {
        fprintf(stderr, "Please supply source and destination image filename!\n");
        print_help();
        return -1;
    }
    
    // Detect action if none is specified
    if ( encode == 0 && decode == 0 ) {
        enum gpujpeg_image_file_format input_format = gpujpeg_image_get_file_format(argv[0]);
        enum gpujpeg_image_file_format output_format = gpujpeg_image_get_file_format(argv[1]);
        if ( input_format & GPUJPEG_IMAGE_FILE_RAW && output_format == GPUJPEG_IMAGE_FILE_JPEG ) {
            encode = 1;
        } else if ( input_format == GPUJPEG_IMAGE_FILE_JPEG && output_format & GPUJPEG_IMAGE_FILE_RAW ) {
            decode = 1;
        } else {
            fprintf(stderr, "Action can't be recognized for specified images!\n");
            fprintf(stderr, "You must specify --encode or --decode option!\n");
            return -1;
        }
    }
    
    // Init device
    gpujpeg_init_device(device_id, 1);
    
    // Detect color spalce
    if ( gpujpeg_image_get_file_format(argv[0]) == GPUJPEG_IMAGE_FILE_YUV )
        param_image.color_space = GPUJPEG_YCBCR_ITU_R;
    
    if ( encode == 1 ) {    
        // Create encoder
        struct gpujpeg_encoder* encoder = gpujpeg_encoder_create(&param_image, &param_encoder);
        if ( encoder == NULL ) {
            fprintf(stderr, "Failed to create encoder!\n");
            return -1;
        }
        
        // Encode images
        for ( int index = 0; index < argc; index += 2 ) {
            // Get and check input and output image
            const char* input = argv[index];
            const char* output = argv[index + 1];
            enum gpujpeg_image_file_format input_format = gpujpeg_image_get_file_format(input);
            enum gpujpeg_image_file_format output_format = gpujpeg_image_get_file_format(output);
            if ( (input_format & GPUJPEG_IMAGE_FILE_RAW) == 0 ) {
                fprintf(stderr, "Encoder input file [%s] should be RGB image (*.rgb)!\n", input);
                return -1;
            }
            if ( output_format != GPUJPEG_IMAGE_FILE_JPEG ) {
                fprintf(stderr, "Encoder output file [%s] should be JPEG image (*.jpg)!\n", output);
                return -1;
            }                
            
            // Encode image
            GPUJPEG_TIMER_INIT();
            GPUJPEG_TIMER_START();
            
            printf("\nEncoding Image [%s]\n", input);
        
            // Load image
            int image_size = param_image.width * param_image.height * param_image.comp_count;
            if ( param_image.sampling_factor == GPUJPEG_4_2_2 ) {
                assert(param_image.comp_count == 3);
                image_size = image_size / 3 * 2;
            }
            uint8_t* image = NULL;
            if ( gpujpeg_image_load_from_file(input, &image, &image_size) != 0 ) {
                fprintf(stderr, "Failed to load image [%s]!\n", argv[index]);
                return -1;
            }
            
            GPUJPEG_TIMER_STOP_PRINT("Load Image:         ");
            GPUJPEG_TIMER_START();
                
            // Encode image
            uint8_t* image_compressed = NULL;
            int image_compressed_size = 0;
            if ( gpujpeg_encoder_encode(encoder, image, &image_compressed, &image_compressed_size) != 0 ) {
                fprintf(stderr, "Failed to encode image [%s]!\n", argv[index]);
                return -1;
            }
            
            GPUJPEG_TIMER_STOP_PRINT("Encode Image:       ");
            GPUJPEG_TIMER_START();
            
            // Save image
            if ( gpujpeg_image_save_to_file(output, image_compressed, image_compressed_size) != 0 ) {
                fprintf(stderr, "Failed to save image [%s]!\n", argv[index]);
                return -1;
            }
            
            GPUJPEG_TIMER_STOP_PRINT("Save Image:         ");
            
            printf("Compressed Size:     %d bytes [%s]\n", image_compressed_size, output);
            
            // Destroy image
            gpujpeg_image_destroy(image);
        }
        
        // Destroy encoder
        gpujpeg_encoder_destroy(encoder);
    }
    
    // Output sampling factor is always 4:4:4
    param_image.sampling_factor = GPUJPEG_4_4_4;
    
    if ( decode == 1 ) {    
        // Create decoder
        struct gpujpeg_decoder* decoder = gpujpeg_decoder_create(&param_image);
        if ( decoder == NULL ) {
            fprintf(stderr, "Failed to create decoder!\n");
            return -1;
        }
        
        // Decode images
        for ( int index = 0; index < argc; index += 2 ) {
            // Get and check input and output image
            const char* input = argv[index];
            const char* output = argv[index + 1];
            if ( encode == 1 ) {
                static char buffer_output[255];
                if ( param_image.color_space == GPUJPEG_YCBCR_ITU_R )
                    sprintf(buffer_output, "%s.decoded.yuv", output);
                else
                    sprintf(buffer_output, "%s.decoded.rgb", output);
                input = output;
                output = buffer_output;
            }
            enum gpujpeg_image_file_format input_format = gpujpeg_image_get_file_format(input);
            enum gpujpeg_image_file_format output_format = gpujpeg_image_get_file_format(output);
            if ( input_format != GPUJPEG_IMAGE_FILE_JPEG ) {
                fprintf(stderr, "Encoder input file [%s] should be JPEG image (*.jpg)!\n", input);
                return -1;
            }
            if ( (output_format & GPUJPEG_IMAGE_FILE_RAW) == 0 ) {
                fprintf(stderr, "Encoder output file [%s] should be RGB image (*.rgb)!\n", output);
                return -1;
            }
            
            // Decode image
            GPUJPEG_TIMER_INIT();
            GPUJPEG_TIMER_START();
            
            printf("\nDecoding Image [%s]\n", input);
        
            // Load image
            int image_size = 0;
            uint8_t* image = NULL;
            if ( gpujpeg_image_load_from_file(input, &image, &image_size) != 0 ) {
                fprintf(stderr, "Failed to load image [%s]!\n", argv[index]);
                return -1;
            }
            
            GPUJPEG_TIMER_STOP_PRINT("Load Image:         ");
            GPUJPEG_TIMER_START();
                
            // Encode image
            uint8_t* image_decompressed = NULL;
            int image_decompressed_size = 0;
            if ( gpujpeg_decoder_decode(decoder, image, image_size, &image_decompressed, &image_decompressed_size) != 0 ) {
                fprintf(stderr, "Failed to decode image [%s]!\n", argv[index]);
                return -1;
            }
            
            GPUJPEG_TIMER_STOP_PRINT("Decode Image:       ");
            GPUJPEG_TIMER_START();
            
            // Save image
            if ( gpujpeg_image_save_to_file(output, image_decompressed, image_decompressed_size) != 0 ) {
                fprintf(stderr, "Failed to save image [%s]!\n", argv[index]);
                return -1;
            }
            
            GPUJPEG_TIMER_STOP_PRINT("Save Image:         ");
            
            printf("Decompressed Size:   %d bytes [%s]\n", image_decompressed_size, output);
            
            // Destroy image
            gpujpeg_image_destroy(image);
        }
        
        // Destroy decoder
        gpujpeg_decoder_destroy(decoder);
    }
    
	return 0;
}
