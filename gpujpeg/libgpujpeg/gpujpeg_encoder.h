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

#ifndef GPUJPEG_ENCODER_H
#define GPUJPEG_ENCODER_H

#include "gpujpeg_common.h"
#include "gpujpeg_table.h"
#include "gpujpeg_writer.h"

/** JPEG segment */
struct gpujpeg_encoder_segment {
    // Data compressed index
    int data_compressed_index;
    // Data compressed size
    int data_compressed_size;
};

/**
 * JPEG encoder parameters
 */
struct gpujpeg_encoder_parameters
{
    // Quality level (0-100)
    int quality;
    
    // Restart interval
    int restart_interval;
};

/**
 * JPEG encoder structure
 */
struct gpujpeg_encoder
{  
    // Parameters (quality, restart_interval, etc.)
    struct gpujpeg_encoder_parameters param;
    
    // Parameters for image data (width, height, comp_count, etc.)
    struct gpujpeg_image_parameters param_image;
    
    // Source image data coefficient count
    int data_source_size;
    
    // Source image data in device memory (loaded from file)
    uint8_t* d_data_source;
    
    // Allocated data width
    int data_width;
    // Allocated data height
    int data_height;
    // Allocated data coefficient count
    int data_size;
    
    // Preprocessed data in device memory (output from preprocessor)
    uint8_t* d_data;
    
    // Data after DCT and quantization (output from DCT and quantization)
    int16_t* data_quantized;
    // Data after DCT and quantization in device memory (output from DCT and quantization)
    int16_t* d_data_quantized;
    
    // Data after huffman coder (output from huffman coder)
    uint8_t* data_compressed;
    // Data after huffman coder (output from huffman coder)
    uint8_t* d_data_compressed;
    
    // Segments for all components
    struct gpujpeg_encoder_segment* segments;
    // Segments in device memory for all components
    struct gpujpeg_encoder_segment* d_segments;
    // Segment count per component
    int segment_count_per_comp;
    // Segment total count for all components
    int segment_count;
    
    // Quantization tables
    struct gpujpeg_table_quantization table_quantization[GPUJPEG_COMPONENT_TYPE_COUNT];
    
    // Huffman coder tables
    struct gpujpeg_table_huffman_encoder table_huffman[GPUJPEG_COMPONENT_TYPE_COUNT][GPUJPEG_HUFFMAN_TYPE_COUNT];
    // Huffman coder tables in device memory
    struct gpujpeg_table_huffman_encoder* d_table_huffman[GPUJPEG_COMPONENT_TYPE_COUNT][GPUJPEG_HUFFMAN_TYPE_COUNT];
    
    // JPEG writer structure
    struct gpujpeg_writer* writer;
};

/**
 * Set default parameters for JPEG encoder
 * 
 * @param param  Parameters for encoder
 * @return void
 */
void
gpujpeg_encoder_set_default_parameters(struct gpujpeg_encoder_parameters* param);

/**
 * Create JPEG encoder
 * 
 * @param param_image  Parameters for image data
 * @param param  Parameters for encoder
 * @return encoder structure if succeeds, otherwise NULL
 */
struct gpujpeg_encoder*
gpujpeg_encoder_create(struct gpujpeg_image_parameters* param_image, struct gpujpeg_encoder_parameters* param);

/**
 * Compress image by encoder
 * 
 * @param encoder  Encoder structure
 * @param image  Source image data
 * @param image_compressed  Pointer to variable where compressed image data buffer will be placed
 * @param image_compressed_size  Pointer to variable where compressed image size will be placed
 * @return 0 if succeeds, otherwise nonzero
 */
int
gpujpeg_encoder_encode(struct gpujpeg_encoder* encoder, uint8_t* image, uint8_t** image_compressed, int* image_compressed_size);

/**
 * Destory JPEG encoder
 * 
 * @param encoder  Encoder structure
 * @return 0 if succeeds, otherwise nonzero
 */
int
gpujpeg_encoder_destroy(struct gpujpeg_encoder* encoder);

#endif // GPUJPEG_ENCODER_H
