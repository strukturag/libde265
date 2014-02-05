/*
 * H.265 video codec.
 * Copyright (c) 2013 StrukturAG, Dirk Farin, <farin@struktur.de>
 *
 * This file is part of libde265.
 *
 * libde265 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * libde265 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with libde265.  If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef DE265_H
#define DE265_H

#ifdef __cplusplus
extern "C" {
#endif
//#define inline static __inline


#define __STDC_LIMIT_MACROS 1
#include <stdint.h>

#if defined(_MSC_VER) && !defined(LIBDE265_STATIC_BUILD)
  #ifdef LIBDE265_EXPORTS
  #define LIBDE265_API __declspec(dllexport)
  #else
  #define LIBDE265_API __declspec(dllimport)
  #endif
#else
  #define LIBDE265_API
#endif


/* === error codes === */

typedef enum {
  DE265_OK = 0,
  DE265_ERROR_NO_SUCH_FILE,
  DE265_ERROR_NO_STARTCODE,
  DE265_ERROR_EOF,
  DE265_ERROR_COEFFICIENT_OUT_OF_IMAGE_BOUNDS,
  DE265_ERROR_CHECKSUM_MISMATCH,
  DE265_ERROR_CTB_OUTSIDE_IMAGE_AREA,
  DE265_ERROR_OUT_OF_MEMORY,
  DE265_ERROR_CODED_PARAMETER_OUT_OF_RANGE,
  DE265_ERROR_IMAGE_BUFFER_FULL,
  DE265_ERROR_CANNOT_START_THREADPOOL,
  DE265_ERROR_LIBRARY_INITIALIZATION_FAILED,
  DE265_ERROR_LIBRARY_NOT_INITIALIZED,

  // --- errors that should become obsolete in later libde265 versions ---

  DE265_ERROR_MAX_THREAD_CONTEXTS_EXCEEDED = 500,
  DE265_ERROR_MAX_NUMBER_OF_SLICES_EXCEEDED = 501,
  DE265_ERROR_SCALING_LIST_NOT_IMPLEMENTED = 502,

  // --- warnings ---

  DE265_WARNING_NO_WPP_CANNOT_USE_MULTITHREADING = 1000,
  DE265_WARNING_WARNING_BUFFER_FULL,
  DE265_WARNING_PREMATURE_END_OF_SLICE_SEGMENT,
  DE265_WARNING_INCORRECT_ENTRY_POINT_OFFSET,
  DE265_WARNING_CTB_OUTSIDE_IMAGE_AREA,
  DE265_WARNING_SPS_HEADER_INVALID,
  DE265_WARNING_PPS_HEADER_INVALID,
  DE265_WARNING_SLICEHEADER_INVALID,
  DE265_WARNING_INCORRECT_MOTION_VECTOR_SCALING,
  DE265_WARNING_NONEXISTING_PPS_REFERENCED,
  DE265_WARNING_NONEXISTING_SPS_REFERENCED,
  DE265_WARNING_BOTH_PREDFLAGS_ZERO,
  DE265_WARNING_NONEXISTING_REFERENCE_PICTURE_ACCESSED,
  DE265_WARNING_NUMMVP_NOT_EQUAL_TO_NUMMVQ,
  DE265_WARNING_NUMBER_OF_SHORT_TERM_REF_PIC_SETS_OUT_OF_RANGE,
  DE265_WARNING_SHORT_TERM_REF_PIC_SET_OUT_OF_RANGE,
  DE265_WARNING_FAULTY_REFERENCE_PICTURE_LIST,
} de265_error;

LIBDE265_API const char* de265_get_error_text(de265_error err);

LIBDE265_API void de265_disable_logging();


/* === image === */

/* The image is currently always 3-channel YCbCr, with 4:2:0 chroma.
   But you may want to check the chroma format anyway for future compatibility.
 */

struct de265_image;

enum de265_chroma {
  de265_chroma_mono,
  de265_chroma_420,  // currently the only used format
  de265_chroma_422,
  de265_chroma_444
};


LIBDE265_API int de265_get_image_width(const struct de265_image*,int channel);
LIBDE265_API int de265_get_image_height(const struct de265_image*,int channel);
LIBDE265_API enum de265_chroma de265_get_chroma_format(const struct de265_image*);
LIBDE265_API const uint8_t* de265_get_image_plane(const struct de265_image*, int channel, int* out_stride);


/* === decoder === */

typedef void* de265_decoder_context; // private structure


enum de265_param {
  DE265_DECODER_PARAM_BOOL_SEI_CHECK_HASH, /* (bool) Perform SEI hash check on decoded pictures. */
  DE265_DECODER_PARAM_DUMP_SPS_HEADERS,    /* (int)  Dump headers to specified file-descriptor. */
  DE265_DECODER_PARAM_DUMP_VPS_HEADERS,
  DE265_DECODER_PARAM_DUMP_PPS_HEADERS,
  DE265_DECODER_PARAM_DUMP_SLICE_HEADERS
};



/* Get a new decoder context. Must be freed with de265_free_decoder(). */
LIBDE265_API de265_decoder_context* de265_new_decoder(void);

/* Initialize background decoding threads. If this function is not called,
   all decoding is done in the main thread (no multi-threading). */
LIBDE265_API de265_error de265_start_worker_threads(de265_decoder_context*, int number_of_threads);

/* Free decoder context. May only be called once on a context. */
LIBDE265_API de265_error de265_free_decoder(de265_decoder_context*);


/* Push more data into the decoder, must be raw h265.
   All complete images in the data will be decoded, hence, do not push
   too much data at once to prevent image buffer overflows.
   The end of a picture can only be detected when the succeeding start-code
   is read from the data.
   If you want to flush the data and force decoding of the data so far
   (e.g. at the end of a file), call de265_decode_data() with 'length' zero. */
LIBDE265_API de265_error de265_decode_data(de265_decoder_context*, const void* data, int length);

/* Return next decoded picture, if there is any. If no complete picture has been
   decoded yet, NULL is returned. You should call de265_release_next_picture() to
   advance to the next picture. */
LIBDE265_API const struct de265_image* de265_peek_next_picture(de265_decoder_context*); // may return NULL

/* Get next decoded picture and remove this picture from the decoder output queue.
   Returns NULL is there is no decoded picture ready.
   You can use the picture only until you call any other de265_* function. */
LIBDE265_API const struct de265_image* de265_get_next_picture(de265_decoder_context*); // may return NULL

/* Release the current decoded picture for reuse in the decoder. You should not
   use the data anymore after calling this function. */
LIBDE265_API void de265_release_next_picture(de265_decoder_context*);


LIBDE265_API int de265_get_number_of_input_bytes_pending(de265_decoder_context*);

LIBDE265_API de265_error de265_get_warning(de265_decoder_context*);

/* Set decoding parameters. */
LIBDE265_API void de265_set_parameter_bool(de265_decoder_context*, enum de265_param param, int value);

LIBDE265_API void de265_set_parameter_int(de265_decoder_context*, enum de265_param param, int value);

/* Get decoding parameters. */
LIBDE265_API int  de265_get_parameter_bool(de265_decoder_context*, enum de265_param param);



/* --- optional library initialization --- */

/* Static library initialization. Must be paired with de265_free().
   Initialization is optional, since it will be done implicitly in de265_new_decoder().
   Return value is false if initialization failed.
   Only call de265_free() when initialization was successful.
   Multiple calls to 'init' are allowed, but must be matched with an equal number of 'free' calls.
*/
LIBDE265_API de265_error de265_init(void);

/* Free global library data.
   An implicit free call is made in de265_free_decoder().
   Returns false if library was not initialized before, or if 'free' was called
   more often than 'init'.
 */
LIBDE265_API de265_error de265_free(void);


#ifdef __cplusplus
}
#endif

#endif
