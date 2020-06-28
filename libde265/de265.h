/*
 * H.265 video codec.
 * Copyright (c) 2013-2014 struktur AG, Dirk Farin <farin@struktur.de>
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


#include <libde265/de265-version.h>

//#define inline static __inline


#ifndef __STDC_LIMIT_MACROS
#define __STDC_LIMIT_MACROS 1
#endif
#include <stdint.h>

#if defined(_MSC_VER) && !defined(LIBDE265_STATIC_BUILD)
  #ifdef LIBDE265_EXPORTS
  #define LIBDE265_API __declspec(dllexport)
  #else
  #define LIBDE265_API __declspec(dllimport)
  #endif
#elif HAVE_VISIBILITY
  #ifdef LIBDE265_EXPORTS
  #define LIBDE265_API __attribute__((__visibility__("default")))
  #else
  #define LIBDE265_API
  #endif
#else
  #define LIBDE265_API
#endif

#if __GNUC__
#define LIBDE265_DEPRECATED __attribute__((deprecated))
#elif defined(_MSC_VER)
#define LIBDE265_DEPRECATED __declspec(deprecated)
#else
#define LIBDE265_DEPRECATED
#endif

#if defined(_MSC_VER)
#define LIBDE265_INLINE __inline
#else
#define LIBDE265_INLINE inline
#endif

#include "de265-error.h"
#include "vps.h"
#include "pps.h"
#include "sps.h"

/* === version numbers === */

// version of linked libde265 library
LIBDE265_API const char *de265_get_version(void);
LIBDE265_API uint32_t de265_get_version_number(void);

LIBDE265_API int de265_get_version_number_major(void);
LIBDE265_API int de265_get_version_number_minor(void);
LIBDE265_API int de265_get_version_number_maintenance(void);


/* === error code management === */
LIBDE265_API const char* de265_get_error_text(de265_error err);

/* Returns true, if 'err' is DE265_OK or a warning.
 */
LIBDE265_API int  de265_isOK(de265_error err);

LIBDE265_API void de265_disable_logging(); // DEPRECATED
LIBDE265_API void de265_set_verbosity(int level);


/* === image === */

/* The image is currently always 3-channel YCbCr, with 4:2:0 chroma.
   But you may want to check the chroma format anyway for future compatibility.
 */

struct de265_image;

enum de265_chroma {
  de265_chroma_mono=0,
  de265_chroma_420=1,
  de265_chroma_422=2,
  de265_chroma_444=3
};

typedef int64_t de265_PTS;


LIBDE265_API int de265_get_image_width(const struct de265_image*,int channel);
LIBDE265_API int de265_get_image_height(const struct de265_image*,int channel);
LIBDE265_API enum de265_chroma de265_get_chroma_format(const struct de265_image*);
LIBDE265_API int de265_get_bits_per_pixel(const struct de265_image*,int channel);
/* The |out_stride| is returned as "bytes per line" if a non-NULL parameter is given. */
LIBDE265_API const uint8_t* de265_get_image_plane(const struct de265_image*, int channel, int* out_stride);
LIBDE265_API void* de265_get_image_plane_user_data(const struct de265_image*, int channel);
LIBDE265_API de265_PTS de265_get_image_PTS(const struct de265_image*);
LIBDE265_API void* de265_get_image_user_data(const struct de265_image*);
LIBDE265_API void de265_set_image_user_data(struct de265_image*, void *user_data);

/* Get NAL-header information of this frame. You can pass in NULL pointers if you
   do not need this piece of information.
 */
LIBDE265_API void de265_get_image_NAL_header(const struct de265_image*,
                                             int* nal_unit_type,
                                             const char** nal_unit_name, // textual description of 'nal_unit_type'
                                             int* nuh_layer_id,
                                             int* nuh_temporal_id);


/* === decoder === */

typedef void de265_decoder_context; // private structure



/* Get a new decoder context. Must be freed with de265_free_decoder(). */
LIBDE265_API de265_decoder_context* de265_new_decoder(void);

/* Initialize background decoding threads. If this function is not called,
   all decoding is done in the main thread (no multi-threading). */
LIBDE265_API de265_error de265_start_worker_threads(de265_decoder_context*, int number_of_threads);

/* Free decoder context. May only be called once on a context. */
LIBDE265_API de265_error de265_free_decoder(de265_decoder_context*);

#ifndef LIBDE265_DISABLE_DEPRECATED
/* Push more data into the decoder, must be raw h265.
   All complete images in the data will be decoded, hence, do not push
   too much data at once to prevent image buffer overflows.
   The end of a picture can only be detected when the succeeding start-code
   is read from the data.
   If you want to flush the data and force decoding of the data so far
   (e.g. at the end of a file), call de265_decode_data() with 'length' zero.

   NOTE: This method is deprecated and will be removed in a future version.
   You should use "de265_push_data" or "de265_push_NAL" and "de265_decode"
   instead.
*/
LIBDE265_API LIBDE265_DEPRECATED de265_error de265_decode_data(de265_decoder_context*, const void* data, int length);
#endif

/* Push more data into the decoder, must be a raw h265 bytestream with startcodes.
   The PTS is assigned to all NALs whose start-code 0x000001 is contained in the data.
   The bytestream must contain all stuffing-bytes.
   This function only pushes data into the decoder, nothing will be decoded.
*/
LIBDE265_API de265_error de265_push_data(de265_decoder_context*, const void* data, int length,
                                         de265_PTS pts, void* user_data);

/* Indicate that de265_push_data has just received data until the end of a NAL.
   The remaining pending input data is put into a NAL package and forwarded to the decoder.
*/
LIBDE265_API void        de265_push_end_of_NAL(de265_decoder_context*);

/* Indicate that de265_push_data has just received data until the end of a frame.
   All data pending at the decoder input will be pushed into the decoder and
   the decoded picture is pushed to the output queue.
*/
LIBDE265_API void        de265_push_end_of_frame(de265_decoder_context*);

/* Push a complete NAL unit without startcode into the decoder. The data must still
   contain all stuffing-bytes.
   This function only pushes data into the decoder, nothing will be decoded.
*/
LIBDE265_API de265_error de265_push_NAL(de265_decoder_context*, const void* data, int length,
                                        de265_PTS pts, void* user_data);

/* Indicate the end-of-stream. All data pending at the decoder input will be
   pushed into the decoder and the decoded picture queue will be completely emptied.
 */
LIBDE265_API de265_error de265_flush_data(de265_decoder_context*);

/* Return number of bytes pending at the decoder input.
   Can be used to avoid overflowing the decoder with too much data.
 */
LIBDE265_API int de265_get_number_of_input_bytes_pending(de265_decoder_context*);

/* Return number of NAL units pending at the decoder input.
   Can be used to avoid overflowing the decoder with too much data.
 */
LIBDE265_API int de265_get_number_of_NAL_units_pending(de265_decoder_context*);

/* Do some decoding. Returns status whether it did perform some decoding or
   why it could not do so. If 'more' is non-null, indicates whether de265_decode()
   should be called again (possibly after resolving the indicated problem).
   DE265_OK - decoding ok
   DE265_ERROR_IMAGE_BUFFER_FULL - DPB full, extract some images before continuing
   DE265_ERROR_WAITING_FOR_INPUT_DATA - insert more data before continuing

   You have to consider these cases:
   - decoding successful   -> err  = DE265_OK, more=true
   - decoding stalled      -> err != DE265_OK, more=true
   - decoding finished     -> err  = DE265_OK, more=false
   - unresolvable error    -> err != DE265_OK, more=false
 */
LIBDE265_API de265_error de265_decode(de265_decoder_context*, int* more);

/* Clear decoder state. Call this when skipping in the stream.
 */
LIBDE265_API void de265_reset(de265_decoder_context*);

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


LIBDE265_API de265_error de265_get_warning(de265_decoder_context*);


enum de265_image_format {
  de265_image_format_mono8    = 1,
  de265_image_format_YUV420P8 = 2,
  de265_image_format_YUV422P8 = 3,
  de265_image_format_YUV444P8 = 4
};

struct de265_image_spec
{
  enum de265_image_format format;
  int width;
  int height;
  int alignment;

  // conformance window

  int crop_left;
  int crop_right;
  int crop_top;
  int crop_bottom;

  int visible_width;  // convenience, width  - crop_left - crop_right
  int visible_height; // convenience, height - crop_top - crop_bottom
};

struct de265_image_allocation
{
  int  (*get_buffer)(de265_decoder_context* ctx, // first parameter deprecated
                     struct de265_image_spec* spec,
                     struct de265_image* img,
                     void* userdata);
  void (*release_buffer)(de265_decoder_context* ctx, // first parameter deprecated
                         struct de265_image* img,
                         void* userdata);
};

/* The user data pointer will be given to the get_buffer() and release_buffer() functions
   in de265_image_allocation. */
LIBDE265_API void de265_set_image_allocation_functions(de265_decoder_context*,
                                                       struct de265_image_allocation*,
                                                       void* userdata);
LIBDE265_API const struct de265_image_allocation *de265_get_default_image_allocation_functions(void);

LIBDE265_API void de265_set_image_plane(struct de265_image* img, int cIdx, void* mem, int stride, void *userdata);


/* --- frame dropping API ---

   To limit decoding to a maximum temporal layer (TID), use de265_set_limit_TID().
   The maximum layer ID in the stream can be queried with de265_get_highest_TID().
   Note that the maximum layer ID can change throughout the stream.

   For a fine-grained selection of the frame-rate, use de265_set_framerate_ratio().
   A percentage of 100% will decode all frames in all temporal layers. A lower percentage
   will drop approximately as many frames. Note that this only accurate if the frames
   are distributed evenly among the layers. Otherwise, the mapping is non-linear.

   The limit_TID has a higher precedence than framerate_ratio. Hence, setting a higher
   framerate-ratio will decode at limit_TID without dropping.

   With change_framerate(), the output frame-rate can be increased/decreased to some
   discrete preferable values. Currently, these are non-dropped decoding at various
   TID layers.
*/

LIBDE265_API int  de265_get_highest_TID(de265_decoder_context*); // highest temporal substream to decode
LIBDE265_API int  de265_get_current_TID(de265_decoder_context*); // currently decoded temporal substream

LIBDE265_API void de265_set_limit_TID(de265_decoder_context*,int max_tid); // highest temporal substream to decode
LIBDE265_API void de265_set_framerate_ratio(de265_decoder_context*,int percent); // percentage of frames to decode (approx)
LIBDE265_API int  de265_change_framerate(de265_decoder_context*,int more_vs_less); // 1: more, -1: less, returns corresponding framerate_ratio


/* --- decoding parameters --- */

enum de265_param {
  DE265_DECODER_PARAM_BOOL_SEI_CHECK_HASH=0, // (bool) Perform SEI hash check on decoded pictures.
  DE265_DECODER_PARAM_DUMP_SPS_HEADERS=1,    // (int)  Dump headers to specified file-descriptor.
  DE265_DECODER_PARAM_DUMP_VPS_HEADERS=2,
  DE265_DECODER_PARAM_DUMP_PPS_HEADERS=3,
  DE265_DECODER_PARAM_DUMP_SLICE_HEADERS=4,
  DE265_DECODER_PARAM_ACCELERATION_CODE=5,   // (int)  enum de265_acceleration, default: AUTO
  DE265_DECODER_PARAM_SUPPRESS_FAULTY_PICTURES=6, // (bool)  do not output frames with decoding errors, default: no (output all images)

  DE265_DECODER_PARAM_DISABLE_DEBLOCKING=7,   // (bool)  disable deblocking
  DE265_DECODER_PARAM_DISABLE_SAO=8           // (bool)  disable SAO filter
  //DE265_DECODER_PARAM_DISABLE_MC_RESIDUAL_IDCT=9,     // (bool)  disable decoding of IDCT residuals in MC blocks
  //DE265_DECODER_PARAM_DISABLE_INTRA_RESIDUAL_IDCT=10  // (bool)  disable decoding of IDCT residuals in MC blocks
};

/* --- callback --- */
struct de265_callback_block
{
  void  (*get_vps)(video_parameter_set* vps);
  void  (*get_sps)(seq_parameter_set* sps);
  void  (*get_pps)(pic_parameter_set* pps);
  void  (*get_image)(de265_image* img);
};
LIBDE265_API void de265_callback_register(de265_decoder_context*, de265_callback_block*);
LIBDE265_API void de265_callback_unregister(de265_decoder_context*);

/* The user data pointer will be given to the get_buffer() and release_buffer() functions
   in de265_image_allocation. */

// sorted such that a large ID includes all optimizations from lower IDs
enum de265_acceleration {
  de265_acceleration_SCALAR = 0, // only fallback implementation
  de265_acceleration_MMX  = 10,
  de265_acceleration_SSE  = 20,
  de265_acceleration_SSE2 = 30,
  de265_acceleration_SSE4 = 40,
  de265_acceleration_AVX  = 50,    // not implemented yet
  de265_acceleration_AVX2 = 60,    // not implemented yet
  de265_acceleration_ARM  = 70,
  de265_acceleration_NEON = 80,
  de265_acceleration_AUTO = 10000
};


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



#endif
