/*
 * H.265 video codec.
 * Copyright (c) 2013-2017 struktur AG, Dirk Farin <farin@struktur.de>
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

#include <libde265/de265-version.h>


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


/* === version numbers === */

// Version string of linked libde265 library.
LIBDE265_API const char *de265_get_version(void);
// Numeric version of linked libde265 library, encoded as 0xHHMMLL00 = HH.MM.LL.
LIBDE265_API uint32_t de265_get_version_number(void);

// Numeric part "HH" from above.
LIBDE265_API int de265_get_version_number_major(void);
// Numeric part "MM" from above.
LIBDE265_API int de265_get_version_number_minor(void);
// Numeric part "LL" from above.
LIBDE265_API int de265_get_version_number_maintenance(void);

// Helper macros to check for given versions of libde265 at compile time.
#define LIBDE265_ENCODED_VERSION(h, m, l) ((h) << 24 | (m) << 16 | (l) << 8)
#define LIBDE265_CHECK_VERSION(h, m, l) (LIBDE265_NUMERIC_VERSION >= LIBDE265_ENCODED_VERSION(h, m, l))


/* === error codes === */

typedef enum de265_error {
  // No error, everything went ok.
  DE265_OK = 0,


  // --- Severe decoding errors because of software or system limitations ---

  // Cannot decode the stream, because an essential feature is not implemented.
  DE265_ERROR_MANDATORY_FUNCTIONALITY_NOT_IMPLEMENTED_YET = 1000,

  // Out of memory, cannot continue decoding.
  DE265_ERROR_OUT_OF_MEMORY = 1001,

  // Cannot start the threads for background decoding.
  DE265_ERROR_CANNOT_START_THREADS = 1002,

  // Library has not been initialized with de265_init() yet.
  DE265_ERROR_LIBRARY_NOT_INITIALIZED = 1003,


  // =================================================================================
  // --- Errors in the video stream that lead to decoding errors, but that are not
  // --- critical for decoder operation.
  // =================================================================================

  // A part of the video stream cannot be decoded, because the feature is not yet
  // implemented. However, decoding will continue, since it is an optional feature.
  DE265_WARNING_OPTIONAL_FUNCTIONALITY_NOT_IMPLEMENTED_YET = 1,

  // A PPS was accessed that was not decoded yet.
  DE265_WARNING_NONEXISTING_PPS_REFERENCED = 2,

  // An SPS was accessed that was not decoded yet.
  DE265_WARNING_NONEXISTING_SPS_REFERENCED = 3,

  // A parameter in a header is outside its valid range or a combination of parameters
  // is not allowed.
  DE265_WARNING_INVALID_VUI_PARAMETER = 4,
  DE265_WARNING_INVALID_SPS_PARAMETER = 5,
  DE265_WARNING_INVALID_PPS_PARAMETER = 6,
  DE265_WARNING_INVALID_VPS_PARAMETER = 7,
  DE265_WARNING_INVALID_SLICE_PARAMETER = 8,

  // An invalid parameter was used in defining the ref-pic-set.
  DE265_WARNING_SHORT_TERM_REF_PIC_SET_PARAMETER_OUT_OF_RANGE = 9,

  // The referenced CTB is outside the image area.
  DE265_WARNING_CTB_OUTSIDE_IMAGE_AREA = 10,


  // --- errors in SPS header ---

  // The SPS's 'chroma_format_idc' is not valid.
  DE265_WARNING_INVALID_CHROMA_FORMAT = 11,

  // Header defines short-term ref-pic-sets than allowed.
  DE265_WARNING_NUMBER_OF_SHORT_TERM_REF_PIC_SETS_OUT_OF_RANGE = 12,

  // --- errors in PPS header ---

  // --- errors in SEI header ---

  // SEI header cannot be decoded because no SPS is active.
  DE265_WARNING_CANNOT_DECODE_SEI_BECAUSE_SPS_IS_MISSING = 13,

  // --- errors in Slice header ---

  // The slice's 'slice_segment_address' references a CTB not within the valid image area.
  DE265_WARNING_SLICE_SEGMENT_ADDRESS_INVALID = 14,

  // A dependent slice may not be the first slice in an image (starting in top-left corner).
  DE265_WARNING_DEPENDENT_SLICE_WITH_ADDRESS_ZERO = 15,

  // A dependent slice was found, but no regular slice preceeded it.
  DE265_WARNING_DEPENDENT_SLICE_WITHOUT_INITIAL_SLICE_HEADER = 16,

  // lt_idx_sps[] is invalid (> num_long_term_ref_pics_sps)
  DE265_WARNING_INVALID_LT_REFERENCE_CANDIDATE = 17,

  // The ref-pic-set index is out of range.
  DE265_WARNING_SHORT_TERM_REF_PIC_SET_OUT_OF_RANGE = 18,

  // The slice header has incorrect entry point offsets (pointing to a wrong position).
  DE265_WARNING_INCORRECT_ENTRY_POINT_OFFSET = 19,


  // --- error in compressed image data ---

  // Slice data ended prematurely.
  DE265_WARNING_PREMATURE_END_OF_SLICE = 20,

  // The mandatory end_of_substream_one_bit was not set at the end of the substream.
  DE265_WARNING_END_OF_SUBSTREAM_BIT_NOT_SET = 21,


  // --- errors during decompression ---

  // Decoded image hash does not match the SEI image hash.
  DE265_WARNING_SEI_CHECKSUM_MISMATCH = 22,

  // Out of memory when trying to apply SAO. However, this is not fatal and SAO was skipped.
  DE265_WARNING_CANNOT_APPLY_SAO_OUT_OF_MEMORY = 23,

  // Too many warnings have been queued. Cannot queue any more warnings until the
  // queue is emptied.
  DE265_WARNING_WARNING_BUFFER_FULL = 24,


  // Next free error ID: 25

  // ================================================================

  // --- encoder errors ---

  DE265_ERROR_PARAMETER_PARSING = 2000,

  DE265_WARNING_SPS_HEADER_INVALID = 2001,


  // ================================================================

  // --- Decoding errors that I think should never happen. Maybe we are missing
  // --- some data sanitize some data on corrupted input streams.
  // --- TODO: it should be checked in which cases these problems can occur in corrupted streams.
  // ---       Until it has been confirmed that this can occur and is no internal decoder problem,
  // --- -> -> LIBRARY USERS SHOULD NOT USE THEM.

  DE265_WARNING_INCORRECT_MOTION_VECTOR_SCALING = 5000,
  DE265_WARNING_BOTH_PREDFLAGS_ZERO = 5001,
  DE265_WARNING_NONEXISTING_REFERENCE_PICTURE_ACCESSED = 5002,
  DE265_WARNING_NUMMVP_NOT_EQUAL_TO_NUMMVQ = 5003, // deblocking
  DE265_WARNING_FAULTY_REFERENCE_PICTURE_LIST = 5004,
  DE265_WARNING_MAX_NUM_REF_PICS_EXCEEDED = 5005,
  DE265_WARNING_COLLOCATED_MOTION_VECTOR_OUTSIDE_IMAGE_AREA = 5006,
  DE265_WARNING_SLICEHEADER_INVALID = 5007,

  // An internal error happened during decoding.
  // This can be caused by an invalid input stream (in which case we should replace this
  // warning with a better one in the future, or a decoder bug).
  DE265_WARNING_DECODING_ERROR = 6000

} de265_error;

LIBDE265_API const char* de265_get_error_text(de265_error err);

// Returns true if 'err' is DE265_OK or a warning.
//
// This function has been removed because it is not required.
// Every de265_error value returned is either OK or a fatal error.
// All warnings messages are returned through the warnings queue.
//
//LIBDE265_API int  de265_isOK(de265_error err);

LIBDE265_API void de265_set_verbosity(int level);



typedef void de265_decoder_context; // private structure


/* === image === */

typedef int64_t de265_PTS;
struct de265_image;


// the numeric values map directly to the values of chroma_format_idc in the h.265 bitstream
enum de265_chroma {
  de265_chroma_mono=0,
  de265_chroma_420=1,
  de265_chroma_422=2,
  de265_chroma_444=3
};

enum de265_colorspace {
  de265_colorspace_YCbCr=0,
  de265_colorspace_GBR  =1
};

LIBDE265_API int de265_get_image_width(const struct de265_image*,int channel);
LIBDE265_API int de265_get_image_height(const struct de265_image*,int channel);
LIBDE265_API enum de265_chroma de265_get_chroma_format(const struct de265_image*);
LIBDE265_API int de265_get_bits_per_pixel(const struct de265_image*,int channel);

/* The |out_stride| is returned as "bytes per line".
   When out_stride is NULL, no value will be written. */
LIBDE265_API const uint8_t* de265_get_image_plane(const struct de265_image*, int channel, int* out_stride);
LIBDE265_API void* de265_get_image_user_data(const struct de265_image*);
LIBDE265_API void* de265_get_image_plane_user_data(const struct de265_image*, int channel);
LIBDE265_API de265_PTS de265_get_image_PTS(const struct de265_image*);

/* True if the image contains no decoding errors because of bitstream errors or
   unavailable references. The image may still contain non-standard content when
   e.g. SAO or deblocking has been deliberately turned off.
*/
LIBDE265_API int de265_is_decoded_image_correct(const struct de265_image* img);



// === image allocation API ===

struct de265_image_intern;


struct de265_image_spec
{
  int width;  // width including an invisible border (also see visible_width below)
  int height; // height including an invisible border (also see visible_height below)
  int alignment;

  enum de265_chroma chroma;

  char luma_bits_per_pixel;
  char chroma_bits_per_pixel;


  // conformance window

  int crop_left;
  int crop_right;
  int crop_top;
  int crop_bottom;

  int visible_width;  // convenience, = width  - crop_left - crop_right
  int visible_height; // convenience, = height - crop_top - crop_bottom
};

LIBDE265_API int de265_get_bits_per_pixel_from_spec(const struct de265_image_spec*,int channel);


struct de265_image_allocation
{
  //! @returns 1 on success, 0 on failure (out of memory)
  int  (*get_buffer)(struct de265_image_intern* img,
                     const struct de265_image_spec* spec,
                     void* allocation_userdata);
  void (*release_buffer)(struct de265_image_intern* img,
                         void* allocation_userdata);

  // the user data that is passed to each call of get_buffer() and release_buffer()
  void* allocation_userdata;
};


// --- Functions to be used within get_buffer() / release_buffer() callbacks ---

// Assign self-allocated memory to an image plane. Stride is number of bytes per line.
LIBDE265_API void de265_set_image_plane_intern(struct de265_image_intern* img,
                                               int cIdx,
                                               void* mem, int stride,
                                               void* userdata);
LIBDE265_API void  de265_set_image_user_data_intern(struct de265_image_intern*, void *user_data);

LIBDE265_API void* de265_get_image_user_data_intern(const struct de265_image_intern*);
LIBDE265_API void* de265_get_image_plane_user_data_intern(const struct de265_image_intern*, int channel);



/* Set custom image allocation functions. */
LIBDE265_API void de265_set_image_allocation_functions(de265_decoder_context*,
                                                       struct de265_image_allocation*);
LIBDE265_API const struct de265_image_allocation *de265_get_default_image_allocation_functions(void);


// For normal decoder operation, you will not need to use this function.
// 'alloc_functions' may be NULL, in which case the default allocators are used
LIBDE265_API struct de265_image* de265_alloc_image(int w,int h,enum de265_chroma chroma,
                                                   int bitDepth_luma, int bitDepth_chroma,
                                                   de265_PTS pts,
                                                   const struct de265_image_allocation* alloc_functions);


/* === decoder === */


/* Get a new decoder context. Must be freed with de265_free_decoder(). */
LIBDE265_API de265_decoder_context* de265_new_decoder(void);

/* Free decoder context. May only be called once on a context. */
LIBDE265_API de265_error de265_free_decoder(de265_decoder_context*);

/* Initialize background decoding threads. If this function is not called,
   all decoding is done in the main thread (no multi-threading). */
LIBDE265_API de265_error de265_start_worker_threads(de265_decoder_context*, int number_of_threads);

LIBDE265_API void de265_set_max_frames_to_decode_in_parallel(de265_decoder_context*, int parallel_frames);


/* Push more data into the decoder, must be a raw h265 bytestream with startcodes.
   The PTS is assigned to all NALs whose start-code 0x000001 is contained in the data.
   The bytestream must contain all stuffing-bytes.
   This function only pushes data into the decoder, nothing will be decoded.

   @returns DE265_OK or DE265_ERROR_OUT_OF_MEMORY
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

   @returns DE265_OK or DE265_ERROR_OUT_OF_MEMORY
*/
LIBDE265_API de265_error de265_push_NAL(de265_decoder_context*, const void* data, int length,
                                        de265_PTS pts, void* user_data);

/* Indicate the end-of-stream. All data pending at the decoder input will be
   pushed into the decoder and the decoded picture queue will be completely emptied.
 */
LIBDE265_API de265_error de265_push_end_of_stream(de265_decoder_context*);

/* Return number of bytes pending at the decoder input.
   Can be used to avoid overflowing the decoder with too much data.
 */
LIBDE265_API int de265_get_number_of_input_bytes_pending(de265_decoder_context*);

/* Return number of NAL units pending at the decoder input.
   Can be used to avoid overflowing the decoder with too much data.
 */
LIBDE265_API int de265_get_number_of_NAL_units_pending(de265_decoder_context*);

/* The number of frames pending at the input of the decoder, waiting to be decoded.
   You can check this to avoid pushing too much data into the decoder and thus growing
   the input buffer without limits.
 */
LIBDE265_API int de265_get_number_of_frames_pending_at_input(de265_decoder_context*);

/* When switching input streams without resetting the decoder, the changes in the POCs
   may result in old pictures staying in the reorder buffer for too long. This then
   results in a wrong picture output sequence. To prevent this, you can here set
   a maximum picture delay in the reorder buffer.
   Deactivate the max_latency by setting it to zero (default).

   Note: if possible, the decoder should be reset when switching streams.
   The maximum latency safety threshold is only for cases in which this is not possible.

   If a maximum delay is specified in the video stream, this is used instead
   (TODO: not yet implemented to take value from stream).
 */
LIBDE265_API void de265_set_max_reorder_buffer_latency(de265_decoder_context*, int n);



/* Query decoder whether:
   - there are decoded images available,
   - more input data is needed,
   - the end of stream is reached.

   These result is returned as a bit field of (de265_action_* values).
   Several flags may be active at once. When 'blocking' is set to 'true',
   this function blocks until one of the actions becomes active
   (image ready, data needed, end of stream).
 */
LIBDE265_API int de265_get_action(de265_decoder_context*, int blocking);
#define de265_action_push_more_input     1
#define de265_action_get_image           2
#define de265_action_end_of_stream       4


/* Clear decoder state. Call this when skipping in the stream.
   This can be called even while the decoder is active. Pictures that are
   currently in the queue are removed.
 */
LIBDE265_API void de265_reset(de265_decoder_context*);

/* Get number of pictures that are ready for display.
 */
LIBDE265_API int de265_get_number_of_pictures_in_output_queue(de265_decoder_context*);

/* Return next decoded picture, if there is any. If no complete picture has been
   decoded yet, NULL is returned. You should call de265_skip_next_picture() to
   advance to the next picture.
   Note that you also have to call de265_release_picture() even if you do not
   advance to the next picture.
*/
LIBDE265_API const struct de265_image* de265_peek_next_picture(de265_decoder_context*);

/* Get next decoded picture and remove this picture from the decoder output queue.
   Returns NULL is there is no decoded picture ready.
   You have to release the picture again with de265_release_picture(), but you may keep
   the picture for as long as you need it. */
LIBDE265_API const struct de265_image* de265_get_next_picture(de265_decoder_context*);

/* Remove the next picture in the output queue.
 */
LIBDE265_API void de265_skip_next_picture(de265_decoder_context*);

/* Release the image received from de265_peek_next_picture() or de265_get_next_picture()
 */
LIBDE265_API void de265_release_picture(const struct de265_image*);

/* Get the next warning from the warning queue.
   When there are no more warnings, DE265_OK is returned.
 */
LIBDE265_API de265_error de265_get_warning(de265_decoder_context*);


/* === frame dropping API ===

   To limit decoding to a maximum temporal layer (TID), use de265_set_limit_TID().
   The maximum layer ID in the stream can be queried with de265_get_highest_TID().
   Note that the maximum layer ID can change throughout the stream.

   For a fine-grained reduction of the frame-rate, use de265_set_framerate_ratio().
   This also works on streams without temporal layers by dropping frames which are
   not required as references. Note that the resulting frame-rate may be higher than
   the requested frame-rate when frames are required as references.
*/

LIBDE265_API int  de265_get_highest_TID(de265_decoder_context*); // highest temporal substream to decode
#define de265_highest_TID_unknown -1

LIBDE265_API void de265_set_highest_TID_to_decode(de265_decoder_context*,int max_tid); // highest temporal substream to decode
#define de265_limit_TID_unlimited -1

LIBDE265_API void de265_set_framerate_ratio(de265_decoder_context*,int percent); // percentage of frames to decode (approx)

// not exported into API yet
// LIBDE265_API int  de265_get_current_TID(de265_decoder_context*); // currently decoded temporal substream

// not exported into API yet...
// LIBDE265_API int  de265_change_framerate(de265_decoder_context*,int more_vs_less); // 1: more, -1: less, returns corresponding framerate_ratio


/* === decoding parameters === */

/* Set CPU features that the decoder may use.
   For x86, the decoder will itself check existing CPU features and does not use non-existing features.
   Hence, you can always enable all of them to get maximum acceleration.
   For ARM, you have to specify the capabilities correctly as they are not checked.

   AArch64 needs not be checked, because it enables instructions that are mandatory for
   every AArch64 processor.

   Options that are not referring to the target architecture are ignored (i.e. you can
   specify them even if you do not need them).
 */
LIBDE265_API void de265_set_CPU_capabilities(de265_decoder_context*, int capabilities);
#define de265_CPU_capability_X86_SSE2    (1<<0)
#define de265_CPU_capability_X86_SSE41   (1<<1)
#define de265_CPU_capability_X86_AVX2    (1<<2)
#define de265_CPU_capability_ARM_NEON    (1<<10)
#define de265_CPU_capability_ARM_AARCH64 (1<<11)
#define de265_CPU_capability_all         (0xFFFF)

/* Return all CPU capabilities that can be auto-detected by the decoder.
   These are also the default CPU capabilities set in a de265_decoder_context object).
 */
LIBDE265_API int de265_get_CPU_capabilites_all_autodetected();


LIBDE265_API void de265_allow_inexact_decoding(de265_decoder_context*, int flags);
#define de265_inexact_decoding_no_SAO                1
#define de265_inexact_decoding_no_deblocking         2
#define de265_inexact_decoding_idct                  4   // not used yet
#define de265_inexact_decoding_only_full_pel_motion  8
#define de265_inexact_decoding_mask_none             0
#define de265_inexact_decoding_mask_all              0xFFFF


LIBDE265_API void de265_suppress_faulty_pictures(de265_decoder_context*, int suppress_enable);


// set callback to NULL to disable
LIBDE265_API void de265_set_dump_headers_callback(de265_decoder_context*, void (*callback)(int nal_unit, const char* text));


  /* TODO
enum de265_SEI_hash_check {
  de265_SEI_hash_check_disabled,
  de265_SEI_hash_check_warn_only,
  de265_SEI_hash_check_suppress_faulty_images
};

LIBDE265_API void de265_set_SEI_hash_check(de265_decoder_context*, enum de265_SEI_hash_check mode);
  */


/* === optional library initialization === */

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
