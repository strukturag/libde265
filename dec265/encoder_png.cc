/*
  libheif example application "convert".

  MIT License

  Copyright (c) 2017 struktur AG, Joachim Bauch <bauch@struktur.de>

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/
#include <cerrno>
#include <png.h>
#include <cstring>
#include <cstdlib>
#include <vector>
#include <cassert>

#include "encoder_png.h"

inline uint8_t clip8(double v)
{
  if (v<0) return 0;
  if (v>255) return 255;
  return uint8_t(v);
}


bool write_png(const struct de265_image* image,
               const struct de265_image* alpha_image, // or nullptr
               const std::string& filename)
{
  png_structp png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, nullptr,
                                                nullptr, nullptr);
  if (!png_ptr) {
    fprintf(stderr, "libpng initialization failed (1)\n");
    return false;
  }

  png_infop info_ptr = png_create_info_struct(png_ptr);
  if (!info_ptr) {
    png_destroy_write_struct(&png_ptr, nullptr);
    fprintf(stderr, "libpng initialization failed (2)\n");
    return false;
  }

  FILE* fp = fopen(filename.c_str(), "wb");
  if (!fp) {
    fprintf(stderr, "Can't open %s: %s\n", filename.c_str(), strerror(errno));
    png_destroy_write_struct(&png_ptr, &info_ptr);
    return false;
  }

  if (setjmp(png_jmpbuf(png_ptr))) {
    png_destroy_write_struct(&png_ptr, &info_ptr);
    fclose(fp);
    fprintf(stderr, "Error while encoding image\n");
    return false;
  }

  png_init_io(png_ptr, fp);

  bool withAlpha = (alpha_image != nullptr);

  int width = de265_get_image_width(image, 0);
  int height = de265_get_image_height(image, 0);

  int bitDepth;
  int input_bpp = de265_get_bits_per_pixel(image, 0);
  if (input_bpp > 8) {
    bitDepth = 16;
  }
  else {
    bitDepth = 8;
  }

  if (bitDepth != 8) {
    // TODO
    fclose(fp);
    fprintf(stderr, "We currently only support 8 bit PNG output.\n");
    return false;
  }

  if (de265_get_chroma_format(image) != de265_chroma_420) {
    // TODO
    fclose(fp);
    fprintf(stderr, "We currently only support chroma 4:2:0 in PNG output.\n");
    return false;
  }

  assert(de265_get_bits_per_pixel(image, 1) == bitDepth);
  if (withAlpha) {
    assert(de265_get_bits_per_pixel(alpha_image, 0) == bitDepth);
  }

  const int colorType = withAlpha ? PNG_COLOR_TYPE_RGBA : PNG_COLOR_TYPE_RGB;

  png_set_IHDR(png_ptr, info_ptr, width, height, bitDepth, colorType,
               PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_BASE, PNG_FILTER_TYPE_BASE);


  png_write_info(png_ptr, info_ptr);

  int stride_y;
  const uint8_t* row_y = de265_get_image_plane(image, 0, &stride_y);
  int stride_u;
  const uint8_t* row_u = de265_get_image_plane(image, 1, &stride_u);
  int stride_v;
  const uint8_t* row_v = de265_get_image_plane(image, 2, &stride_v);

  int stride_a = 0;
  const uint8_t* row_a = nullptr;
  if (withAlpha) {
    row_a = de265_get_image_plane(alpha_image, 0, &stride_a);
  }

#if 0
  uint8_t** row_pointers = new uint8_t* [height];
  for (int y = 0; y < height; ++y) {
    row_pointers[y] = const_cast<uint8_t*>(&row_rgb[y * stride_rgb]);
  }
#endif

#if 0
  if (bitDepth == 16) {
    // shift image data to full 16bit range

    int shift = 16 - input_bpp;
    if (shift > 0) {
      for (int y = 0; y < height; ++y) {
        for (int x = 0; x < stride_rgb; x += 2) {
          uint8_t* p = (&row_pointers[y][x]);
          int v = (p[0] << 8) | p[1];
          v = (v << shift) | (v >> (16 - shift));
          p[0] = (uint8_t) (v >> 8);
          p[1] = (uint8_t) (v & 0xFF);
        }
      }
    }
  }
#endif

  int bytes_per_pixel = (withAlpha ? 4 : 3);
  auto* row = new uint8_t[bytes_per_pixel * width];

  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      int Y = row_y[y * stride_y + x] - 16;
      int U = row_u[(y / 2) * stride_u + x / 2] - 128;
      int V = row_v[(y / 2) * stride_v + x / 2] - 128;

      row[bytes_per_pixel * x + 0] = clip8(1.164 * Y + 1.596 * V);
      row[bytes_per_pixel * x + 1] = clip8(1.164 * Y - 0.392 * U - 0.813 * V);
      row[bytes_per_pixel * x + 2] = clip8(1.164 * Y + 2.017 * U);

      if (withAlpha) {
        row[bytes_per_pixel * x + 3] = row_a[y * stride_a + x];
      }
    }

    png_write_row(png_ptr, row);
  }

  //png_write_image(png_ptr, row_pointers);

  png_write_end(png_ptr, nullptr);
  png_destroy_write_struct(&png_ptr, &info_ptr);
  //delete[] row_pointers;
  delete[] row;

  fclose(fp);
  return true;
}
