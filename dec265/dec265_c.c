/*
 * H.265 video codec.
 * Copyright (c) 2013 StrukturAG, Dirk Farin, <farin@struktur.de>
 *
 * This file is part of libde265.
 *
 * libde265 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * libde265 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with libde265.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "de265.h"
#include <stdio.h>
#include <stdlib.h>


void write_image(const struct de265_image* img, FILE *fho)
{
  int width  = de265_get_image_width(img,0);
  int height = de265_get_image_height(img,0);

  for (int ch=0;ch<3;ch++) {
    const uint8_t* data;
    int stride;

    data   = de265_get_image_plane(img,ch,&stride);
    width  = de265_get_image_width(img,ch);
    height = de265_get_image_height(img,ch);

    for (int y=0;y<height;y++) {
        fwrite(data + y*stride, 1, width, fho);
    }
  }
  fflush(fho);
}



#define BUFFER_SIZE 4096


int main(int argc, char** argv)
{
  if (argc != 3) {
    fprintf(stderr,"usage: dec265 videofile.bin videofile.yuv\n");
    fprintf(stderr,"The video file must be a raw h.265 bitstream (e.g. HM-10.0 output)\n");
    exit(5);
  }


  de265_init();
  de265_decoder_context* ctx = de265_new_decoder();

  FILE* fh = fopen(argv[1], "rb");
  if (fh==NULL) {
    fprintf(stderr,"cannot open file %s!\n", argv[1]);
    exit(10);
  }

  FILE* fho = fopen(argv[2], "wb");
  if (fh==NULL) {
    fprintf(stderr,"cannot open file %s!\n", argv[2]);
    exit(20);
  }

  de265_error err =DE265_OK;
  for (;;)
    {
      // read a chunk of input data
      uint8_t buf[BUFFER_SIZE];
      int n = fread(buf,1,BUFFER_SIZE,fh);

      // decode input data
      if (n) {
	err = de265_decode_data(ctx, buf, n);
	if (err != DE265_OK) {
	  break;
	}
      }

      if (feof(fh)) {
        err = de265_decode_data(ctx, NULL, 0); // indicate end of stream
        break;
      }

      // show queued output images
      for (;;) {
        const de265_image* img = de265_get_next_picture(ctx);
        if (img==NULL) break;

        static bool first=true;

        if (first) {
          first=false;

          int width  = de265_get_image_width(img,0);
          int height = de265_get_image_height(img,0);
          printf("Decode video %dx%d\n", width, height);
        }

        // write picture

        write_image(img, fho);
      }
    }

  fclose(fh);

  de265_free_decoder(ctx);

  if (err != DE265_OK) {
    fprintf(stderr,"decoding error: %s\n", de265_get_error_text(err));
  }

  return err==DE265_OK ? 0 : 10;
}
