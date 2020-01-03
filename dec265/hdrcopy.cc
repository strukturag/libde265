/*
  libde265 example application.

  MIT License

  Copyright (c) 2013-2014 struktur AG, Dirk Farin <farin@struktur.de>

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


#include "libde265/nal-parser.h"
#include "libde265/decctx.h"
#include <assert.h>

error_queue errqueue;

video_parameter_set vps;
seq_parameter_set   sps;
pic_parameter_set   pps;

CABAC_encoder_bitstream writer;


void process_nal(NAL_unit* nal)
{
  de265_error err = DE265_OK;

  bitreader reader;
  bitreader_init(&reader, nal->data(), nal->size());

  nal_header nal_hdr;
  nal_hdr.read(&reader);
  writer.write_startcode();
  nal_hdr.write(writer);

  printf("NAL: 0x%x 0x%x -  unit type:%s temporal id:%d\n",
         nal->data()[0], nal->data()[1],
         get_NAL_name(nal_hdr.nal_unit_type),
         nal_hdr.nuh_temporal_id);


  if (nal_hdr.nal_unit_type<32) {
    //err = read_slice_NAL(reader, nal, nal_hdr);
  }
  else switch (nal_hdr.nal_unit_type) {
    case NAL_UNIT_VPS_NUT:
      vps.read(&errqueue, &reader);
      vps.dump(1);
      vps.write(&errqueue, writer);
      writer.flush_VLC();
      break;

    case NAL_UNIT_SPS_NUT:
      sps.read(&errqueue, &reader);
      sps.dump(1);
      sps.write(&errqueue, writer);
      writer.flush_VLC();
      break;

    case NAL_UNIT_PPS_NUT:
      //err = read_pps_NAL(reader);
      break;

    case NAL_UNIT_PREFIX_SEI_NUT:
    case NAL_UNIT_SUFFIX_SEI_NUT:
      //err = read_sei_NAL(reader, nal_hdr.nal_unit_type==NAL_UNIT_SUFFIX_SEI_NUT);
      break;

    case NAL_UNIT_EOS_NUT:
      //ctx->FirstAfterEndOfSequenceNAL = true;
      break;
    }
}


int main(int argc, char** argv)
{
  NAL_Parser nal_parser;

  FILE* fh = fopen(argv[1],"rb");
  unsigned char buf[1024];

  writer.write_bits(0,8); // because HM has an extra byte at the beginning

  while(!feof(fh))
    {
      int n = fread(buf,1,1024,fh);
      if (n>0) {
        nal_parser.push_data(buf,n, 0);
      }

      if (nal_parser.get_NAL_queue_length()>0) {
        NAL_unit* nal = nal_parser.pop_from_NAL_queue();
        assert(nal);
        process_nal(nal);
        nal_parser.free_NAL_unit(nal);
      }
    }

  fclose(fh);

  fh = fopen("out.bin","wb");
  fwrite(writer.data(), 1,writer.size(), fh);
  fclose(fh);

  return 0;
}
