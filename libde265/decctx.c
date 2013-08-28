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

#include "decctx.h"
#include "util.h"
#include "pps_func.h"

#include <string.h>
#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>


void init_decoder_context(decoder_context* ctx)
{
  // --- parameters ---

  ctx->param_sei_check_hash = true;

  // --- internal data ---

  rbsp_buffer_init(&ctx->nal_data);
  ctx->input_push_state = 0;

  memset(ctx, 0, sizeof(decoder_context));

  ctx->ref_pic_sets = NULL;

  de265_init_image(&ctx->img);
  de265_init_image(&ctx->coeff);
  //init_image(&ctx->intra_pred_available);

  for (int i=0;i<DE265_IMAGE_OUTPUT_QUEUE_LEN;i++) {
    ctx->image_output_queue[i] = -1;
    ctx->image_ref_count[i]    =  0;
  }
}


void reset_decoder_context_for_new_picture(decoder_context* ctx)
{
  memset(ctx->ctb_info, 0,sizeof(CTB_info) * ctx->ctb_info_size);
  memset(ctx->cb_info,  0,sizeof(CB_info)  * ctx->cb_info_size);
  memset(ctx->tu_info,  0,sizeof(TU_info)  * ctx->tu_info_size);
  memset(ctx->deblk_info,  0,sizeof(deblock_info)  * ctx->deblk_info_size);

  de265_fill_image(&ctx->coeff, 0,0,0);

  ctx->next_free_slice_index = 0;
}


void free_info_arrays(decoder_context* ctx)
{
  if (ctx->ctb_info) { free(ctx->ctb_info); ctx->ctb_info=NULL; }
  if (ctx->cb_info)  { free(ctx->cb_info);  ctx->cb_info =NULL; }
  if (ctx->tu_info)  { free(ctx->tu_info);  ctx->tu_info =NULL; }
  if (ctx->deblk_info)  { free(ctx->deblk_info);  ctx->deblk_info =NULL; }
}


void allocate_info_arrays(decoder_context* ctx)
{
  seq_parameter_set* sps = ctx->current_sps;

  int deblk_w = (ctx->current_sps->pic_width_in_luma_samples+3)/4;
  int deblk_h = (ctx->current_sps->pic_height_in_luma_samples+3)/4;

  if (ctx->ctb_info_size != sps->PicSizeInCtbsY ||
      ctx->cb_info_size  != sps->PicSizeInMinCbsY ||
      ctx->tu_info_size  != sps->PicSizeInTbsY ||
      ctx->deblk_info_size != deblk_w*deblk_h)
    {
      free_info_arrays(ctx);

      ctx->ctb_info_size  = sps->PicSizeInCtbsY;
      ctx->cb_info_size   = sps->PicSizeInMinCbsY;
      ctx->tu_info_size   = sps->PicSizeInTbsY;
      ctx->deblk_info_size= deblk_w*deblk_h;
      ctx->deblk_width    = deblk_w;
      ctx->deblk_height   = deblk_h;

      // TODO: CHECK: *1 was *2 previously, but I guess this was only for debugging...
      ctx->ctb_info   = malloc( sizeof(CTB_info)   * ctx->ctb_info_size *1);
      ctx->cb_info    = malloc( sizeof(CB_info)    * ctx->cb_info_size  *1);
      ctx->tu_info    = malloc( sizeof(TU_info)    * ctx->tu_info_size  *1);
      ctx->deblk_info = malloc( sizeof(deblock_info) * ctx->deblk_info_size);
    }
}


void free_decoder_context(decoder_context* ctx)
{
  rbsp_buffer_free(&ctx->nal_data);

  free_ref_pic_sets(&ctx->ref_pic_sets);
  de265_free_image(&ctx->img);
  de265_free_image(&ctx->coeff);
  free_info_arrays(ctx);
  //free_image(&ctx->intra_pred_available);

  free_info_arrays(ctx);

  //video_parameter_set vps[ MAX_VPS_SETS ];
  //seq_parameter_set   sps[ MAX_SPS_SETS ];

  for (int i=0;i<DE265_MAX_PPS_SETS;i++) {
    free_pps(&ctx->pps[i]);
  }
}


void process_nal_hdr(decoder_context* ctx, nal_header* nal)
{
  ctx->nal_unit_type = nal->nal_unit_type;

  ctx->IdrPicFlag = (nal->nal_unit_type == NAL_UNIT_IDR_W_RADL ||
                     nal->nal_unit_type == NAL_UNIT_IDR_N_LP);

  ctx->RapPicFlag = (nal->nal_unit_type >= 16 &&
                     nal->nal_unit_type <= 23);
}


void process_vps(decoder_context* ctx, video_parameter_set* vps)
{
  memcpy(&ctx->vps[ vps->video_parameter_set_id ], vps, sizeof(video_parameter_set));
}


void process_sps(decoder_context* ctx, seq_parameter_set* sps)
{
  memcpy(&ctx->sps[ sps->seq_parameter_set_id ], sps, sizeof(seq_parameter_set));
}


void process_pps(decoder_context* ctx, pic_parameter_set* pps)
{
  memcpy(&ctx->pps[ (int)pps->pic_parameter_set_id ], pps, sizeof(pic_parameter_set));
}


seq_parameter_set* get_sps(decoder_context* ctx, int id)
{
  if (ctx->sps[id].sps_read==false) {
    logerror(LogHeaders, "SPS %d has not been read\n", id);
    assert(false); // TODO
    return NULL;
  }

  return &ctx->sps[id];
}

int get_next_slice_index(decoder_context* ctx)
{
  assert(ctx->next_free_slice_index < DE265_MAX_SLICES);
  return ctx->next_free_slice_index++;
}

void process_slice_segment_header(decoder_context* ctx, slice_segment_header* hdr)
{
  // get PPS and SPS for this slice

  int pps_id = hdr->slice_pic_parameter_set_id;
  if (ctx->pps[pps_id].pps_read==false) {
    logerror(LogHeaders, "PPS %d has not been read\n", pps_id);
    assert(false); // TODO
  }

  ctx->current_pps = &ctx->pps[pps_id];
  ctx->current_sps = &ctx->sps[ (int)ctx->current_pps->seq_parameter_set_id ];

  
  // --- prepare decoding of new picture ---

  if (hdr->first_slice_segment_in_pic_flag) {

    // allocate info arrays

    allocate_info_arrays(ctx);

    seq_parameter_set* sps = ctx->current_sps;


    // --- allocate image buffer for decoding ---

    if (ctx->img.y == NULL) {
      int w = sps->pic_width_in_luma_samples;
      int h = sps->pic_height_in_luma_samples;

      enum de265_chroma chroma;
      switch (sps->chroma_format_idc) {
      case 0: chroma = de265_chroma_mono; break;
      case 1: chroma = de265_chroma_420;  break;
      case 2: chroma = de265_chroma_422;  break;
      case 3: chroma = de265_chroma_444;  break;
      default: chroma = de265_chroma_420; assert(0); break; // should never happen
      }

      de265_alloc_image(&ctx->img,
                        w,h,
                        chroma,
                        0 /* border */); // border large enough for intra prediction

      de265_alloc_image(&ctx->coeff,
                        w*2,h,  // 2 bytes per pixel
                        chroma,
                        0 /* border */); // border large enough for intra prediction

#if 0
      alloc_image(&ctx->intra_pred_available,
                  w,h,
                  chroma_mono,
                  64 /* border */); // border large enough for intra prediction
#endif
    }

    reset_decoder_context_for_new_picture(ctx);
  }


  // TODO: only when starting new image...
  // fill_image(&ctx->intra_pred_available, 0,-1,-1);
}


void debug_dump_cb_info(const decoder_context* ctx)
{
  for (int y=0;y<ctx->current_sps->PicHeightInMinCbsY;y++)
    {
      for (int x=0;x<ctx->current_sps->PicWidthInMinCbsY;x++)
        {
          logtrace(LogPixels,"*%d ", ctx->cb_info[ y*ctx->current_sps->PicWidthInMinCbsY + x].depth);
        }
      logtrace(LogPixels,"*\n");
    }
}


#define PIXEL2CB(x) (x >> ctx->current_sps->Log2MinCbSizeY)
#define CB_IDX(x0,y0) (PIXEL2CB(x0) + PIXEL2CB(y0)*ctx->current_sps->PicWidthInMinCbsY)
#define GET_CB_BLK(x,y) ctx->cb_info[CB_IDX(x,y)]
//#define GET_CB_BLK(x,y) (assert(CB_IDX(x,y) < ctx->cb_info_size) , ctx->cb_info[CB_IDX(x,y)])
#define SET_CB_BLK(x,y,log2BlkWidth,  Field,value)                      \
  int cbX = PIXEL2CB(x);                                                \
  int cbY = PIXEL2CB(y);                                                \
  int width = 1 << (log2BlkWidth - ctx->current_sps->Log2MinCbSizeY);   \
  for (int cby=cbY;cby<cbY+width;cby++)                                 \
    for (int cbx=cbX;cbx<cbX+width;cbx++)                               \
      {                                                                 \
        { assert( cbx + cby*ctx->current_sps->PicWidthInMinCbsY < ctx->cb_info_size ); } \
        ctx->cb_info[ cbx + cby*ctx->current_sps->PicWidthInMinCbsY ].Field = value; \
      }

#define SET_CB_BLK_SAVE(x,y,log2BlkWidth,  Field,value)                 \
  int cbX = PIXEL2CB(x);                                                \
  int cbY = PIXEL2CB(y);                                                \
  int width = 1 << (log2BlkWidth - ctx->current_sps->Log2MinCbSizeY);   \
  for (int cby=cbY;cby<cbY+width;cby++)                                 \
    for (int cbx=cbX;cbx<cbX+width;cbx++)                               \
      if (cbx < ctx->current_sps->PicWidthInMinCbsY &&                  \
          cby < ctx->current_sps->PicHeightInMinCbsY)                   \
      {                                                                 \
        { assert( cbx + cby*ctx->current_sps->PicWidthInMinCbsY < ctx->cb_info_size ); } \
        ctx->cb_info[ cbx + cby*ctx->current_sps->PicWidthInMinCbsY ].Field = value; \
      }

void set_ctDepth(decoder_context* ctx, int x,int y, int log2BlkWidth, int depth)
{
  SET_CB_BLK(x,y,log2BlkWidth, depth, depth);
}

int get_ctDepth(const decoder_context* ctx, int x,int y)
{
  return ctx->cb_info[ CB_IDX(x,y) ].depth;
}


void    set_cu_skip_flag(decoder_context* ctx, int x,int y, uint8_t flag)
{
  int cbX = PIXEL2CB(x);
  int cbY = PIXEL2CB(y);

  ctx->cb_info[ cbX + cbY*ctx->current_sps->PicWidthInMinCbsY ].cu_skip_flag = flag;
}

uint8_t get_cu_skip_flag(const decoder_context* ctx, int x,int y)
{
  int cbX = PIXEL2CB(x);
  int cbY = PIXEL2CB(y);

  return ctx->cb_info[ cbX + cbY*ctx->current_sps->PicWidthInMinCbsY ].cu_skip_flag;
}

void          set_PartMode(decoder_context* ctx, int x,int y, enum PartMode mode)
{
  int cbX = PIXEL2CB(x);
  int cbY = PIXEL2CB(y);

  ctx->cb_info[ cbX + cbY*ctx->current_sps->PicWidthInMinCbsY ].PartMode = mode;
}

enum PartMode get_PartMode(const decoder_context* ctx, int x,int y)
{
  int cbX = PIXEL2CB(x);
  int cbY = PIXEL2CB(y);

  return ctx->cb_info[ cbX + cbY*ctx->current_sps->PicWidthInMinCbsY ].PartMode;
}


void set_pred_mode(decoder_context* ctx, int x,int y, int log2BlkWidth, enum PredMode mode)
{
  SET_CB_BLK(x,y,log2BlkWidth, PredMode, mode);
}

enum PredMode get_pred_mode(const decoder_context* ctx, int x,int y)
{
  return ctx->cb_info[ CB_IDX(x,y) ].PredMode;
}

void set_intra_chroma_pred_mode(decoder_context* ctx, int x,int y, int log2BlkWidth, int mode)
{
  SET_CB_BLK(x,y,log2BlkWidth, intra_chroma_pred_mode, mode);
}

int  get_intra_chroma_pred_mode(const decoder_context* ctx, int x,int y)
{
  return GET_CB_BLK(x,y).intra_chroma_pred_mode;
}


#define PIXEL2TU(x) (x >> ctx->current_sps->Log2MinTrafoSize)
#define TU_IDX(x0,y0) (PIXEL2TU(x0) + PIXEL2TU(y0)*ctx->current_sps->PicWidthInTbsY)
#define GET_TU_BLK(x,y) (assert(TU_IDX(x,y) < ctx->tu_info_size) , ctx->tu_info[TU_IDX(x,y)])
#define SET_TU_BLK(x,y,log2BlkWidth,  Field,value)                      \
  int tuX = PIXEL2TU(x);                                                \
  int tuY = PIXEL2TU(y);                                                \
  int width = 1 << (log2BlkWidth - ctx->current_sps->Log2MinTrafoSize); \
  for (int tuy=tuY;tuy<tuY+width;tuy++)                                 \
    for (int tux=tuX;tux<tuX+width;tux++)                               \
      {                                                                 \
        ctx->tu_info[ tux + tuy*ctx->current_sps->PicWidthInTbsY ].Field = value; \
      }

void set_cbf_cb(decoder_context* ctx, int x0,int y0, int depth)
{
  logtrace(LogSlice,"set_cbf_cb at %d;%d depth %d\n",x0,y0,depth);
  ctx->tu_info[TU_IDX(x0,y0)].cbf_cb |= (1<<depth);
}

void set_cbf_cr(decoder_context* ctx, int x0,int y0, int depth)
{
  logtrace(LogSlice,"set_cbf_cr at %d;%d depth %d\n",x0,y0,depth);
  ctx->tu_info[TU_IDX(x0,y0)].cbf_cr |= (1<<depth);
}

int  get_cbf_cb(const decoder_context* ctx, int x0,int y0, int depth)
{
  return (ctx->tu_info[TU_IDX(x0,y0)].cbf_cb & (1<<depth)) ? 1:0;
}

int  get_cbf_cr(const decoder_context* ctx, int x0,int y0, int depth)
{
  return (ctx->tu_info[TU_IDX(x0,y0)].cbf_cr & (1<<depth)) ? 1:0;
}

void set_IntraPredMode(decoder_context* ctx, int x,int y, int log2BlkWidth, enum IntraPredMode mode)
{
  SET_TU_BLK(x,y,log2BlkWidth, IntraPredMode,mode);
}

enum IntraPredMode get_IntraPredMode(const decoder_context* ctx, int x,int y)
{
  return GET_TU_BLK(x,y).IntraPredMode;
}

void set_IntraPredModeC(decoder_context* ctx, int x,int y, int log2BlkWidth, enum IntraPredMode mode)
{
  SET_TU_BLK(x,y,log2BlkWidth, IntraPredModeC,mode);
}

enum IntraPredMode get_IntraPredModeC(const decoder_context* ctx, int x,int y)
{
  return GET_TU_BLK(x,y).IntraPredModeC;
}

void set_SliceAddrRS(decoder_context* ctx, int ctbX, int ctbY, int SliceAddrRS)
{
  assert(ctbX + ctbY*ctx->current_sps->PicWidthInCtbsY < ctx->ctb_info_size);
  ctx->ctb_info[ctbX + ctbY*ctx->current_sps->PicWidthInCtbsY].SliceAddrRS = SliceAddrRS;
}

int  get_SliceAddrRS(const decoder_context* ctx, int ctbX, int ctbY)
{
  return ctx->ctb_info[ctbX + ctbY*ctx->current_sps->PicWidthInCtbsY].SliceAddrRS;
}


void set_SliceHeaderIndex(decoder_context* ctx, int x, int y, int SliceHeaderIndex)
{
  int ctbX = x >> ctx->current_sps->Log2CtbSizeY;
  int ctbY = y >> ctx->current_sps->Log2CtbSizeY;
  ctx->ctb_info[ctbX + ctbY*ctx->current_sps->PicWidthInCtbsY].SliceHeaderIndex = SliceHeaderIndex;
}

int  get_SliceHeaderIndex(const decoder_context* ctx, int x, int y)
{
  int ctbX = x >> ctx->current_sps->Log2CtbSizeY;
  int ctbY = y >> ctx->current_sps->Log2CtbSizeY;
  return ctx->ctb_info[ctbX + ctbY*ctx->current_sps->PicWidthInCtbsY].SliceHeaderIndex;
}

slice_segment_header* get_SliceHeader(decoder_context* ctx, int x, int y)
{
  return &ctx->slice[ get_SliceHeaderIndex(ctx,x,y) ];
}

slice_segment_header* get_SliceHeaderCtb(decoder_context* ctx, int ctbX, int ctbY)
{
  return &ctx->slice[ ctx->ctb_info[ctbX + ctbY*ctx->current_sps->PicWidthInCtbsY].SliceHeaderIndex ];
}

void set_split_transform_flag(decoder_context* ctx,int x0,int y0,int trafoDepth)
{
  ctx->tu_info[TU_IDX(x0,y0)].split_transform_flag |= (1<<trafoDepth);
}

int  get_split_transform_flag(const decoder_context* ctx,int x0,int y0,int trafoDepth)
{
  int idx = TU_IDX(x0,y0);
  return (ctx->tu_info[idx].split_transform_flag & (1<<trafoDepth)) ? 1:0;
}


void set_transform_skip_flag(decoder_context* ctx,int x0,int y0,int cIdx)
{
  ctx->tu_info[TU_IDX(x0,y0)].transform_skip_flag |= (1<<cIdx);
}

int  get_transform_skip_flag(const decoder_context* ctx,int x0,int y0,int cIdx)
{
  int idx = TU_IDX(x0,y0);
  return (ctx->tu_info[idx].transform_skip_flag & (1<<cIdx)) ? 1:0;
}


void set_QPY(decoder_context* ctx,int x,int y, int QP_Y)
{
  assert(x>=0 && x<ctx->current_sps->pic_width_in_luma_samples);
  assert(y>=0 && y<ctx->current_sps->pic_height_in_luma_samples);

  SET_CB_BLK_SAVE(x,y,ctx->current_pps->Log2MinCuQpDeltaSize, QP_Y, QP_Y);
}

int  get_QPY(const decoder_context* ctx,int x,int y)
{
  return GET_CB_BLK(x,y).QP_Y;
}



void get_image_plane(const decoder_context* ctx, int cIdx, uint8_t** image, int* stride)
{
  switch (cIdx) {
  case 0: *image = ctx->img.y;  *stride = ctx->img.stride; break;
  case 1: *image = ctx->img.cb; *stride = ctx->img.chroma_stride; break;
  case 2: *image = ctx->img.cr; *stride = ctx->img.chroma_stride; break;
  }
}


void get_coeff_plane(const decoder_context* ctx, int cIdx, int16_t** image, int* stride)
{
  switch (cIdx) {
  case 0: *image = (int16_t*)ctx->coeff.y;  *stride = ctx->coeff.stride/2; break;
  case 1: *image = (int16_t*)ctx->coeff.cb; *stride = ctx->coeff.chroma_stride/2; break;
  case 2: *image = (int16_t*)ctx->coeff.cr; *stride = ctx->coeff.chroma_stride/2; break;
  }
}


void set_log2CbSize(decoder_context* ctx, int x0, int y0, int log2CbSize)
{
  ctx->cb_info[ CB_IDX(x0,y0) ].CB_size = log2CbSize;

  // assume that remaining cb_info blocks are initialized to zero
}

int get_log2CbSize(const decoder_context* ctx, int x0, int y0)
{
  return ctx->cb_info[ CB_IDX(x0,y0) ].CB_size;
}

int get_log2CbSize_cbUnits(const decoder_context* ctx, int x0, int y0)
{
  return ctx->cb_info[ x0 + y0 * ctx->current_sps->PicWidthInMinCbsY ].CB_size;
}

void    set_deblk_flags(decoder_context* ctx, int x0,int y0, uint8_t flags)
{
  ctx->deblk_info[x0/4 + y0/4*ctx->deblk_width].deblock_flags |= flags;
}

uint8_t get_deblk_flags(const decoder_context* ctx, int x0,int y0)
{
  return ctx->deblk_info[x0/4 + y0/4*ctx->deblk_width].deblock_flags;
}

void    set_deblk_bS(decoder_context* ctx, int x0,int y0, uint8_t bS)
{
  uint8_t* data = &ctx->deblk_info[x0/4 + y0/4*ctx->deblk_width].deblock_flags;
  *data &= ~DEBLOCK_BS_MASK;
  *data |= bS;
}

uint8_t get_deblk_bS(const decoder_context* ctx, int x0,int y0)
{
  return ctx->deblk_info[x0/4 + y0/4*ctx->deblk_width].deblock_flags & DEBLOCK_BS_MASK;
}

void            set_sao_info(decoder_context* ctx, int ctbX,int ctbY,const sao_info* saoinfo)
{
  assert(ctbX + ctbY*ctx->current_sps->PicWidthInCtbsY < ctx->ctb_info_size);
  memcpy(&ctx->ctb_info[ctbX + ctbY*ctx->current_sps->PicWidthInCtbsY].saoInfo,
         saoinfo,
         sizeof(sao_info));
}

const sao_info* get_sao_info(const decoder_context* ctx, int ctbX,int ctbY)
{
  assert(ctbX + ctbY*ctx->current_sps->PicWidthInCtbsY < ctx->ctb_info_size);
  return &ctx->ctb_info[ctbX + ctbY*ctx->current_sps->PicWidthInCtbsY].saoInfo;
}


bool available_zscan(const decoder_context* ctx,
                     int xCurr,int yCurr, int xN,int yN)
{
  seq_parameter_set* sps = ctx->current_sps;
  pic_parameter_set* pps = ctx->current_pps;


  if (xN<0 || yN<0) return false;
  if (xN>=sps->pic_width_in_luma_samples ||
      yN>=sps->pic_height_in_luma_samples) return false;

  int minBlockAddrN = pps->MinTbAddrZS[ (xN>>sps->Log2MinTrafoSize) +
                                        (yN>>sps->Log2MinTrafoSize) * sps->PicWidthInTbsY ];
  int minBlockAddrCurr = pps->MinTbAddrZS[ (xCurr>>sps->Log2MinTrafoSize) +
                                           (yCurr>>sps->Log2MinTrafoSize) * sps->PicWidthInTbsY ];

  if (minBlockAddrN > minBlockAddrCurr) return false;

  int xCurrCtb = xCurr >> sps->Log2CtbSizeY;
  int yCurrCtb = yCurr >> sps->Log2CtbSizeY;
  int xNCtb = xN >> sps->Log2CtbSizeY;
  int yNCtb = yN >> sps->Log2CtbSizeY;

  if (get_SliceAddrRS(ctx, xCurrCtb,yCurrCtb) !=
      get_SliceAddrRS(ctx, xNCtb,   yNCtb)) {
    return false;
  }

  if (pps->TileId[xCurrCtb + yCurrCtb*sps->PicWidthInCtbsY] !=
      pps->TileId[xNCtb    + yNCtb   *sps->PicWidthInCtbsY]) {
    return false;
  }

  return true;
}


void write_picture(const decoder_context* ctx)
{
  static FILE* fh = NULL;
  if (fh==NULL) { fh = fopen("out.yuv","wb"); }

  const de265_image* img = &ctx->img;
  for (int y=0;y<img->height;y++)
    fwrite(img->y + y*img->stride, img->width, 1, fh);

  for (int y=0;y<img->chroma_height;y++)
    fwrite(img->cb + y*img->chroma_stride, img->chroma_width, 1, fh);

  for (int y=0;y<img->chroma_height;y++)
    fwrite(img->cr + y*img->chroma_stride, img->chroma_width, 1, fh);

  fflush(fh);
  //fclose(fh);
}


void draw_block_boundary(const decoder_context* ctx,
                         uint8_t* img,int stride,
                         int x,int y,int log2BlkSize, uint8_t value)
{
  for (int i=0;i<(1<<log2BlkSize);i++)
    {
      int yi = y + i;
      int xi = x + i;
      
      if (yi < ctx->current_sps->pic_height_in_luma_samples) {
        img[yi*stride + x] = value;
      }
      
      if (xi < ctx->current_sps->pic_width_in_luma_samples) {
        img[y*stride + xi] = value;
      }
    }
}


#include "intrapred.h"

void draw_intra_pred_mode(const decoder_context* ctx,
                          uint8_t* img,int stride,
                          int x0,int y0,int log2BlkSize,
                          enum IntraPredMode mode, uint8_t value)
{
  int w = 1<<log2BlkSize;

  if (mode==0) {
    // Planar -> draw square

    for (int i=-w*1/4;i<=w*1/4;i++)
      {
        img[(y0+w/2+i)*stride + x0+w*1/4] = value;
        img[(y0+w/2+i)*stride + x0+w*3/4] = value;
        img[(y0+w*1/4)*stride + x0+w/2+i] = value;
        img[(y0+w*3/4)*stride + x0+w/2+i] = value;
      }
  }
  else if (mode==1) {
    // DC -> draw circle

    for (int i=-w/4;i<w/4;i++)
      {
        int k = (sqrt((double)(w*w - i*i*16))+2)/4;

        img[(y0+w/2+k)*stride + x0+w/2+i] = value;
        img[(y0+w/2-k)*stride + x0+w/2+i] = value;
        img[(y0+w/2+i)*stride + x0+w/2+k] = value;
        img[(y0+w/2+i)*stride + x0+w/2-k] = value;
      }
  }
  else {
    // angular -> draw line in prediction direction

    int slope = intraPredAngle_table[mode];
    bool horiz = (mode<18);

    if (horiz) {
      for (int i=-w/2;i<w/2;i++)
        {
          int dy = (slope*i+Sign(slope*i)*16)/32;
          int y = y0+w/2-dy;
          if (y>=0 && y<ctx->current_sps->pic_height_in_luma_samples) {
            img[y*stride + x0+i+w/2] = value;
          }
        }
    }
    else {
      for (int i=-w/2;i<w/2;i++)
        {
          int dx = (slope*i+Sign(slope*i)*16)/32;
          int x = x0+w/2-dx;
          if (x>=0 && x<ctx->current_sps->pic_width_in_luma_samples) {
            img[(y0+i+w/2)*stride + x] = value;
          }
        }
    }
  }
}


void drawTBgrid(const decoder_context* ctx, uint8_t* img, int stride,
                int x0,int y0, uint8_t value, int log2CbSize, int trafoDepth)
{
  if (get_split_transform_flag(ctx,x0,y0,trafoDepth)) {
    int x1 = x0 + ((1<<(log2CbSize-trafoDepth))>>1);
    int y1 = y0 + ((1<<(log2CbSize-trafoDepth))>>1);
    drawTBgrid(ctx,img,stride,x0,y0,value,log2CbSize,trafoDepth+1);
    drawTBgrid(ctx,img,stride,x1,y0,value,log2CbSize,trafoDepth+1);
    drawTBgrid(ctx,img,stride,x0,y1,value,log2CbSize,trafoDepth+1);
    drawTBgrid(ctx,img,stride,x1,y1,value,log2CbSize,trafoDepth+1);
  }
  else {
    draw_block_boundary(ctx,img,stride,x0,y0,log2CbSize-trafoDepth, value);
  }
}




enum DrawMode {
  Partitioning_CB,
  Partitioning_TB,
  Partitioning_PB,
  IntraPredMode
};


void draw_tree_grid(const decoder_context* ctx, uint8_t* img, int stride,
                    uint8_t value, enum DrawMode what)
{
  int minCbSize = ctx->current_sps->MinCbSizeY;

  for (int y0=0;y0<ctx->current_sps->PicHeightInMinCbsY;y0++)
    for (int x0=0;x0<ctx->current_sps->PicWidthInMinCbsY;x0++)
      {
        int log2CbSize = get_log2CbSize_cbUnits(ctx,x0,y0);
        if (log2CbSize==0) {
          continue;
        }

        int xb = x0*minCbSize;
        int yb = y0*minCbSize;


        if (what == Partitioning_TB) {
          drawTBgrid(ctx,img,stride,x0*minCbSize,y0*minCbSize, value, log2CbSize, 0);
        }
        else if (what == Partitioning_CB) {
          draw_block_boundary(ctx,img,stride,xb,yb, log2CbSize, value);
        }
        else if (what == Partitioning_PB) {
          enum PartMode partMode = get_PartMode(ctx,xb,yb);

          int HalfCbSize = (1<<(log2CbSize-1));

          switch (partMode) {
          case PART_2Nx2N:
            draw_block_boundary(ctx,img,stride,xb,yb,log2CbSize, value);
            break;
          case PART_NxN:
            draw_block_boundary(ctx,img,stride,xb,           yb,           log2CbSize-1, value);
            draw_block_boundary(ctx,img,stride,xb+HalfCbSize,yb,           log2CbSize-1, value);
            draw_block_boundary(ctx,img,stride,xb           ,yb+HalfCbSize,log2CbSize-1, value);
            draw_block_boundary(ctx,img,stride,xb+HalfCbSize,yb+HalfCbSize,log2CbSize-1, value);
            break;
          default:
            assert(false);
            break;
          }
        }
        else if (what==IntraPredMode) {
          enum PartMode partMode = get_PartMode(ctx,xb,yb);

          int HalfCbSize = (1<<(log2CbSize-1));

          switch (partMode) {
          case PART_2Nx2N:
            draw_intra_pred_mode(ctx,img,stride,xb,yb,log2CbSize,
                                 get_IntraPredMode(ctx,xb,yb), value);
            break;
          case PART_NxN:
            draw_intra_pred_mode(ctx,img,stride,xb,           yb,           log2CbSize-1,
                                 get_IntraPredMode(ctx,xb,yb), value);
            draw_intra_pred_mode(ctx,img,stride,xb+HalfCbSize,yb,           log2CbSize-1,
                                 get_IntraPredMode(ctx,xb+HalfCbSize,yb), value);
            draw_intra_pred_mode(ctx,img,stride,xb           ,yb+HalfCbSize,log2CbSize-1,
                                 get_IntraPredMode(ctx,xb,yb+HalfCbSize), value);
            draw_intra_pred_mode(ctx,img,stride,xb+HalfCbSize,yb+HalfCbSize,log2CbSize-1,
                                 get_IntraPredMode(ctx,xb+HalfCbSize,yb+HalfCbSize), value);
            break;
          default:
            assert(false);
            break;
          }
        }
      }
}


void draw_CB_grid(const decoder_context* ctx, uint8_t* img, int stride, uint8_t value)
{
  draw_tree_grid(ctx,img,stride,value, Partitioning_CB);
}

void draw_TB_grid(const decoder_context* ctx, uint8_t* img, int stride, uint8_t value)
{
  draw_tree_grid(ctx,img,stride,value, Partitioning_TB);
}

void draw_PB_grid(const decoder_context* ctx, uint8_t* img, int stride, uint8_t value)
{
  draw_tree_grid(ctx,img,stride,value, Partitioning_PB);
}

void draw_intra_pred_modes(const decoder_context* ctx, uint8_t* img, int stride, uint8_t value)
{
  draw_tree_grid(ctx,img,stride,value, IntraPredMode);
}

