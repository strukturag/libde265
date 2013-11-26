
#include "fallback-motion.h"
#include "util.h"

#ifdef _MSC_VER
# include <malloc.h>
#else
# include <alloca.h>
#endif

#include <assert.h>


void put_unweighted_pred_8_fallback(uint8_t *dst, ptrdiff_t dststride,
                                    int16_t *src, ptrdiff_t srcstride,
                                    int width, int height)
{
  int offset8bit = 32;
  int shift8bit = 6;

  assert((width&1)==0);

  for (int y=0;y<height;y++) {
    int16_t* in  = &src[y*srcstride];
    uint8_t* out = &dst[y*dststride];

    for (int x=0;x<width;x+=2) {
      out[0] = Clip1_8bit((in[0] + offset8bit)>>shift8bit);
      out[1] = Clip1_8bit((in[1] + offset8bit)>>shift8bit);
      out+=2; in+=2;
    }
  }
}


void put_weighted_pred_avg_8_fallback(uint8_t *dst, ptrdiff_t dststride,
                                      int16_t *src1, int16_t *src2,
                                      ptrdiff_t srcstride, int width,
                                      int height)
{
  int offset8bit = 64;
  int shift8bit = 7;

  assert((width&1)==0);

  // I had a special case for 8-pixel parallel, unrolled code,
  // but I did not see any speedup.

#if 0
  for (int y=0;y<height;y++) {
    int16_t* in1 = &src1[y*srcstride];
    int16_t* in2 = &src2[y*srcstride];
    uint8_t* out = &dst[y*dststride];

    for (int x=0;x<width;x++) {
      out[0] = Clip1_8bit((in1[0] + in2[0] + offset8bit)>>shift8bit);
      out++; in1++; in2++;
    }
  }
#endif

#if 0
  if ((width&7)==0) {
    for (int y=0;y<height;y++) {
      int16_t* in1 = &src1[y*srcstride];
      int16_t* in2 = &src2[y*srcstride];
      uint8_t* out = &dst[y*dststride];

      for (int x=0;x<width;x+=8) {
        out[0] = Clip1_8bit((in1[0] + in2[0] + offset8bit)>>shift8bit);
        out[1] = Clip1_8bit((in1[1] + in2[1] + offset8bit)>>shift8bit);
        out[2] = Clip1_8bit((in1[2] + in2[2] + offset8bit)>>shift8bit);
        out[3] = Clip1_8bit((in1[3] + in2[3] + offset8bit)>>shift8bit);
        out[4] = Clip1_8bit((in1[4] + in2[4] + offset8bit)>>shift8bit);
        out[5] = Clip1_8bit((in1[5] + in2[5] + offset8bit)>>shift8bit);
        out[6] = Clip1_8bit((in1[6] + in2[6] + offset8bit)>>shift8bit);
        out[7] = Clip1_8bit((in1[7] + in2[7] + offset8bit)>>shift8bit);
        out+=8; in1+=8; in2+=8;
      }
    }
  }
  else
#endif
    {
      for (int y=0;y<height;y++) {
        int16_t* in1 = &src1[y*srcstride];
        int16_t* in2 = &src2[y*srcstride];
        uint8_t* out = &dst[y*dststride];

        for (int x=0;x<width;x+=2) {
          out[0] = Clip1_8bit((in1[0] + in2[0] + offset8bit)>>shift8bit);
          out[1] = Clip1_8bit((in1[1] + in2[1] + offset8bit)>>shift8bit);
          out+=2; in1+=2; in2+=2;
        }
      }
    }
}



void put_epel_8_fallback(int16_t *out, ptrdiff_t out_stride,
                         uint8_t *src, ptrdiff_t src_stride,
                         int width, int height,
                         int mx, int my, int16_t* mcbuffer)
{
  int shift3 = 6;

  for (int y=0;y<height;y++) {
    int16_t* o = &out[y*out_stride];
    uint8_t* i = &src[y*src_stride];

    for (int x=0;x<width;x++) {
      *o = *i << shift3;
      o++;
      i++;
    }
  }
}


void put_epel_hv_8_fallback(int16_t *dst, ptrdiff_t dst_stride,
                            uint8_t *src, ptrdiff_t src_stride,
                            int nPbWC, int nPbHC,
                            int xFracC, int yFracC, int16_t* mcbuffer)
{
  const int shift1 = 0;
  const int shift2 = 6;
  const int shift3 = 6;

  int extra_left = 1;
  int extra_top  = 1;
  int extra_right = 2;
  int extra_bottom= 2;

  int nPbW_extra = extra_left + nPbWC + extra_right;
  int nPbH_extra = extra_top  + nPbHC + extra_bottom;

  int16_t* tmp2buf = (int16_t*)alloca( nPbWC      * nPbH_extra * sizeof(int16_t) );

  /*
  printf("x,y FracC: %d/%d\n",xFracC,yFracC);


  printf("---IN---\n");

  for (int y=-extra_top;y<nPbHC+extra_bottom;y++) {
    uint8_t* p = &src[y*src_stride -extra_left];

    for (int x=-extra_left;x<nPbWC+extra_right;x++) {
      printf("%05d ",*p << 6);
      p++;
    }
    printf("\n");
  }
  */


  // H-filters

  logtrace(LogMotion,"---H---\n");
  //printf("---H---(%d)\n",xFracC);

  for (int y=-extra_top;y<nPbHC+extra_bottom;y++) {
    uint8_t* p = &src[y*src_stride - extra_left];

    for (int x=0;x<nPbWC;x++) {
      int16_t v;
      switch (xFracC) {
      case 0: v = p[1]; break;
      case 1: v = (-2*p[0]+58*p[1]+10*p[2]-2*p[3])>>shift1; break;
      case 2: v = (-4*p[0]+54*p[1]+16*p[2]-2*p[3])>>shift1; break;
      case 3: v = (-6*p[0]+46*p[1]+28*p[2]-4*p[3])>>shift1; break;
      case 4: v = (-4*p[0]+36*p[1]+36*p[2]-4*p[3])>>shift1; break;
      case 5: v = (-4*p[0]+28*p[1]+46*p[2]-6*p[3])>>shift1; break;
      case 6: v = (-2*p[0]+16*p[1]+54*p[2]-4*p[3])>>shift1; break;
      case 7: v = (-2*p[0]+10*p[1]+58*p[2]-2*p[3])>>shift1; break;
      }

      //printf("%d %d %d %d -> %d\n",p[0],p[1],p[2],p[3],v);
        
      tmp2buf[y+extra_top + x*nPbH_extra] = v;
      p++;

      //printf("%05d ",tmp2buf[y+extra_top + x*nPbH_extra]);
    }
    //printf("\n");
  }

  // V-filters

  int vshift = (xFracC==0 ? shift1 : shift2);

  for (int x=0;x<nPbWC;x++) {
    int16_t* p = &tmp2buf[x*nPbH_extra];

    for (int y=0;y<nPbHC;y++) {
      int16_t v;
      //logtrace(LogMotion,"%x %x %x  %x  %x %x %x\n",p[0],p[1],p[2],p[3],p[4],p[5],p[6]);

      switch (yFracC) {
      case 0: v = p[1]; break;
      case 1: v = (-2*p[0]+58*p[1]+10*p[2]-2*p[3])>>vshift; break;
      case 2: v = (-4*p[0]+54*p[1]+16*p[2]-2*p[3])>>vshift; break;
      case 3: v = (-6*p[0]+46*p[1]+28*p[2]-4*p[3])>>vshift; break;
      case 4: v = (-4*p[0]+36*p[1]+36*p[2]-4*p[3])>>vshift; break;
      case 5: v = (-4*p[0]+28*p[1]+46*p[2]-6*p[3])>>vshift; break;
      case 6: v = (-2*p[0]+16*p[1]+54*p[2]-4*p[3])>>vshift; break;
      case 7: v = (-2*p[0]+10*p[1]+58*p[2]-2*p[3])>>vshift; break;
      }
        
      dst[x + y*dst_stride] = v;
      p++;
    }

  }

  /*
  printf("---V---\n");
  for (int y=0;y<nPbHC;y++) {
    for (int x=0;x<nPbWC;x++) {
      printf("%05d ",dst[x+y*dst_stride]);
    }
    printf("\n");
  }
  */
}
