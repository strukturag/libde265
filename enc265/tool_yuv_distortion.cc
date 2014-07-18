
#include <stdio.h>
#include <stdlib.h>

#include <libde265/quality.h>


int main(int argc, char** argv)
{
  if (argc != 5) {
    fprintf(stderr,"need two YUV files and image size as input: FILE1 FILE2 WIDTH HEIGHT\n");
    exit(5);
  }


  FILE* fh_ref = fopen(argv[1],"rb");
  FILE* fh_cmp = fopen(argv[2],"rb");

  int width  = atoi(argv[3]);
  int height = atoi(argv[4]);

  uint8_t* yp_ref = (uint8_t*)malloc(width*height);
  uint8_t* yp_cmp = (uint8_t*)malloc(width*height);

  double mse_y=0.0;
  int nFrames=0;

  for (;;)
    {
      fread(yp_ref,1,width*height,fh_ref);
      fread(yp_cmp,1,width*height,fh_cmp);

      if (feof(fh_ref)) break;
      if (feof(fh_cmp)) break;

      fseek(fh_ref,width*height/2,SEEK_CUR);
      fseek(fh_cmp,width*height/2,SEEK_CUR);

      double curr_mse_y = MSE(yp_ref, width,  yp_cmp, width,  width, height);
      mse_y += curr_mse_y;

      printf("%4d %f\n",nFrames,PSNR(curr_mse_y));

      nFrames++;
    }

  printf("total: %f\n",PSNR(mse_y/nFrames));

  fclose(fh_ref);
  fclose(fh_cmp);

  return 0;
}
