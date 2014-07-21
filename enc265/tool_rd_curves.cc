
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <assert.h>
#include <stdint.h>
#include <string.h>
#include <getopt.h>

#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <vector>



static struct {
  const char* name;
  const char* value;
} variables[] = {
  { "$HOME"   , "/home/domain/farindk/prog/h265" },
  //  { "$ENC256" , "./enc256/enc256" },
  { "$ENC265" , "$HOME/libde265/enc265/enc265" },
  { "$DEC265" , "$HOME/libde265/dec265/dec265" },
  { "$YUVDIST", "$HOME/libde265/enc265/yuv-distortion" },
  { "$YUV"    , "/storage/users/farindk/yuv" },
  { "$HMENC"  , "HM13enc" },
  { "$HM13CFG", "$HOME/HM/HM-13.0-dev/cfg" },
  { "$X265ENC", "$HOME/x265/build/linux/x265" },
  { "$X264"   , "x264" },
  { "$FFMPEG" , "ffmpeg" },
  { "$F265"   , "$HOME/f265/build/f265cli" },
  { 0,0 }
};


bool keepStreams = false;

std::string replace_variables(std::string str)
{
  bool replaced = false;
  for (int i=0;variables[i].name;i++) {
    size_t pos = str.find(variables[i].name);
    if (pos != std::string::npos) {
      replaced = true;
      str = str.replace(pos, strlen(variables[i].name), variables[i].value);
      break;
    }
  }

  if (!replaced) return str;
  else return replace_variables(str);
}


// ---------------------------------------------------------------------------

struct Preset
{
  const int ID;
  const char* name;
  const char* descr;

  const char* options_de265;
  const char* options_hm;
  const char* options_x265;
  const char* options_f265;
  const char* options_x264;
  const char* options_x264_ffmpeg;
};


Preset preset[] = {
  { 1, "pre01-intra-noLF", "intra, no LF, no SBH, CTB-size 32, min CB=8",
    /* de265  */ "",
    /* HM     */ "-c $HM13CFG/encoder_intra_main.cfg -SBH 0 --SAO=0 --LoopFilterDisable --DeblockingFilterControlPresent --MaxCUSize=32 --MaxPartitionDepth=2",
    /* x265   */ "--no-lft -I 1 --no-signhide",
    /* f265   */ "key-frame-spacing=1",
    /* x264   */ "-I 1",
    /* ffmpeg */ "-g 1"
  },

  { 50, "cb-auto16", "(development test)",
    /* de265  */ "--max-cb-size 16 --min-cb-size 8",
    /* HM     */ "-c $HM13CFG/encoder_intra_main.cfg -SBH 0 --SAO=0 --LoopFilterDisable --DeblockingFilterControlPresent --MaxCUSize=32 --MaxPartitionDepth=2",
    /* x265   */ "--no-lft -I 1 --no-signhide",
    /* f265   */ "key-frame-spacing=1",
    /* x264   */ "-I 1",
    /* ffmpeg */ "-g 1"
  },

  { 99, "best", "default (random-access) encoder parameters",
    /* de265  */ "--max-cb-size 16 --min-cb-size 8",
    /* HM     */ "-c $HM13CFG/encoder_randomaccess_main.cfg",
    /* x265   */ "",
    /* f265   */ "",
    /* x264   */ "",
    /* ffmpeg */ ""
  },

  { 0, NULL }
};

// ---------------------------------------------------------------------------

class Input
{
public:
  Input() {
    width=height=0;
    maxFrames=0;
  }

  void setInput(const char* yuvfilename,int w,int h, float fps) {
    mInputFilename = yuvfilename;
    width = w;
    height = h;
    mFPS = fps;
  }

  void setMaxFrames(int n) { maxFrames=n; }

  std::string options_de265() const {
    std::stringstream sstr;
    sstr << " -i " << mInputFilename << " --width " << width << " --height " << height;
    if (maxFrames) sstr << " --frames " << maxFrames;

    return sstr.str();
  }

  std::string options_HM() const {
    std::stringstream sstr;
    sstr << "-i " << mInputFilename << " -wdt " << width << " -hgt " << height
         << " -fr " << mFPS;
    if (maxFrames) sstr << " -f " << maxFrames;

    return sstr.str();
  }

  std::string options_x265() const {
    std::stringstream sstr;
    sstr << mInputFilename << " --input-res " << width << "x" << height
         << " --fps " << mFPS;
    if (maxFrames) sstr << " -f " << maxFrames;

    return sstr.str();
  }

  std::string options_x264() const {
    std::stringstream sstr;
    sstr << mInputFilename << " --input-res " << width << "x" << height;
    sstr << " --fps 25"; // TODO: check why crf/qp rate-control freaks out when fps is != 25
    if (maxFrames) sstr << " --frames " << maxFrames;

    return sstr.str();
  }

  std::string options_ffmpeg() const {
    std::stringstream sstr;
    sstr << "-f rawvideo -vcodec rawvideo -s " << width << "x" << height; // << " -r " << mFPS
    sstr << " -pix_fmt yuv420p -i " << mInputFilename;
    if (maxFrames) sstr << " -vframes " << maxFrames;

    return sstr.str();
  }

  std::string options_f265() const {
    std::stringstream sstr;
    sstr << mInputFilename << " -w " << width << ":" << height;
    if (maxFrames) sstr << " -c " << maxFrames;

    return sstr.str();
  }

  std::string getFilename() const { return mInputFilename; }
  float getFPS() const { return mFPS; }
  int   getNFrames() const { return maxFrames; }
  int   getWidth() const { return width; }
  int   getHeight() const { return height; }

private:
  std::string mInputFilename;
  int width, height;
  int maxFrames;
  float mFPS;
};

Input input;

struct InputSpec
{
  const char* name;
  const char* filename;
  int width,height, nFrames;
  float fps;
} inputSpec[] = {
  { "paris",     "$YUV/paris_cif.yuv",352,288,1065, 30.0 },
  { "paris10",   "$YUV/paris_cif.yuv",352,288,  10, 30.0 },
  { "paris100",  "$YUV/paris_cif.yuv",352,288, 100, 30.0 },
  { "johnny",    "$YUV/Johnny_1280x720_60.yuv",1280,720,600,60.0 },
  { "johnny10",  "$YUV/Johnny_1280x720_60.yuv",1280,720, 10,60.0 },
  { "johnny100", "$YUV/Johnny_1280x720_60.yuv",1280,720,100,60.0 },
  { "cactus",    "$YUV/Cactus_1920x1080_50.yuv",1920,1080,500,50.0 },
  { NULL }
};


void setInput(const char* input_preset)
{
  bool presetFound=false;

  for (int i=0;inputSpec[i].name;i++) {
    if (strcmp(input_preset, inputSpec[i].name)==0) {
      input.setInput(inputSpec[i].filename,
                     inputSpec[i].width,
                     inputSpec[i].height,
                     inputSpec[i].fps);
      input.setMaxFrames(inputSpec[i].nFrames);
      presetFound=true;
      break;
    }
  }

  if (!presetFound) {
    fprintf(stderr,"no input preset '%s'\n",input_preset);
    exit(5);
  }
}


float bitrate(const char* filename)
{
  struct stat s;
  stat(filename,&s);

  int size = s.st_size;
  int frames = input.getNFrames();
  assert(frames!=0);

  float bitrate = size*8/(frames/input.getFPS());
  return bitrate;
}


// ---------------------------------------------------------------------------

class Quality
{
public:
  virtual ~Quality() { }

  virtual float measure(const char* h265filename) const = 0;
};

class Quality_PSNR : public Quality
{
public:
  virtual float measure(const char* h265filename) const;
  virtual float measure_yuv(const char* yuvfilename) const;
};


float Quality_PSNR::measure(const char* h265filename) const
{
  std::stringstream sstr;
  sstr << "$DEC265 " << h265filename << " -q -t6 -m " << input.getFilename() << " | grep total "
    "| awk '{print $2}' >/tmp/xtmp";

  //std::cout << sstr.str() << "\n";
  int retval = system(replace_variables(sstr.str()).c_str());

  std::ifstream istr;
  istr.open("/tmp/xtmp");
  float quality;
  istr >> quality;

  unlink("/tmp/xtmp");

  return quality;
}


float Quality_PSNR::measure_yuv(const char* yuvfilename) const
{
  std::stringstream sstr;

  sstr << "$YUVDIST " << input.getFilename() << " " << yuvfilename
       << " " << input.getWidth() << " " << input.getHeight()
       << "|grep total| awk '{print $2}' >/tmp/xtmp";

  //std::cout << sstr.str() << "\n";
  int retval = system(replace_variables(sstr.str()).c_str());

  std::ifstream istr;
  istr.open("/tmp/xtmp");
  float quality;
  istr >> quality;

  unlink("/tmp/xtmp");

  return quality;
}


Quality_PSNR quality_psnr;
//Quality* quality = &quality_psnr;

// ---------------------------------------------------------------------------

struct RDPoint
{
  float rate;
  float psnr;
  float time; // computation time in seconds


  RDPoint() { }

  void compute_from_h265(std::string stream_name) {
    rate = bitrate(stream_name.c_str());
    psnr = quality_psnr.measure(stream_name.c_str());
  }

  void compute_from_yuv(std::string stream_name, std::string yuv_name) {
    rate = bitrate(stream_name.c_str());
    psnr = quality_psnr.measure_yuv(yuv_name.c_str());
  }
};


FILE* output_fh;

void write_rd_line(RDPoint p)
{
  fprintf(output_fh,"%7.2f %6.4f %5.0f\n", p.rate/1024, p.psnr, p.time);
  fflush(output_fh);
}


class Encoder
{
public:
  virtual ~Encoder() { }

  virtual std::vector<RDPoint> encode_curve(const Preset& preset) const = 0;

private:
};


class Encoder_de265 : public Encoder
{
public:
  Encoder_de265();
  void setQPRange(int low,int high,int step) { mQPLow=low; mQPHigh=high; mQPStep=step; }

  virtual std::vector<RDPoint> encode_curve(const Preset& preset) const;

private:
  RDPoint encode(const Preset& preset,int qp) const;

  int mQPLow,mQPHigh,mQPStep;
};


Encoder_de265::Encoder_de265()
{
  mQPLow = 10;
  mQPHigh= 40;
  mQPStep=  2;
}


std::vector<RDPoint> Encoder_de265::encode_curve(const Preset& preset) const
{
  std::vector<RDPoint> curve;

  for (int qp=mQPLow ; qp<=mQPHigh ; qp+=mQPStep) {
    curve.push_back(encode(preset, qp));
  }

  return curve;
}


RDPoint Encoder_de265::encode(const Preset& preset,int qp) const
{
  std::stringstream streamname;
  streamname << "de265-" << preset.name << "-" << qp << ".265";

  std::stringstream cmd1;
  cmd1 << "$ENC265 " << input.options_de265()
       << " " << preset.options_de265
       << " -q " << qp << " -o " << streamname.str();

  std::string cmd2 = replace_variables(cmd1.str());

  //std::cout << "CMD: '" << cmd2 << "'\n";
  clock_t t1 = clock();
  int retval = system(cmd2.c_str());
  clock_t t2 = clock();

  RDPoint rd;
  rd.compute_from_h265(streamname.str());
  rd.time = double(t2-t1)/CLOCKS_PER_SEC;

  if (!keepStreams) { unlink(streamname.str().c_str()); }

  write_rd_line(rd);

  return rd;
}




class Encoder_HM : public Encoder
{
public:
  Encoder_HM();
  void setQPRange(int low,int high,int step) { mQPLow=low; mQPHigh=high; mQPStep=step; }

  virtual std::vector<RDPoint> encode_curve(const Preset& preset) const;

private:
  RDPoint encode(const Preset& preset,int qp) const;

  int mQPLow,mQPHigh,mQPStep;
};


Encoder_HM::Encoder_HM()
{
  mQPLow = 10;
  mQPHigh= 40;
  mQPStep=  2;
}


std::vector<RDPoint> Encoder_HM::encode_curve(const Preset& preset) const
{
  std::vector<RDPoint> curve;

  for (int qp=mQPLow ; qp<=mQPHigh ; qp+=mQPStep) {
    curve.push_back(encode(preset, qp));
  }

  return curve;
}


RDPoint Encoder_HM::encode(const Preset& preset,int qp) const
{
  std::stringstream streamname;
  streamname << "hm-" << preset.name << "-" << qp << ".265";

  std::stringstream cmd1;
  cmd1 << "$HMENC " << input.options_HM()
       << " " << preset.options_hm
       << " -q " << qp << " -b " << streamname.str() << " >&2";

  std::string cmd2 = replace_variables(cmd1.str());

  //std::cout << "CMD: '" << cmd2 << "'\n";
  int retval = system(cmd2.c_str());

  RDPoint rd;
  rd.compute_from_h265(streamname.str());
  if (!keepStreams) { unlink(streamname.str().c_str()); }

  write_rd_line(rd);

  return rd;
}



class Encoder_x265 : public Encoder
{
public:
  Encoder_x265();
  void setQPRange(int low,int high,int step) { mQPLow=low; mQPHigh=high; mQPStep=step; }

  virtual std::vector<RDPoint> encode_curve(const Preset& preset) const;

private:
  RDPoint encode(const Preset& preset,int qp) const;

  int mQPLow,mQPHigh,mQPStep;
};


Encoder_x265::Encoder_x265()
{
  /* CRF
  mQPLow =  4;
  mQPHigh= 34;
  mQPStep=  2;
  */

  mQPLow = 10;
  mQPHigh= 40;
  mQPStep=  2;
}


std::vector<RDPoint> Encoder_x265::encode_curve(const Preset& preset) const
{
  std::vector<RDPoint> curve;

  for (int qp=mQPLow ; qp<=mQPHigh ; qp+=mQPStep) {
    curve.push_back(encode(preset, qp));
  }

  return curve;
}


RDPoint Encoder_x265::encode(const Preset& preset,int qp) const
{
  std::stringstream streamname;
  streamname << "x265-" << preset.name << "-" << qp << ".265";

  std::stringstream cmd1;
  cmd1 << "$X265ENC " << input.options_x265()
       << " " << preset.options_x265
       << " --qp " << qp << " " << streamname.str() << " >&2";

  std::string cmd2 = replace_variables(cmd1.str());

  //std::cout << "CMD: '" << cmd2 << "'\n";
  int retval = system(cmd2.c_str());

  RDPoint rd;
  rd.compute_from_h265(streamname.str());

  if (!keepStreams) { unlink(streamname.str().c_str()); }

  write_rd_line(rd);

  return rd;
}




class Encoder_f265 : public Encoder
{
public:
  Encoder_f265();
  void setQPRange(int low,int high,int step) { mQPLow=low; mQPHigh=high; mQPStep=step; }

  virtual std::vector<RDPoint> encode_curve(const Preset& preset) const;

private:
  RDPoint encode(const Preset& preset,int qp) const;

  int mQPLow,mQPHigh,mQPStep;
};


Encoder_f265::Encoder_f265()
{
  mQPLow = 10;
  mQPHigh= 40;
  mQPStep=  2;
}


std::vector<RDPoint> Encoder_f265::encode_curve(const Preset& preset) const
{
  std::vector<RDPoint> curve;

  for (int qp=mQPLow ; qp<=mQPHigh ; qp+=mQPStep) {
    curve.push_back(encode(preset, qp));
  }

  return curve;
}


RDPoint Encoder_f265::encode(const Preset& preset,int qp) const
{
  std::stringstream cmd1;
  cmd1 << "$F265 " << input.options_f265()
       << " f265.out -v -p\"" << preset.options_f265 << " qp=" << qp << "\" >&2";

  std::string cmd2 = replace_variables(cmd1.str());

  std::cout << "CMD: '" << cmd2 << "'\n";
  int retval = system(cmd2.c_str());

  RDPoint rd;
  rd.compute_from_h265("f265.out");

  if (!keepStreams) { unlink("f265.out"); }

  write_rd_line(rd);

  return rd;
}



class Encoder_x264 : public Encoder
{
public:
  Encoder_x264();
  void setCRFRange(int low,int high,int step) { mCRFLow=low; mCRFHigh=high; mCRFStep=step; }

  virtual std::vector<RDPoint> encode_curve(const Preset& preset) const;

private:
  RDPoint encode(const Preset& preset,int crf) const;

  int mCRFLow,mCRFHigh,mCRFStep;
};


Encoder_x264::Encoder_x264()
{
  mCRFLow =  4;
  mCRFHigh= 36;
  mCRFStep=  2;
}


std::vector<RDPoint> Encoder_x264::encode_curve(const Preset& preset) const
{
  std::vector<RDPoint> curve;

  for (int crf=mCRFLow ; crf<=mCRFHigh ; crf+=mCRFStep) {
    curve.push_back(encode(preset, crf));
  }

  return curve;
}


RDPoint Encoder_x264::encode(const Preset& preset,int qp_crf) const
{
  std::stringstream streamname;
  streamname << "x264-" << preset.name << "-" << qp_crf << ".264";

  std::stringstream cmd1;
#if 0
  cmd1 << "$X264 " << input.options_x264()
       << " " << preset.options_x264
       << " --crf " << qp_crf
       << " -o " << streamname.str();
#else
  cmd1 << "$FFMPEG " << input.options_ffmpeg()
       << " " << preset.options_x264_ffmpeg
       << " -crf " << qp_crf
       << " -f h264 " << streamname.str();
#endif

  std::string cmd2 = replace_variables(cmd1.str());

  std::cerr << "CMD: '" << cmd2 << "'\n";
  clock_t t1 = clock();
  int retval = system(cmd2.c_str());
  if (0) execlp("ffmpeg",
         "-f","rawvideo","-vcodec","rawvideo","-s","352x288",
         "-pix_fmt","yuv420p",
         "-i","/storage/users/farindk/yuv/paris_cif.yuv",
         "-vframes","100",
         "-crf","20",
         "-f","h264",
         "out.264",
         NULL);
  clock_t t2 = clock();

  std::string cmd3 = "ffmpeg -i " + streamname.str() + " rdout.yuv";

  retval = system(cmd3.c_str());

  RDPoint rd;
  rd.compute_from_yuv(streamname.str(), "rdout.yuv");
  rd.time = double(t2-t1)/CLOCKS_PER_SEC;

  unlink("rdout.yuv");
  if (!keepStreams) { unlink(streamname.str().c_str()); }

  write_rd_line(rd);

  return rd;
}


Encoder_de265 enc_de265;
Encoder_HM enc_hm;
Encoder_x265 enc_x265;
Encoder_x264 enc_x264;
Encoder_f265 enc_f265;

// ---------------------------------------------------------------------------

static struct option long_options[] = {
  {"keep-streams",      no_argument,       0, 'k' },
  //{"write-bytestream", required_argument,0, 'B' },
  {0,         0,                 0,  0 }
};


void show_usage()
{
  fprintf(stderr,
          "usage: rd-curves 'preset_id' 'input_preset' 'encoder'\n"
          "supported encoders: de265 / hm / x265 / f265 / x264\n");
  fprintf(stderr,
          "presets:\n");

  for (int i=0;preset[i].name!=NULL;i++) {
    fprintf(stderr,
            " %2d %-20s %s\n",preset[i].ID,preset[i].name,preset[i].descr);
  }

  fprintf(stderr,
          "\ninput presets:\n");
  for (int i=0;inputSpec[i].name;i++) {
    fprintf(stderr,
            " %-12s %-30s %4dx%4d, %4d frames, %5.2f fps\n",
            inputSpec[i].name,
            inputSpec[i].filename,
            inputSpec[i].width,
            inputSpec[i].height,
            inputSpec[i].nFrames,
            inputSpec[i].fps);
  }
}

int main(int argc, char** argv)
{
  while (1) {
    int option_index = 0;

    int c = getopt_long(argc, argv, "k", // "qB:",
                        long_options, &option_index);
    if (c == -1)
      break;

    switch (c) {
      case 'k': keepStreams=true; break;
      //case 'B': write_bytestream=true; bytestream_filename=optarg; break;
    }
  }

  if (optind != argc-3) {
    show_usage();
    exit(5);
  }

  int presetID = atoi( argv[optind] );
  const char* inputName = argv[optind+1];
  const char* encoderName = argv[optind+2];

  int presetIdx = -1;

  for (int i=0;preset[i].name != NULL;i++) {
    if (preset[i].ID == presetID) {
      presetIdx = i;
      break;
    }
  }

  if (presetIdx == -1) {
    fprintf(stderr,"preset ID %d does not exist\n",presetID);
    exit(5);
  }

  setInput(inputName);

  Encoder* enc = NULL;
  /**/ if (strcmp(encoderName,"de265")==0) { enc = &enc_de265; }
  else if (strcmp(encoderName,"hm"   )==0) { enc = &enc_hm;   }
  else if (strcmp(encoderName,"x265" )==0) { enc = &enc_x265; }
  else if (strcmp(encoderName,"f265" )==0) { enc = &enc_f265; }
  else if (strcmp(encoderName,"x264" )==0) { enc = &enc_x264; }

  if (enc==NULL) {
    fprintf(stderr, "unknown encoder");
    exit(5);
  }


  std::stringstream data_filename;
  data_filename << encoderName << "-" << inputName << "-" << preset[presetIdx].name << ".rd";
  output_fh = fopen(data_filename.str().c_str(), "wb");

  fprintf(output_fh,"# %s\n", preset[presetIdx].descr);

  std::vector<RDPoint> curve = enc->encode_curve(preset[presetIdx]);

  for (int i=0;i<curve.size();i++) {
    //fprintf(out_fh,"%7.2f %6.4f\n", curve[i].rate/1024, curve[i].psnr);
  }

  fclose(output_fh);

  return 0;
}
