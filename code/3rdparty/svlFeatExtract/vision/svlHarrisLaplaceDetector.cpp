/*****************************************************************************
** STAIR VISION LIBRARY
** Copyright (c) 2007-2010, Stephen Gould
** All rights reserved.
**
** Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are met:
**     * Redistributions of source code must retain the above copyright
**       notice, this list of conditions and the following disclaimer.
**     * Redistributions in binary form must reproduce the above copyright
**       notice, this list of conditions and the following disclaimer in the
**       documentation and/or other materials provided with the distribution.
**     * Neither the name of the Stanford University nor the
**       names of its contributors may be used to endorse or promote products
**       derived from this software without specific prior written permission.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
** EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
** WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
** DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY
** DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
** (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
** LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
** ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
** SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**
******************************************************************************
** FILENAME:    svlHarrisLaplaceDetector.cpp
** AUTHOR(S):   Olga Russakovsky <olga@cs.stanford.edu>
**
*****************************************************************************/

#include "svlVision.h"
#include "svlHarrisLaplaceDetector.h"

float svlHarrisLaplaceDetector::DSCALE = 1.2f;
int svlHarrisLaplaceDetector::NUM_SCALES = 15;
float svlHarrisLaplaceDetector::INIT_SCALE = 1.0f;

svlHarrisLaplaceDetector::svlHarrisLaplaceDetector(float cornerness_threshold,
						   int nscales, float initscale,
						   float dscale) :
  _threshold(cornerness_threshold / (255 * 255)),
  _numScales(nscales), _initScale(initscale), _dscale(dscale)
{
  // do nothing
}

svlHarrisLaplaceDetector::svlHarrisLaplaceDetector(svlHarrisLaplaceDetector &d) :
  _threshold(d._threshold), _numScales(d._numScales), _initScale(d._initScale),
  _dscale(d._dscale)
{
  // do nothing
}

void svlHarrisLaplaceDetector::findInterestPoints(const IplImage * cimg,
						 vector<CvPoint> & output)
{
  vector<float> tmp;
  findInterestPointsScaled(cimg, output, tmp);
}

#define CREATE(img) cvCreateImage(cvSize(img->width, img->height), IPL_DEPTH_32F, 1)
#define DESTROY(img) cvReleaseImage(&img)

// [[TODO: optimize the smoothing at higher scales]]
void svlHarrisLaplaceDetector::findInterestPointsScaled(const IplImage *cimg,
							vector<CvPoint> &output,
							vector<float> &scales)
{
  SVL_ASSERT(cimg);
  IplImage *img = create32Fimage(cimg);
  if (img->nChannels > 1) {
    svlChangeColorModel(img, SVL_COLOR_GRAY);
  }

  output.clear();

  IplImage *LoG_prev = CREATE(img);
  IplImage *LoG_curr = CREATE(img);
  IplImage *LoG_next = CREATE(img);

  IplImage *smoothed = CREATE(img);
  IplImage *dx = CREATE(img);
  IplImage *dy = CREATE(img);
  IplImage *dxdx = CREATE(img);
  IplImage *dxdy = CREATE(img);
  IplImage *dydy = CREATE(img);
  IplImage *cornerness = CREATE(img);

  int handle;
  float scale = _initScale;

  // compute the previous and current LoG image
  svlCodeProfiler::tic(handle = svlCodeProfiler::getHandle("HL: LoG"));
  float scale_prev = scale / _dscale;
  int n = kernelSize(scale_prev);
  svlLoGConvolution LoG_conv1(n, n, scale_prev);
  LoG_conv1.filter(img, LoG_prev);
  cvScale(LoG_prev, LoG_prev, scale_prev * scale_prev);

  n = kernelSize(scale);
  svlLoGConvolution LoG_conv(n, n, scale);
  LoG_conv.filter(img, LoG_curr);
  cvScale(LoG_curr, LoG_curr, scale * scale);
  svlCodeProfiler::toc(handle);

  // iterate over all scales
  for (int i = 0; i < _numScales; i++, scale *= _dscale) {
    // compute the next LoG image
    svlCodeProfiler::tic(handle = svlCodeProfiler::getHandle("HL: LoG"));
    float scale_next = scale * _dscale;
    n = kernelSize(scale_next);
    svlLoGConvolution LoG_conv2(n, n, scale_next);
    LoG_conv2.filter(img, LoG_next);
    cvScale(LoG_next, LoG_next, scale_next * scale_next);
    svlCodeProfiler::toc(handle);
    
    // compute L_x(sigma_D), L_y(sigma_D)
    svlCodeProfiler::tic(handle = svlCodeProfiler::getHandle("HL: L_x, L_y"));
    float diff_scale = (_dscale/2) * scale;
    n = kernelSize(diff_scale);
    cvSmooth(img, smoothed, CV_GAUSSIAN, n, n, diff_scale);
    cvSobel(smoothed, dx, 1, 0, 1);
    cvSobel(smoothed, dy, 0, 1, 1);
    svlCodeProfiler::toc(handle);

    // compute the entries of the Harris matrix
    svlCodeProfiler::tic(handle = svlCodeProfiler::getHandle("HL: L_xx, L_yy, L_xy"));
    cvMul(dx, dx, dxdx);
    cvMul(dx, dy, dxdy);
    cvMul(dy, dy, dydy);
    svlCodeProfiler::toc(handle);

    // smooth the Harris matrix by sigma_I
    svlCodeProfiler::tic(handle = svlCodeProfiler::getHandle("HL: smooth Harris matrix"));
    n = kernelSize(scale);
    cvSmooth(dxdx, dxdx, CV_GAUSSIAN, n, n, scale);
    cvSmooth(dxdy, dxdy, CV_GAUSSIAN, n, n, scale);
    cvSmooth(dydy, dydy, CV_GAUSSIAN, n, n, scale);
    svlCodeProfiler::toc(handle);
    
    // compute the measure of cornerness
    svlCodeProfiler::tic(handle = svlCodeProfiler::getHandle("HL: cornerness measure"));
    for (int y = 0; y < img->height; y++) {
      float *corner_ptr = &CV_IMAGE_ELEM(cornerness, float, y, 0);
      const float *dxdx_ptr = &CV_IMAGE_ELEM(dxdx, float, y, 0);
      const float *dxdy_ptr = &CV_IMAGE_ELEM(dxdy, float, y, 0);
      const float *dydy_ptr = &CV_IMAGE_ELEM(dydy, float, y, 0);

      for (int x = 0; x < img->width; x++) {
	float a = dxdx_ptr[x];
	float b = dxdy_ptr[x];
	float c = dydy_ptr[x];
	//if (x %100 == 0 && y % 100 == 0)
	  //cerr << "x " << x << " y " << y << " a " << a << " b " << b
	  //<< " c" << c << endl;
	corner_ptr[x] = a * c - b * b - 0.04 * (a + c) * (a + c);
      }
    }
    svlCodeProfiler::toc(handle);

    // instead of scaling every measure of cornerness, just scale down
    // the threshold
    float diff_scale_sq = diff_scale * diff_scale; 
    float thr = _threshold / (diff_scale_sq * diff_scale_sq);

    // non-maximal suppression
    svlCodeProfiler::tic(handle = svlCodeProfiler::getHandle("HL: non-max suppression"));
    for (int y = 1; y < img->height-1; y++) {
      const float *prev_row = &CV_IMAGE_ELEM(cornerness, float, y-1, 0);
      const float *cur_row = &CV_IMAGE_ELEM(cornerness, float, y, 0);
      const float *next_row = &CV_IMAGE_ELEM(cornerness, float, y+1, 0);

      const float *prev_scale_row = &CV_IMAGE_ELEM(LoG_prev, float, y, 0);
      const float *curr_scale_row = &CV_IMAGE_ELEM(LoG_curr, float, y, 0);
      const float *next_scale_row = &CV_IMAGE_ELEM(LoG_next, float, y, 0);

      for (int x = 1; x < img->width-1; x++) {
	float value = cur_row[x];
	if (value < thr) continue;

	// local maximum in space
	if (value > prev_row[x-1] && value > prev_row[x] &&
	    value > prev_row[x+1] && value > cur_row[x-1] &&
	    value > cur_row[x+1] && value > next_row[x-1] &&
	    value > next_row[x] && value > next_row[x+1]) {

	  // local maximum in scale
	  value = curr_scale_row[x];
	  if (value > prev_scale_row[x] && value > next_scale_row[x]) {
	    output.push_back(cvPoint(x, y));
	    scales.push_back(scale);
	  }
	} // finish the if checks
      } // end the x loop
    } // end the y loop
    svlCodeProfiler::toc(handle);

    // exchange pointers to LoG
    IplImage *tmp = LoG_prev;
    LoG_prev = LoG_curr;
    LoG_curr = LoG_next;
    LoG_next = tmp;
  } // end the scales loop

  cvReleaseImage(&img);

  DESTROY(LoG_prev);
  DESTROY(LoG_prev);
  DESTROY(LoG_curr);
  DESTROY(LoG_next);

  DESTROY(smoothed);
  DESTROY(dx);
  DESTROY(dy);
  DESTROY(dxdx);
  DESTROY(dxdy);
  DESTROY(dydy);
  DESTROY(cornerness);
}

void svlHarrisLaplaceDetector::write(ofstream & ofs) const
{
  SVL_LOG(SVL_LOG_FATAL, "not implemented yet");
}


bool svlHarrisLaplaceDetector::operator!=(const svlInterestPointDetector & other) const
{
    SVL_LOG(SVL_LOG_FATAL, "not implemented yet");
    return false; // to suppress warnings
}

class svlHarrisLaplaceDetectorConfig : public svlConfigurableModule {
public:
  svlHarrisLaplaceDetectorConfig() : svlConfigurableModule("svlVision.svlHarrisLaplaceDetector") {}

  void usage(ostream &os) const {
    os << "      initScale     :: initial scale for detection (default: 1)\n"
       << "      delta         :: change in scales between successive levels (default: 1.2)\n"
       << "      numScales     :: number of scales to consider (default: 15)\n";
  }

  void setConfiguration(const char *name, const char *value) {
    if (!strcmp(name, "initScale")) {
      svlHarrisLaplaceDetector::INIT_SCALE = atof(value);
    } else if (!strcmp(name, "delta")) {
      svlHarrisLaplaceDetector::DSCALE = atof(value);
    } else if (!strcmp(name, "numScales")) {
      svlHarrisLaplaceDetector::NUM_SCALES = atoi(value);
    } else {
      SVL_LOG(SVL_LOG_FATAL, "unrecognized configuration option for " << this->name());
    }
  }
};

static svlHarrisLaplaceDetectorConfig gSlidingWindowDetectorConfig;
