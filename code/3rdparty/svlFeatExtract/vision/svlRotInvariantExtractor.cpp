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
** FILENAME:    svlRotInvariantExtractor.cpp
** AUTHOR(S):   Olga Russakovsky <olga@cs.stanford.edu>
**
*****************************************************************************/

#include "svlRotInvariantExtractor.h"

// svlRotInvariantExtractor ----------------------------------------------------

svlRotInvariantExtractor::svlRotInvariantExtractor(int numDistBins,
						     int numOtherBins,
						     int radius) : 
  _secondDimCircular(false), _numDistBins(numDistBins), _numOtherBins(numOtherBins),
  _supportRegionRadius(radius), _supportWindowDim(2 * radius + 1)
{
  // allocate the distance assignments
  _distAssignments = new svlSoftBinAssignment*[_supportWindowDim];
  for (int i = 0; i < _supportWindowDim; i++)
    _distAssignments[i] = new svlSoftBinAssignment[_supportWindowDim];

  // precompute the distance assignments
  svlHistogramOperators ops(numDistBins, 0, radius);
  int centerPixel = radius;
  for (int i = 0; i < _supportWindowDim; i++) {
    for (int j = 0; j < _supportWindowDim; j++) {
      int dx = centerPixel - i;
      int dy = centerPixel - j;
      double dist = sqrt((double)(dx * dx + dy * dy));
      svlSoftBinAssignment sb;
      if (ops.getBinNumLinearSmooth(dist, sb))
	_distAssignments[i][j] = sb;
    }
  }
}

svlRotInvariantExtractor::svlRotInvariantExtractor(const svlRotInvariantExtractor &d) :
  _secondDimCircular(d._secondDimCircular),
  _numDistBins(d._numDistBins), _numOtherBins(d._numOtherBins),
  _supportRegionRadius(d._supportRegionRadius), _supportWindowDim(d._supportWindowDim)
{
  _windowSize = d._windowSize;
  _numBuffers = d._numBuffers;

  _distAssignments = new svlSoftBinAssignment*[_supportWindowDim];
  for (int i = 0; i < _supportWindowDim; i++) {
    _distAssignments[i] = new svlSoftBinAssignment[_supportWindowDim];
    for (int j = 0; j < _supportWindowDim; j++)
      _distAssignments[i][j] = d._distAssignments[i][j];
  }
}

svlRotInvariantExtractor::~svlRotInvariantExtractor()
{
  for (int i = 0; i < _supportWindowDim; i++)
    delete[] _distAssignments[i];
  delete[] _distAssignments;
}

void svlRotInvariantExtractor::getDescriptor(const vector<IplImage *> &responses, CvPoint loc,
					     vector<double> &output, int offset,
					     svlImageBufferManager &manager) const
{
  // assumes that the img is a 32F image at the correct scale
  SVL_ASSERT(responses.size() == 1);

  IplImage *img = responses[0];
  SVL_ASSERT(img);

  // initialize the histogram to fill in this feature vector
  svl2dHistogram hist(_numDistBins, 0, _supportRegionRadius, // maximum distance
		      _numOtherBins, 0, 1.0,    // maximum value of second dimension
		      &output, offset);
  if (_secondDimCircular)
    hist.makeSecondCircular();

  // figure out the feature support window, and by how much
  // it's sticking out of the image on the left and top
  // to be able to relate the values vector to the distance
  // from the center
  int roi_x = loc.x - _supportRegionRadius;
  int roi_y = loc.y - _supportRegionRadius;
  int clippedTop = (roi_y < 0 ? -roi_y : 0);
  int clippedLeft = (roi_x < 0 ? -roi_x : 0);
  CvRect roi = cvRect(roi_x, roi_y, _supportWindowDim, _supportWindowDim);
  svlClipRect(roi, img);

  int handle;
  svlCodeProfiler::tic(handle = svlCodeProfiler::getHandle("extract::bin assignments"));
  cvSetImageROI(img, roi); 
  IplImage *values = NULL;
  IplImage *weights = NULL;
  extractOtherValues(img, values, weights, manager);
  cvResetImageROI(img);

  for (int y = 0; y < cvGetSize(values).height; y++) {
    const float *values_ptr = &CV_IMAGE_ELEM(values, float, y, 0);
    const float *weights_ptr = NULL;
    if (weights) {
      weights_ptr = &CV_IMAGE_ELEM(weights, float, y, 0);
    }
    for (int x = 0; x < cvGetSize(values).width; x++) {
      // put the values corresponding to this location into the
      // histogram: the first dimension is the distance from the
      // center, the second is the value that was just precomputed
      // in extractOtherValues
      hist.insertLinearSmooth(_distAssignments[x+clippedLeft][y+clippedTop],
			       values_ptr[x],
			       weights ? weights_ptr[x] : 1.0);
    }
  }

  postprocessFeatureVector(hist);
  svlCodeProfiler::toc(handle);
}


// svlSpinDescriptor ----------------------------------------------------

svlSpinDescriptor::svlSpinDescriptor() :
  svlRotInvariantExtractor(NUM_DISTANCE_BINS, NUM_INTENSITY_BINS,
			    REGION_RADIUS)
{
  _numBuffers = NUM_BUFFERS;
}

svlSpinDescriptor::svlSpinDescriptor(const svlSpinDescriptor &d) :
  svlRotInvariantExtractor(d)
{
  // do nothing
}

bool svlSpinDescriptor::writeOut(ofstream &ofs)
{
  if (ofs.fail()) return false;
  ofs << "<svlFeatureExtractor id=\"SpinDescriptor\"/>\n";  
  return true;
}

bool svlSpinDescriptor::load(XMLNode &node)
{
  if (node.isEmpty()) {
    SVL_LOG(SVL_LOG_WARNING, "Spin descriptor file is missing or has incorrect root.");
    return false;
  }

  if (string(node.getAttribute("id")) != "SpinDescriptor" ) {
    SVL_LOG(SVL_LOG_WARNING, "Attempt to read spin descriptor from XMLNode that is not it.");
    return false;
  }

  return true;
}

svlFeatureExtractor* svlSpinDescriptor::getPrunedExtractor(const vector<bool> &featureUsedBits) const
{
  SVL_LOG(SVL_LOG_FATAL, "not implemented yet");
  return NULL; // to suppress warnings
}

svlFeatureExtractor * svlSpinDescriptor::clone() const
{
  return dynamic_cast<svlFeatureExtractor *>(new svlSpinDescriptor(*this));
}


void svlSpinDescriptor::extractOtherValues(const IplImage *image,
					   IplImage *&values,
					   IplImage *&weights,
					   svlImageBufferManager &manager) const
{
  int handle = svlCodeProfiler::getHandle("spin::extractOtherValues");
  
  // allocate the necessary amount of space
  manager.allocateBuffers(cvGetSize(image));

  values = manager.get(0);
  weights = NULL;

  // returns normalized intensity values
  double min, max;
  cvMinMaxLoc(image, &min, &max);
  if (min == max)
    SVL_LOG(SVL_LOG_WARNING, "svlSpinDescriptor::extractOtherValues -- "
	    << "patch is uniform, skipping normalization");

  float scale = 1.0 / (max - min + 0.001); // to ensure the values are in [0, 1)
  cvScale(image, values, scale, - min * scale);
  svlCodeProfiler::toc(handle);
}



// svlRIFTdescriptor ----------------------------------------------------

svlRIFTdescriptor::svlRIFTdescriptor() :
  svlRotInvariantExtractor(NUM_DISTANCE_BINS, NUM_ORIENTATION_BINS,
			    REGION_RADIUS)
{
  // makes the second dimension circular
  _secondDimCircular = true;
  _numBuffers = NUM_BUFFERS;
}

svlRIFTdescriptor::svlRIFTdescriptor(const svlRIFTdescriptor &d) :
  svlRotInvariantExtractor(d)
{
  // do nothing
}


// todo: better approximation

static const int MAX_ORIENTATION = 8;
static inline float approx_angle(float dx, float dy)
{
  if (dy >= 0) {
    if (dx >= 0) {
      // quadrant I
      if (dx >= dy) {
	return 0.5 / MAX_ORIENTATION;
      } else {
	return 1.5 / MAX_ORIENTATION;
      }
    } else {
      // quadrant II
      if (dy >= -dx) {
	return 2.5 / MAX_ORIENTATION;
      } else {
	return 3.5 / MAX_ORIENTATION;
      }
    }
  } else {
    if (dx < 0) {
      // quadrant III
      if (dy >= dx) {
	return 4.5 / MAX_ORIENTATION;
      } else {
	return 5.5 / MAX_ORIENTATION;
      }
    } else {
      // quadrant IV
      if (-dy >= dx) {
	return 6.5 / MAX_ORIENTATION;
      } else {
	return 7.5 / MAX_ORIENTATION;
      }
    }
  }
}

bool svlRIFTdescriptor::writeOut(ofstream &ofs)
{
  if (ofs.fail()) return false;
  ofs << "<svlFeatureExtractor id=\"RIFTdescriptor\"/>\n";  
  return true;
}

bool svlRIFTdescriptor::load(XMLNode &node)
{
  if (node.isEmpty()) {
    SVL_LOG(SVL_LOG_WARNING, "RIFT descriptor file is missing or has incorrect root.");
    return false;
  }

  if (string(node.getAttribute("id")) != "RIFTdescriptor" ) {
    SVL_LOG(SVL_LOG_WARNING, "Attempt to read RIFT descriptor from XMLNode that is not it.");
    return false;
  }

  return true;
}

svlFeatureExtractor* svlRIFTdescriptor::getPrunedExtractor(const vector<bool> &featureUsedBits) const
{
  SVL_LOG(SVL_LOG_FATAL, "not implemented yet");
  return NULL; // to suppress warnings
}

svlFeatureExtractor * svlRIFTdescriptor::clone() const
{
  return dynamic_cast<svlFeatureExtractor *>(new svlRIFTdescriptor(*this));
}

void svlRIFTdescriptor::extractOtherValues(const IplImage *img,
					   IplImage *&values,
					   IplImage *&weights,
					   svlImageBufferManager &manager) const
{
  int handle = svlCodeProfiler::getHandle("RIFT::extractOtherValues");

  svlCodeProfiler::tic(handle = svlCodeProfiler::getHandle("RIFT::Sobel"));
  // allocate the necessary amount of space
  manager.allocateBuffers(cvGetSize(img));

  // compute the direction and magnitude of each point
  // in the image
  IplImage *dx = manager.get(0);
  IplImage *dy = manager.get(1);

  cvSobel(img, dx, 1, 0, 1);
  cvSobel(img, dy, 0, 1, 1);
  svlCodeProfiler::toc(handle);

  values = manager.get(2);
  weights = manager.get(3);

  for (int y = 0; y < values->height; y++) {
    const float *dx_row_ptr = &CV_IMAGE_ELEM(dx, float, y, 0);
    const float *dy_row_ptr = &CV_IMAGE_ELEM(dy, float, y, 0);
    float *values_ptr = &CV_IMAGE_ELEM(values, float, y, 0);
    float *magn_ptr = &CV_IMAGE_ELEM(weights, float, y, 0);

    for (int x = 0; x < values->width; x++) {
      float dx_val = dx_row_ptr[x];
      float dy_val = dy_row_ptr[x];
      values_ptr[x] = approx_angle(dx_val, dy_val);
      magn_ptr[x] = sqrt(dx_val*dx_val + dy_val*dy_val);
    }
  }

  svlCodeProfiler::toc(svlCodeProfiler::getHandle("RIFT::extractOtherValues"));
}

void svlRIFTdescriptor::postprocessFeatureVector(svl2dHistogram &hist)
{
  // normalizing the histogram, c.f. Lowe's 2004 SIFT paper, pg. 16
  // second paragraph
  hist.normalize();
  hist.truncate(0.2);
  hist.normalize();
}








/*
static void test_bin()
{
cerr << "Testing bin assignments" << endl;
double dx[8] = {0.8, 0.2, -0.2, -0.8, -0.8, -0.2, 0.2, 0.8};
double dy[8] = {0.2, 0.8, 0.8, 0.2, -0.2, -0.8, -0.8, -0.2};

for (int i = 0; i < 8; i++)
cerr << "(" << dx[i] << ", " << dy[i] << " assigned to " 
<< approx_angle(dx[i], dy[i]) - 0.5 << ", expected "
<< i << endl;

}*/

/*void svlRotInvariantExtractor::check_distance_bins() 
{
  for (int i = 0; i < WINDOW_SIZE; i++) {
    for (int j = 0; j < WINDOW_SIZE; j++) {
      svlSoftBinAssignment curr = _distAssignments[i][j];

      cerr << "(" << i << ", " << j << "): "
	   << curr._bin[0] << " " << curr._binWeight[0]
	   << "\t" << curr._bin[1] << " " << curr._binWeight[1]
	   << endl;
    }
  }
  }*/

