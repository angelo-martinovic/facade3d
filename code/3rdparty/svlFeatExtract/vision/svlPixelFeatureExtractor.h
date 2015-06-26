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
** FILENAME: svlPixelFeatureExtractor.h
** AUTHOR(S): Olga Russakovsky <olga@cs.stanford.edu>
** DESCRIPTION: Abstract superclass of all local pixel
**  descriptors. The simplest case is when the descriptors are
**  extracted at each location individually from an IplImage; in this
**  case only the basic getDescriptor function needs to be implemented
**  (and storeImage, if there's pre-processing to be done on the entire
**  image).
**
*****************************************************************************/

#pragma once

#include <string>
#include <vector>
#include <fstream>
#include <cv.h>
#include "svlBase.h"
#include "svlVisionUtils.h"
#include "svlConvolution.h"
#include "svlImageBufferManager.h"
#include "svlFeatureExtractor.h"

using namespace std;

// svlPixelFeatureExtractor

class svlPixelFeatureExtractor : public svlFeatureExtractor {
 public:
  svlPixelFeatureExtractor() : _numBuffers(0) {}
  virtual ~svlPixelFeatureExtractor() {}

  virtual void extract(const vector<CvPoint> &locations,
	       const vector<svlDataFrame*> &frames,
	       vector<vector<double> > &output,
	       bool sparse,     // ignored for now
	       int outputOffset = 0) const;

  // for ease of use with scale-invariant interest point detectors
  virtual void getDescriptors(const IplImage *image,
			      const vector<CvPoint> &locations,
			      vector<vector<double> > &output,
			      const vector<float> *scales = NULL,
			      int outputOffset = 0) const;

  // two subroutines of getDescriptors
  virtual void preprocessImage(vector<IplImage *> &responses,
			  const IplImage *cimg, float scale = 1.0) const;

  // would be pure virual except user has option of overwriting
  // extract() or getDescriptors functions instead
  virtual void getDescriptor(const vector<IplImage*> &responses, CvPoint loc,
			     vector<double> &output, int offset,
			     svlImageBufferManager &manager) const {}

  // images are often used for precomputation; for efficiency,
  // buffers can be created for each class as requested and
  // passed into the getDescriptor function
  int _numBuffers; 
};
