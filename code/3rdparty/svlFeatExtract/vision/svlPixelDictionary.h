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
** FILENAME:    svlPixelDictionary.h
** AUTHOR(S):   Olga Russakovsky <olga@cs.stanford.edu>
** DESCRIPTION:
**   Defines a superclass for all interest point feature extractors
** that are accumulated over a region and vector-quantized to form a
** feature histogram
**
*****************************************************************************/

#pragma once

#include <string>
#include <sstream>
#include <vector>
#include <fstream>
#include <cv.h>
#include "xmlParser/xmlParser.h"

#include "svlBase.h"
#include "svlML.h"
#include "svlHarrisLaplaceDetector.h"
#include "svlPixelFeatureExtractor.h"
#include "svlFeatureExtractor.h"

using namespace std;

class svlPixelDictionary : public svlFeatureExtractor
{
 public:
  svlPixelDictionary(int w = -1, int h = -1);
  svlPixelDictionary(const svlPixelDictionary &d);
  ~svlPixelDictionary();

  void setIPdetector(svlHarrisLaplaceDetector *det)
    { _ipDetector = new svlHarrisLaplaceDetector(*det); }
  void addDescriptor(svlPixelFeatureExtractor *d) 
  { _descriptors.push_back(dynamic_cast<svlPixelFeatureExtractor *>(d->clone())); }
  
  // overridden by the IP detector, if it exists
  void setNumPointsPerImage(int n) { _numPointsPerSample = n;}

  // builds a dictionary given _windowSize x _windowSize samples by
  // vector-quantization; numPointsPerImage is used if no interest
  // point detector is specified; numPointsPerImage < 0 => take all
  // points within the image; optionally the featureVectors will return the 
  // features computed over the samples so don't need to immediatelly
  // call extract
  void buildDictionary(const vector<IplImage *> &samples,
		       int numClusters,
		       vector<vector<double> > *featureVectors = NULL);

  // required by the superclass; if a dictionary was built, then uses a
  // histogram of vector-quantized features over _windowSize x
  // _windowSize region as the output; otherwise, uses just the 
  // raw features at the specified locations
  void extract(const vector<CvPoint> &locations,
	       const vector<svlDataFrame*> &frames,
	       vector< vector<double> > &output,
	       bool sparse,
	       int outputOffset = 0) const;

  bool writeOut(ofstream &ofs);
  bool load(XMLNode &root);

  unsigned numFeatures() const;

  void appendToCodebook(svlPixelDictionary *d) {
    _codebook.append(d->_codebook);
  }

  // if want to switch from vector-quantized version to the regular feature
  // space (see extract()).
  void clearCodebook() { _codebookTrained = false; }

  // deletes the interest point detector, the descriptors, and the codebook
  void clear(); 

  void clearDetector();

  // not yet implemented
  svlFeatureExtractor* getPrunedExtractor(const vector<bool> &featureUsedBits) const;

  svlFeatureExtractor * clone() const;

  // given an image, extracts a set of descriptors based on the
  // interest point detector or, only if that's not present, on the
  // number of points per image currently specified (can be
  // temporarily overwritten here); returns a vector of locations
  // along with the correponding descriptors
  void extractDescriptors(IplImage *img, vector<CvPoint> &locations,
			  vector<vector<double> > &descriptors,
			  int *numPointsPerImage = NULL) const;

 protected:
  // todo: use the general svlScaledInterestPointDetector class
  svlHarrisLaplaceDetector *_ipDetector;
  vector<svlPixelFeatureExtractor *> _descriptors;
  int _numPointsPerSample; // used if _ipDetector is not provided

 private:
  svlCodebook _codebook;
  bool _codebookTrained;

  unsigned _validChannel;
};

SVL_AUTOREGISTER_H( PixelDictionary , svlFeatureExtractor);
