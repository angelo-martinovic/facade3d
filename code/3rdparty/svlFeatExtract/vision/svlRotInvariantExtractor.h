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
** FILENAME:    svlRotInvariantExtractor.h
** AUTHOR(S):   Olga Russakovsky <olga@cs.stanford.edu>
** DESCRIPTION:
**   Extracts RIFT and Spin image features from images, cf.
**     Lazebnik, S.; Schmid, C.; Ponce, J., "A sparse texture
**     representation using local affine regions," Pattern Analysis
**     and Machine Intelligence, IEEE Transactions on , vol.27, no.8,
**     pp.1265-1278, Aug. 2005
**
*****************************************************************************/

#pragma once

#include <string>
#include <vector>
#include <fstream>
#include <cv.h>
#include "svlBase.h"
#include "svlML.h"
#include "svlImageBufferManager.h"
#include "svlPixelFeatureExtractor.h"

using namespace std;

// svlRotInvariantExtractor ---------------------------------------------

class svlRotInvariantExtractor : public svlPixelFeatureExtractor
{
 public:
  svlRotInvariantExtractor(int numDistBins, int numOtherBins,
			    int radius);
  svlRotInvariantExtractor(const svlRotInvariantExtractor &d);
  virtual ~svlRotInvariantExtractor();

  // required by superclass
  void getDescriptor(const vector<IplImage *> &responses, CvPoint loc,
		     vector<double> &output, int offset,
		     svlImageBufferManager &manager) const;

 protected:
  // given the image with ROI set to be _windowSize (or less),
  // extract the values to insert into the histogram
  // (the values are assumed to be normalized to the [0, 1] range)
  // - for RIFT images, these would be the gradient angles
  // and the gradient magnitudes for the weights
  // - for Spin, these would be the intensity values
  // with NULL for the weights
  virtual void extractOtherValues(const IplImage *image,
				  IplImage *&values,
				  IplImage *&weights,
				  svlImageBufferManager &manager) const = 0;

  // the standard is to just normalize
  virtual void postprocessFeatureVector(svl2dHistogram &hist) const
        { hist.normalize(); }

  bool _secondDimCircular;

 private:
  int _numDistBins;
  int _numOtherBins;
  int _supportRegionRadius;
  int _supportWindowDim;

  // precomputed in constructor
  svlSoftBinAssignment **_distAssignments;

};

// svlSpinDescriptor ----------------------------------------------------

class svlSpinDescriptor : public svlRotInvariantExtractor
{
 private:
  static const int NUM_INTENSITY_BINS = 10;
  static const int NUM_DISTANCE_BINS = 10;
  static const int REGION_RADIUS = 10;

  static const int NUM_BUFFERS = 1;
 
 public:
  svlSpinDescriptor();
  svlSpinDescriptor(const svlSpinDescriptor &d);
  virtual ~svlSpinDescriptor() {}

  // required by superclass
  unsigned numFeatures() const { return NUM_INTENSITY_BINS * NUM_DISTANCE_BINS; }
  bool writeOut(ofstream &ofs);
  bool load(XMLNode &root);
  svlFeatureExtractor *getPrunedExtractor(const vector<bool> &featureUsedBits) const;
  svlFeatureExtractor *clone() const;

 protected:
  void extractOtherValues(const IplImage *image,
			  IplImage *&values,
			  IplImage *&weights,
			  svlImageBufferManager &manager) const;

};

// svlRIFTextractor ----------------------------------------------------

class svlRIFTdescriptor : public svlRotInvariantExtractor
{
 private:
  static const int NUM_ORIENTATION_BINS = 8;
  static const int NUM_DISTANCE_BINS = 4;
  static const int REGION_RADIUS = 8;

  static const int NUM_BUFFERS = 4;

 public:
  svlRIFTdescriptor();
  svlRIFTdescriptor(const svlRIFTdescriptor &d);
  virtual ~svlRIFTdescriptor() {}
  
  // required by superclass
  unsigned numFeatures() const { return NUM_ORIENTATION_BINS * NUM_DISTANCE_BINS; }
  bool writeOut(ofstream &ofs);
  bool load(XMLNode &root);
  svlFeatureExtractor *getPrunedExtractor(const vector<bool> &featureUsedBits) const;
  svlFeatureExtractor *clone() const;

 protected:
  void extractOtherValues(const IplImage *image,
			  IplImage *&values,
			  IplImage *&weights,
			  svlImageBufferManager &manager) const;

  void postprocessFeatureVector(svl2dHistogram &hist);
};

SVL_AUTOREGISTER_H( RIFTdescriptor , svlFeatureExtractor);
SVL_AUTOREGISTER_H( SpinDescriptor , svlFeatureExtractor);
