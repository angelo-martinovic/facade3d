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
** FILENAME:    svlImageWindowExtractor.h
** AUTHOR(S):   Olga Russakovsky <olga@cs.stanford.edu>
**              based on applications by
**                     Stephen Gould <sgould@stanford.edu>
**                     Ian Goodfellow <ia3n@cs.stanford.edu>
** DESCRIPTION:
**  Extracts a training dataset of positive and negative image patches
**
*****************************************************************************/

#pragma once

#include <cassert>
#include <iostream>
#include <vector>
#include <map>

#include "cv.h"
#include "cxcore.h"
#include "ml.h"

#include "svlBase.h"
#include "svlVision.h"
#include "svlObjectList.h"
#include "svlSlidingWindowDetector.h"

using namespace std;

class svlImageWindowExtractor {
  
 public:
  static const string RANDOM_NEG_NAME;

 private:
  svlSlidingWindowDetector _windowMaker;
  svlObject2dFrame _allWindows;
  int _windowListWidth;
  int _windowListHeight;
  bool _bBaseScaleOnly; // set in constructor

  int _imageWidth;
  int _imageHeight;
  svlObject2dFrame _posObjects;
  svlObject2dFrame _otherObjects;

  // for extracting random negatives
  static const int MAX_NUM_TRIES = 100;
  static const int MIN_WIDTH = 32;
  static const int MIN_HEIGHT = 32;
  static CvRNG RNG_STATE;

  // initialized in constructor
  set<string> _posObjectNames;
  double _overlapThreshold;
  bool _bTextureOverlapDef; // defined overlap as (W & G)/W instead of (W & G)/(W | G)

 public:
  // (1) only objects with posNames from the groundtruth objects
  // will be considered positive; if empty, **all** ground truth objects
  // considered positive
  svlImageWindowExtractor(const set<string> & posNames,
			  double overlapThreshold = 0.5,
			  bool textureOverlapDef = false,
			  int slidingWindowsWidth = -1,
			  int slidingWindowsHeight = -1,
			  bool baseScaleOnly = false);

  // should be set for each frame
  void setImageSize(int w, int h);
  void setGroundTruthLabels(const svlObject2dFrame &_gt);
  void clearGroundTruthLabels();

  // returns all positive detections currently stored (clears result)
  void getPositives(svlObject2dFrame &result) const;

  // for each positive detection currently stored, returns the sliding
  // window with the greatest overlap
  void getBestPositiveWindows(svlObject2dFrame &result);

  // returns the top maxToGet detections (with pr greater than
  // threshold) that are not positive groundtruth objects;
  // will APPEND to the result list
  void getFalsePositives(svlObject2dFrame &result,
			 const svlObject2dFrame &detections,
			 double threshold = 0.0,
			 unsigned maxToGet = numeric_limits<unsigned>::max()) const;

  // returns all groundtruth objects not marked as positive;
  // will APPEND to the result list
  void getOtherObjects(svlObject2dFrame &result,
		       unsigned maxToGet = numeric_limits<unsigned>::max()) const;

  // will not return any windows that overlap positive groundtruth objects;
  // all objects returned will be named RANDOM_NEG_NAME
  // will APPEND to the result list
  void getRandomNegatives(svlObject2dFrame &result,
			  unsigned maxToGet = 100,
			  double negAspectRatio = 1.0, // width/height; < 0 => random
			  int negHeight = -1) const;   // <= 0 => random, >= MIN_HEIGHT
			  
  // returns all sliding windows; those that sufficiently overlap one
  // of the positive groundtruth objects are returned as positive, the
  // rest are returned as negative (clears result)
  void getAllPosWindows(svlObject2dFrame &result);
  void getAllNegWindows(svlObject2dFrame &result);
  // same functionality but more efficient than calling the above functions one by one
  void getAllWindows(svlObject2dFrame &posResult, svlObject2dFrame &negResult);  

 private:
  void getAllWindowsHelper(svlObject2dFrame &posResult, svlObject2dFrame &negResult,
			   bool inclPos, bool inclNeg);
  void checkWindowMaker();

  inline double computeOverlap(double windowArea, double gtArea, double overlapArea) {
    return _bTextureOverlapDef ? (overlapArea / windowArea) : (overlapArea / (gtArea + windowArea - overlapArea));
  }

};

