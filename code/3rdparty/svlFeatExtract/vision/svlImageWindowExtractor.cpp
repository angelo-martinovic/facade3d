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
** FILENAME:    svlImageWindowExtractor.cpp
** AUTHOR(S):   Olga Russakovsky <olga@cs.stanford.edu>
**              based on applications by
**                     Stephen Gould <sgould@stanford.edu>
**                     Ian Goodfellow <ia3n@cs.stanford.edu>
**
*****************************************************************************/

#include <cassert>
#include <iostream>
#include <fstream>
#include <vector>
#include <sys/types.h>
#include <limits>

#include "svlBase.h"
#include "svlVision.h"
#include "svlImageWindowExtractor.h"

#if defined(_WIN32)||defined(WIN32)||defined(__WIN32__)
#define _CRT_SECURE_NO_DEPRECATE
#include "win32/dirent.h"
#else
#include <dirent.h>
#endif

using namespace std;

// randRect -------------------------------------------------------------------
// Given a base size and frame size, this returns a rectangle in the frame that
// is sampled from the distribution consistent with what the classifier will
// see in the sliding windows (essentially, more small, fewer large images).

// [[TODO: use]]

// TODO: aspect ratio input is redundant, given base size.  Which is better for
// caller?

static CvRect randRect(double aspectRatio, CvSize base, CvSize frame, CvRNG *rngState);

CvRNG svlImageWindowExtractor::RNG_STATE = cvRNG(0xffffffff); // time(NULL)
const string svlImageWindowExtractor::RANDOM_NEG_NAME = string("random");

svlImageWindowExtractor::svlImageWindowExtractor(const set<string> & pos,
						 double overlapThreshold,
						 bool textureOverlapDef,
						 int slidingWindowWidth,
						 int slidingWindowHeight,
						 bool baseScaleOnly) :
  _windowMaker(NULL, slidingWindowWidth, slidingWindowHeight),
  _windowListWidth(-1), _windowListHeight(-1),_bBaseScaleOnly(baseScaleOnly),
  _posObjectNames(pos), _overlapThreshold(overlapThreshold),
  _bTextureOverlapDef(textureOverlapDef)
{
  // do nothing
}

void svlImageWindowExtractor::setImageSize(int w, int h)
{ 
  _imageWidth = w;
  _imageHeight = h;
}

void svlImageWindowExtractor::setGroundTruthLabels(const svlObject2dFrame &_gt) {
  _posObjects = _gt;
  _otherObjects.clear();
  
  if (!_posObjectNames.empty()) {
    removeNonMatchingObjects(_posObjects, _posObjectNames);

    _otherObjects = _gt;
    removeMatchingObjects(_otherObjects, _posObjectNames);
  }
}

void svlImageWindowExtractor::clearGroundTruthLabels() {
  _posObjects.clear();
  _otherObjects.clear();
}

void svlImageWindowExtractor::getPositives(svlObject2dFrame &result) const
{
  result.clear();
  result.reserve(_posObjects.size());

  for (unsigned k = 0; k < _posObjects.size(); k++) {
    svlObject2d object = _posObjects[k];
		      
    // check that r does not exceed bounds of image
    if (object.x+object.w >= _imageWidth)
      object.w = _imageWidth - 1 - object.x;
		      
    if (object.y+object.h >= _imageHeight)
      object.h = _imageHeight - 1 - object.y;
		      
    result.push_back(object);
  }
}

void svlImageWindowExtractor::getBestPositiveWindows(svlObject2dFrame &result)
{
  checkWindowMaker();

  result.clear();
  result.reserve(_posObjects.size());

  for (unsigned k = 0; k < _posObjects.size(); k++) {
    svlObject2d rect = _posObjects[k];

    svlObject2d bestWindow;
    double bestOverlap = -1;
			  
    double areaR = rect.area();
    for (unsigned win = 0; win < _allWindows.size(); win++) {
      double areaOverlap = rect.overlap(_allWindows[win]);
      double propOverlap = computeOverlap(_allWindows[win].area(), areaR, areaOverlap);
      if (propOverlap > bestOverlap) {
	bestWindow = _allWindows[win];
	bestOverlap = propOverlap;
      }
    }
    SVL_ASSERT(bestOverlap > 0);

    bestWindow.name = rect.name;    
    result.push_back(bestWindow);
  }
}

void svlImageWindowExtractor::getFalsePositives(svlObject2dFrame &result,
						const svlObject2dFrame &detections,
						double threshold,
						unsigned maxToGet) const
{
  svlObject2dFrame fp = detections;
  removeGroundTruthObjects(fp, _posObjects, _overlapThreshold);
  fp = sortObjects(fp);

  unsigned num;
  for (num = 0; num < min(maxToGet, (unsigned) fp.size()); num++) {
    if (fp[num].pr < threshold)
      break;
  }

  result.insert(result.end(), fp.begin(), fp.begin() + num);
}

void svlImageWindowExtractor::getOtherObjects(svlObject2dFrame &result,
					      unsigned maxToGet) const
{
  // in the future this can potentially do something smarter like
  // check the ratio of the other object to roughly correspond to what
  // we'd expect for the positive objects we're interested in, etc.
  unsigned num = min(maxToGet, (unsigned) _otherObjects.size());
  result.insert(result.end(), _otherObjects.begin(), _otherObjects.begin() + num);
}

void svlImageWindowExtractor::getRandomNegatives(svlObject2dFrame &result,
						 unsigned maxToGet,
						 double negAspectRatio,
						 int negHeight) const
{
  float imageRatio = (float)_imageWidth/_imageHeight;
  result.reserve(result.size() + maxToGet);
  for (unsigned j = 0; j < maxToGet; j++) {
    CvRect r;

    //Generate random rectangle
    if (negAspectRatio <= 0.0) {
      r.width = (cvRandInt(&RNG_STATE) % (_imageWidth - MIN_WIDTH)) + MIN_WIDTH;
      r.height = (cvRandInt(&RNG_STATE) % (_imageHeight - MIN_HEIGHT)) + MIN_HEIGHT;
    }
    else if (negAspectRatio >= imageRatio) {
      if (negHeight <= 0) {
	// want to generate width first to avoid running over the image size
	r.width = (cvRandInt(&RNG_STATE) % (_imageWidth - MIN_WIDTH)) + MIN_WIDTH;
	r.height = (int)(r.width / negAspectRatio);
      } else {
	r.height = negHeight;
	r.width = (int)(negAspectRatio * r.height);
      }
    }
    else { // 0 < negAspectRatio < imageRatio
      if (negHeight <= 0) {
	// want to generate height first to avoid running over the image size
	r.height = (cvRandInt(&RNG_STATE) % (_imageHeight - MIN_HEIGHT)) + MIN_HEIGHT;
	r.width = (int)(negAspectRatio * r.height);
      } else {
	r.height = negHeight;
	r.width = (int)(negAspectRatio * r.height);
      }
    }

    SVL_ASSERT_MSG(r.width <= _imageWidth && r.height <= _imageHeight,
		   "generated a random negative of size " << r.width
		   << "x" << r.height << " from an image of size "
		   << _imageWidth << "x" << _imageHeight);

    // check for overlap with groundtruth objects in the image
    bool found;
    int numTries = 0;
    
    do {
      found = true;

      r.x = cvRandInt(&RNG_STATE) % (_imageWidth - r.width);
      r.y = cvRandInt(&RNG_STATE) % (_imageHeight - r.height);

      svlObject2d temp(r.x,r.y,r.width,r.height);

      // search through all the objects (as long as you still believe you found a good location)
      for (unsigned k =0; k < _posObjects.size(); k++) {
	if (_posObjects[k].overlap(temp) != 0) {
	  found = false;
	  break;
	}
      }

      // it's possible that there's just no good position
      // to put it; in that case, continue
      numTries++;
    } while (!found && numTries < MAX_NUM_TRIES);

    if (found) {
      svlObject2d obj(r);
      obj.name = RANDOM_NEG_NAME;
      result.push_back(obj);
    }
  }
}

void svlImageWindowExtractor::getAllPosWindows(svlObject2dFrame &result)
{
  svlObject2dFrame tmp;
  getAllWindowsHelper(result, tmp, true, false); 
}
void svlImageWindowExtractor::getAllNegWindows(svlObject2dFrame &result)
{
  svlObject2dFrame tmp;
  getAllWindowsHelper(tmp, result, false, true);
}
void svlImageWindowExtractor::getAllWindows(svlObject2dFrame &posResult,
					    svlObject2dFrame &negResult)
{
  getAllWindowsHelper(posResult, negResult, true, true);
}

void svlImageWindowExtractor::getAllWindowsHelper(svlObject2dFrame &posResult,
						  svlObject2dFrame &negResult,
						  bool inclPos, bool inclNeg)
{
  checkWindowMaker();

  posResult.clear();
  negResult.clear();

  // optimization; chances are, most windows are negative,
  // and there's like 50,000 of those
  if (inclNeg)
    negResult.reserve(_allWindows.size());

  for (unsigned win = 0; win < _allWindows.size(); win++) {
    double areaWin = _allWindows[win].area();

    bool positive = false;
    unsigned obj;
    for (obj = 0; obj < _posObjects.size(); obj++) {
      double areaObj = _posObjects[obj].area();
      double areaOverlap = _allWindows[win].overlap(_posObjects[obj]);
      double propOverlap = computeOverlap(areaWin, areaObj, areaOverlap);

      if (propOverlap > _overlapThreshold) {
	positive = true;
	break;
      }
    }

    CvRect r = cvRect((int) _allWindows[win].x, (int) _allWindows[win].y,
		      (int) _allWindows[win].w, (int) _allWindows[win].h);
    svlObject2d object(r);

    if (positive && inclPos) {
	object.name = _posObjects[obj].name;
	posResult.push_back(object);
    }
    else if (!positive && inclNeg) {
      negResult.push_back(object);
    }
  }
}

void svlImageWindowExtractor::checkWindowMaker()
{
  if (_windowListWidth != _imageWidth || _windowListHeight != _imageHeight) {
    _windowListWidth = _imageWidth;
    _windowListHeight = _imageHeight;

    if (_bBaseScaleOnly)
      _windowMaker.getWindowsAtBaseScale(_windowListWidth, _windowListHeight, _allWindows);
    else
      _windowMaker.getAllWindows(_windowListWidth, _windowListHeight, _allWindows);
  }
}

CvRect randRect(double aspectRatio, CvSize base, CvSize frame, CvRNG *rngState)
{
  const double SEL_FACTOR = 100000000.0;
  const double SEL_FACTOR_SQRT = 10000.0;
  int minSel, maxSel;
  CvRect rect;

  // TODO: minSel and maxSel calculation can be done once, given the frame
  // size and base size
  if (frame.width / frame.height > aspectRatio) {
    // Frame is wider than aspect, rectangle is constrained by height
    minSel = (int) ceil(SEL_FACTOR / pow(frame.height, 2.0));
    maxSel = (int) floor(SEL_FACTOR / pow(base.height, 2.0));

    float sel = (float) (cvRandInt(rngState) % (maxSel - minSel) + minSel);
    rect.height = (int) (SEL_FACTOR_SQRT / sqrt(sel));
    rect.width = (int) (rect.height * aspectRatio);
  } else {
    // Frame is thinner than aspect, rectangle is constrained by width
    minSel = (int) ceil(SEL_FACTOR / pow(frame.width, 2.0));
    maxSel = (int) floor(SEL_FACTOR / pow(base.width, 2.0));

    float sel = (float) (cvRandInt(rngState) % (maxSel - minSel) + minSel);
    rect.width = (int) (SEL_FACTOR_SQRT / sqrt(sel));
    rect.height = (int) (rect.width / aspectRatio);
  }

  rect.x = cvRandInt(rngState) % (frame.width - rect.width);
  rect.y = cvRandInt(rngState) % (frame.height - rect.height);

  return rect;
}








