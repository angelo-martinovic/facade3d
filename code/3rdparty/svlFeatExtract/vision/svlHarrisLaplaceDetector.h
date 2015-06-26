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
** FILENAME:    svlHarrisLaplaceDetector.h
** AUTHOR(S):   Olga Russakovsky
** DESCRIPTION:
**   Implements svlScaledInterestPointDetector using Harris Laplace
** corner detection with automatic scale selection
**
*****************************************************************************/

#pragma once

#include "svlBase.h"
#include "svlInterestPointDetector.h"
#include "svlImageBufferManager.h"

class svlHarrisLaplaceDetector : public svlScaledInterestPointDetector
{
 public:
  static float DSCALE;
  static int NUM_SCALES;
  static float INIT_SCALE;

 public:
  // cornerness threshold is specified on 0...255 images
  svlHarrisLaplaceDetector(float cornerness_threshold,
			   int nscales = NUM_SCALES,
			   float initscale = INIT_SCALE,
			   float dscale = DSCALE);
  svlHarrisLaplaceDetector(svlHarrisLaplaceDetector &d);

  void findInterestPoints(const IplImage * img, vector<CvPoint> & output);

  void findInterestPointsScaled(const IplImage *img, vector<CvPoint> &output,
				vector<float> &scales);

  void write(ofstream & ofs) const;

  bool operator!=(const svlInterestPointDetector & other) const;

  // convert back to the input type
  float getThreshold() const {return _threshold * 255 * 255; }

 protected:
  float _threshold;
  int _numScales;
  float _initScale;
  float _dscale;
};
