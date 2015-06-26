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
** FILENAME:    svlMultiAspectAngleSWD.h
** AUTHOR(S):   Paul Baumstarck <pbaumstarck@stanford.edu>
** DESCRIPTION:
**  A sliding window detector for multiple aspect ratios and angles. Merely 
**  wraps an existing sliding window detector and calls it multiple times.
**
*****************************************************************************/

#pragma once

#ifndef _SVL_MULTI_AA_SWD_H
#define _SVL_MULTI_AA_SWD_H

#include <cassert>
#include <iostream>
#include <vector>
#include <map>
#include <string>

#include "cv.h"
#include "cxcore.h"
#include "ml.h"

#include "svlBase.h"
#include "svlVision.h"

using namespace std;

// svlCudaMultiAspectAngleSWD ---------------------------------------------------

class svlMultiAspectAngleSWD {
protected:
	// Wrapped sliding window detector.
	svlSlidingWindowDetector *_detector;
public:
	svlMultiAspectAngleSWD() {
		_detector = NULL;
	}
	svlMultiAspectAngleSWD(svlSlidingWindowDetector *detector) : _detector(detector) {}
	virtual ~svlMultiAspectAngleSWD() {}

	// Accessors.
	inline svlSlidingWindowDetector* getDetector() { return _detector; }
	inline void setDetector(svlSlidingWindowDetector *det) { _detector = det; }

	// Run classification/feature extraction at multiple aspects
	// and angles.
	
	// Default NULL arguments to aspectsin and anglesin uses
	// unchanged aspect ratio and 0-degree rotation.
	
	// Objects and vecs are cleared, with dim 1 mapped to the
	// aspects and dim 2 mapped to the angles requested.

	// Aguments: objects = return objects
	//           vecs = return vectors
	//           aspectsin = aspects to use (greater than one
	//                       squashes height; less than one
	//                       squashes width).
	//           anglesin = angles to use (in degrees, rotating
	//                      sliding window counter-clockwise)
	void classifyImage(const vector<IplImage*>& imagesin,
			   vector<vector<svlObject2dFrame> > *objects,
			   vector<vector<svlFeatureVectors> > *vecs = NULL,
			   vector<float> *aspectsin = NULL,
			   vector<float> *anglesin = NULL);
	// float version of above.
	void classifyImageF(const vector<IplImage*>& imagesin,
		vector<vector<svlObject2dFrame> > *objects,
		vector<vector<svlFeatureVectorsF> > *vecs = NULL,
		vector<float> *aspectsin = NULL,
		vector<float> *anglesin = NULL);

private:
	// Accepts both svlFeatureVectors and VectorsF.
	void classifyImage(const vector<IplImage*>& imagesin,
		vector<vector<svlObject2dFrame> > *objects,
		vector<vector<svlFeatureVectors> > *vecs = NULL,
		vector<vector<svlFeatureVectorsF> > *vecsF = NULL,
		vector<float> *aspectsin = NULL,
		vector<float> *anglesin = NULL);
};

#endif


