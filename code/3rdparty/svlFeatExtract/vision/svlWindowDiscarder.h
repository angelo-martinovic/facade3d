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
** FILENAME:    svlSlidingWindowDetector.h
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
**              Sid Batra <sidbatra@stanford.edu>
**              Ian Goodfellow <ia3n@cs.stanford.edu>
**              Olga Russakovsky <olga@cs.stanford.edu>
**				Paul Baumstarck <pbaumstarck@stanford.edu>
** DESCRIPTION:
**  Trims bad locations from use by a sliding window detector.
**
*****************************************************************************/

#pragma once

#include <cassert>
#include <iostream>
#include <vector>

#include "cv.h"
#include "cxcore.h"

#include "svlBase.h"
using namespace std;


// svlWindowDiscarder ----------------------------------------------------

class svlWindowDiscarder {
protected:
	int _windowWidth, _windowHeight;
public:
	svlWindowDiscarder(int w = 32, int h = 32);
	virtual ~svlWindowDiscarder();
	void setSize(int w, int h) { _windowWidth = w; _windowHeight = h; }
	// discards locations that would go beyond the size of the image
	void discardInvalid(int imageWidth, int imageHeight, vector<CvPoint> &locations) const;

	// can be overwritten; the default is to check for uniformity of the image,
	// or of the *first* image in the list of vectors (we're assuming it corresponds
	// to the intensity image) 
	virtual void discardUninteresting(IplImage *image, double scale, vector<CvPoint> &locations) const;
	virtual void discardUninteresting(const vector<IplImage*> images, double scale, vector<CvPoint> &locations) const;
};

// svlRotatedWindowDiscarder ----------------------------------------------------

class svlRotatedWindowDiscarder : public svlWindowDiscarder {
protected:
	double _ang;
	CvSize _sz;
public:
	svlRotatedWindowDiscarder(int w = 32, int h = 32, double ang = 0.0, CvSize sz = cvSize(0,0))
		: svlWindowDiscarder(w,h), _ang(ang), _sz(sz) {}
	virtual ~svlRotatedWindowDiscarder() {}
	
	// Accessors.
	inline void setSize(CvSize sz) {
		_sz.width = sz.width;
		_sz.height = sz.height;
	}
	inline void setAngle(double ang) { _ang = ang; }
	inline double getAngle() { return _ang; }

	// Discard locations that would contain null pixels in a rotated, trimmed image.
	virtual void discardUninteresting(IplImage *image, double scale, vector<CvPoint> &locations) const;
	virtual void discardUninteresting(const vector<IplImage*> images, double scale, vector<CvPoint> &locations) const;
};


