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
** FILENAME:    svlRotatableIpl.h
** AUTHOR(S):   Paul Baumstarck <pbaumstarck@stanford.edu>
** DESCRIPTION:
**  An Ipl that can be rotated through all deformations without pixel loss
**  using the OpenCV rotation commands.
**
*****************************************************************************/

#pragma once

#include <cassert>
#include <iostream>
#include <vector>
#include <map>
#include <string>

#include "cv.h"
#include "cxcore.h"
#include "ml.h"

using namespace std;

// svlRotatableIpl ---------------------------------------------------

class svlRotatableIpl {
protected:
	// The padded, rotated image.
	IplImage *_img;
	CvMat *_translate;
	int _orig_w, _orig_h,	// Width and height of original image.
			_cent;						// Center of padded image.

public:
	svlRotatableIpl();
	svlRotatableIpl(const IplImage *img);
	svlRotatableIpl(const svlRotatableIpl *c);
	virtual ~svlRotatableIpl();

	// Accessors.
	IplImage* getImage() { return _img; }
	inline int getOrigWidth() { return _orig_w; }
	inline int getOrigHeight() { return _orig_h; }
	inline int getCent() { return _cent; }
	inline int getWidth() { return _img->width; }
	inline int getHeight() { return _img->height; }

	// Get a rotated image.
	IplImage* rotate(float ang);
	// Rotate image (pseudo-) in-place.
	IplImage* rotateInPlace(float ang);
	// Return an image with no zero rows or columns.
	IplImage* getTrimmed();
	// Rotate and return the trimmed image.
	IplImage* rotateGetTrimmed(float ang);
	// Rotate in place and return the trimmed image.
	IplImage* rotateInPlaceGetTrimmed(float ang);
	
	// Static method to trim an image of zero rows and columns.
	static IplImage* trimImage(const IplImage *img);
	// Return the bounds to trim an image < [row1,row2), [col1,col2) >.
	static pair<pair<int,int>,pair<int,int> > trimImageBounds(const IplImage *img);
	// Trims an image using row-column bounds.
	static IplImage* trimImage(const IplImage *img, pair<pair<int,int>,pair<int,int> > &bounds);
	// Same, only in tandem so that all images have the same size.
	static void tandemTrimImage(const vector<IplImage*> &imgs, vector<IplImage*> &rets);
	// Rotate and trim several RIpls in tandem (so that they all have the same size at the end).
	static void tandemRotateGetTrimmed(const vector<svlRotatableIpl*> &rimgs, float ang, vector<IplImage*> &imgs);
};


