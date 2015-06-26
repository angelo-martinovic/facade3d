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
** FILENAME:    svlWindowDiscarder.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
**              Sid Batra <sidbatra@stanford.edu>
**              Olga Russakovsky <olga@cs.stanford.edu>
**              Ian Goodfellow <ia3n@cs.stanford.edu>
**              Paul Baumstarck <pbaumstarck@stanford.edu>
**
*****************************************************************************/

#include "svlVision.h"

// svlWindowDiscarder ----------------------------------------------------------------

svlWindowDiscarder::svlWindowDiscarder(int w, int h)
	: _windowWidth(w), _windowHeight(h)
{
}

svlWindowDiscarder::~svlWindowDiscarder()
{
}

void svlWindowDiscarder::discardInvalid(int imageWidth, int imageHeight, vector<CvPoint> &locations) const
{
 for (int i = (int)locations.size() - 1; i >= 0; i--) 
    {
      if (locations[i].x < 0 || locations[i].y < 0 ||
	  locations[i].x + _windowWidth > imageWidth || locations[i].y + _windowHeight > imageHeight) {
	locations.erase(locations.begin() + i);
      }
    }
}

void svlWindowDiscarder::discardUninteresting(IplImage *image, double scale, vector<CvPoint> &locations) const
{
  if (image == NULL) return;
  for (int i = (int)locations.size() - 1; i >= 0; i--)  {
      cvSetImageROI(image, cvRect(locations[i].x, locations[i].y, _windowWidth, _windowHeight));

      if (svlImageAlmostUniform(image)) {
		locations.erase(locations.begin() + i);
      }
      cvResetImageROI(image);
    }
}

void svlWindowDiscarder::discardUninteresting(const vector<IplImage *> images, double scale, vector<CvPoint> &locations) const
{
  if (images.size() == 0) return;
  discardUninteresting(images[0], scale, locations);
}


// svlRotatedWindowDiscarder ----------------------------------------------------------------

// Discard locations that would contain non-existent pixels in this rotated image.
void svlRotatedWindowDiscarder::discardUninteresting(IplImage *image, double scale, vector<CvPoint> &locations) const
{
	if ( !image )
		return;

	//printf("ang: %0.1f; sz: %d,%d; in: %d,%d; scale: %0.3f\n", _ang, _sz.width, _sz.height, image->width, image->height, scale);
	// Project all points into rectified dimension for easy bounds check (within w/2 and h/2 of the origin).
	scale = 1.0/scale;
	double orig_x = 0.5*double(image->width), orig_y = 0.5*double(image->height),
		   orig_w = scale*0.5*double(_sz.width), orig_h = scale*0.5*double(_sz.height);
	double x, y, a = _ang * 0.017453292519943;
	double wx = cos(a), wy = -sin(a), hx = -sin(a), hy = -cos(a);
	for ( vector<CvPoint>::iterator it=locations.begin(); it!=locations.end(); )
		if ( fabs( (x=double(it->x)-orig_x)*wx + (y=double(it->y)-orig_y)*wy) > orig_w || fabs(x*hx + y*hy) > orig_h
			|| fabs( (x+=double(_windowWidth))*wx + y*wy) > orig_w || fabs(x*hx + y*hy) > orig_h
			|| fabs( x*wx + (y+=double(_windowHeight))*wy) > orig_w || fabs(x*hx + y*hy) > orig_h
			|| fabs( (x-=double(_windowWidth))*wx + y*wy) > orig_w || fabs(x*hx + y*hy) > orig_h )
			it = locations.erase(it);
		else
			++it;
}

void svlRotatedWindowDiscarder::discardUninteresting(const vector<IplImage *> images, double scale, vector<CvPoint> &locations) const
{
	if (images.size() == 0) return;
	discardUninteresting(images[0], scale, locations);
}


