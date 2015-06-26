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
** FILENAME:    svlCacheOutputUtils.h
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
**              Olga Russakovsky <olga@cs.stanford.edu>
**              Ian Goodfellow <ia3n@cs.stanford.edu>
** DESCRIPTION:
**  Utility functions for reading and writing training examples from cache
**
*****************************************************************************/

#pragma once

#include <vector>
#include <iostream>
#include <sstream>

#include "cv.h"
#include "cxcore.h"
#include "highgui.h"

#include "svlBase.h"
#include "svlPoint3d.h"
#include "svlSlidingWindowDetector.h"

vector<vector<double> > svlReadExamplesFromCache(const char* dirName, long int maxImages, bool binary);

void svlWriteExampleToCache(const char *dirName, unsigned sample_idx, vector<double> &v);
void svlWriteExampleToBinaryCache(std::ostream &ofs, vector<double> &v);

struct SWcacheInfo {
  int img_w;
  int img_h;
  int w;
  int h;
  int delta_x;
  int delta_y;
  float delta_scale;
  bool center;
  SWcacheInfo(int iw, int ih, int ww, int wh, 
	      int dx, int dy, float ds, bool c) :
    img_w(iw), img_h(ih), w(ww), h(wh), 
    delta_x(dx), delta_y(dy), delta_scale(ds), center(c) {}
  SWcacheInfo(const svlSlidingWindowDetector *d, const IplImage *img) :
    img_w(img->width), img_h(img->height), w(d->width()), h(d->height()),
    delta_x(d->getDeltaX()), delta_y(d->getDeltaY()), 
    delta_scale(d->getDeltaScale()), center(false) {}
  SWcacheInfo(string dir, string baseName);
  bool write(string dir, string baseName);
};

