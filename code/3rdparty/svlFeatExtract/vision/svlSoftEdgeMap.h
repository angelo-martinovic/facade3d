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
** FILENAME:    svlSoftEdgeMap.h
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
** DESCRIPTION:
**   OpenCV code for creating a soft edge map.
**
*****************************************************************************/

#pragma once

#include "cv.h"
#include "cxcore.h"

class svlSoftEdgeMap {
protected:
    IplImage *_imageBuffer;
    //IplImage *_resultBuffer32F;
    IplImage *_hEdgeBuffer;
    IplImage *_vEdgeBuffer;
    IplImage *_resultBuffer;

public:
    svlSoftEdgeMap();
    svlSoftEdgeMap(const svlSoftEdgeMap& m);
    ~svlSoftEdgeMap();

    inline int getWidth() const { 
        return (_imageBuffer == NULL ? 0 : _imageBuffer->width);
    }
    inline int getHeight() const {
        return (_imageBuffer == NULL ? 0 : _imageBuffer->height);
    }

    const IplImage *processImage(const unsigned char *data, int w, int h,
	bool bNormalize = false);
    const IplImage *processImage(const IplImage *image,
	bool bNormalize = false);
    inline const IplImage *getImage() { return _imageBuffer; }
    inline const IplImage *getEdgeMap() { return _resultBuffer; }

 protected:
    void allocateMemory(int w, int h);

    void processImageBuffer(int numChannelsInOrigImage);
};

