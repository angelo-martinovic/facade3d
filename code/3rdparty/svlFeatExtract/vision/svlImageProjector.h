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
** FILENAME:    svlImageProjector.h
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
** DESCRIPTION:
**   Class for overlaying back projection of 3d data into images. Uses OpenCV
**   functions and classes and 3d Vision Project's intrinsic and extrinsic
**   classes.
**
*****************************************************************************/

#pragma once

#include <cassert>
#include <vector>

#include "cv.h"
#include "cxcore.h"

#include "svlBase.h"
#include "svlVision.h"

using namespace std;

class svlImageProjector {
 protected:
    static const int _MAX_CHANNELS;
    static const int _OCCLUSION_WIDTH;

    IplImage *_image;
    IplImage *_projectedImage;
    CvMat *_projectedData[3];
    unsigned char *_imageData;
    bool _bImageDataDirty;
    int _width, _height;
    svlCameraIntrinsics _intrinsics;
    svlCameraExtrinsics _extrinsics;
    double _smoothing;
    double _alpha;
    int _pointWidth;

 public:
    svlImageProjector();
    svlImageProjector(const svlCameraIntrinsics& i, const svlCameraExtrinsics& e);
    //svlImageProjector(const svlImageProjector& b);
    virtual ~svlImageProjector();

    void setCameraParameters(svlCameraIntrinsics& i, svlCameraExtrinsics& e);
    void clearImage(int width, int height);
    void setImage(const IplImage *image);
    void setImage(const unsigned char *data, int width, int height);
    inline void clearSmoothing() { _smoothing = 0.0; }
    inline void setSmoothing(double s) { _smoothing = s; }
    inline void setAlpha(double a) { assert((_alpha >= 0.0) && (_alpha <= 1.0)); _alpha = a; }
    inline void setPointWidth(int w) { assert(w >= 0); _pointWidth = w; }

    inline int getImageWidth() const { return _width; };
    inline int getImageHeight() const { return _height; };
    const IplImage *getAsImage(double scale = 1.0, double shift = 0.0);
    const unsigned char *getAsImageData(double scale = 1.0, double shift = 0.0);
    const CvMat *getData(int channel) const { 
	assert((channel >= 0) && (channel < _MAX_CHANNELS));
	return _projectedData[channel];
    };
    
    void project(const vector<svlPoint3d>& points, const svlPoint3d& offset = svlPoint3d(),
	double pan = 0.0, double tilt = 0.0, double roll = 0.0);
    void project(const vector<svlPoint3d>& points, const vector<svlPoint3d>& colours,
	const svlPoint3d& offset = svlPoint3d(), double pan = 0.0, double tilt = 0.0, double roll = 0.0);
    void project(const vector<svlPoint3d>& points, const vector<double>& data,
	const svlPoint3d& offset = svlPoint3d(), double pan = 0.0, double tilt = 0.0, double roll = 0.0,
	int channel = -1);
    
    void getRange(int channel, double *minVal, double *maxVal) const;
    void getRange(int channel, const CvRect& region, double *minVal, double *maxVal) const;
    void scaleToRange(int channel, double minVal, double maxVal);
    vector<double> featureStatistics(int channel, const CvRect& region) const;

 protected:
    void freeMemory();
};


