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
** FILENAME:    svlDepthSegImage.h
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
** DESCRIPTION:
**  Holds an image corresponding depthmap and over segmentation (superpixel 
**  description). The image and depthmap must be the same size.
**
**  Images are assumed to be 8-bit colour (3-channel). Depthmaps are assumed
**  to be 32-bit floats. Segmentation is assumed to be 32-bit integers.
*****************************************************************************/

#pragma once

#include <string>
#include <vector>
#include <set>

#include "cv.h"
#include "cxcore.h"

#include "svlBase.h"
#include "svlPoint3d.h"
#include "svlSegImage.h"

using namespace std;

// svlDepthSegImage ----------------------------------------------------------

class svlDepthSegImage : public svlSegImageBase {
 public:
    // camera intrinsics
    static float ALPHA;      // pixel width x focal length
    static float BETA;       // pixel height x focal length
    static CvPoint2D32f CC;  // principal point

    // depth filtering parameters
    static float MIN_DEPTH;
    static float MAX_DEPTH;

 protected:
    CvMat *_pixelDepths;
    vector<svlPoint3d> _segNormal;

 public:
    svlDepthSegImage(const IplImage *image, const CvMat *seg, 
	const CvMat *depths);
    svlDepthSegImage(const char *imgFilename, const char *segFilename, 
	const char *depthFilename);
    svlDepthSegImage(const svlDepthSegImage& segImg);
    virtual ~svlDepthSegImage();

    // accessor for the image
    inline CvMat *depthMap() const { return _pixelDepths; }

    // functions for getting and setting depth
    void setDepth(const CvMat *depths);
    void setSegDepths(const vector<double>& depths);
    void setSegNormals(const vector<svlPoint3d>& normals);
    void setSegDepthsAndNormals(const vector<double>& depths,
        const vector<svlPoint3d>& normals);

    inline float getDepth(int x, int y) const {
	SVL_ASSERT((x >= 0) && (x < width()) && (y >= 0) && (y < height()));
	return CV_MAT_ELEM(*_pixelDepths, float, y, x);
    }
    inline float getDepth(int segment) const {
	SVL_ASSERT((segment >= 0) && (segment < _numSegments));	
	return CV_MAT_ELEM(*_pixelDepths, float, 
            _segCentroid[segment].y, _segCentroid[segment].x);
    }
    inline const svlPoint3d& getNormal(int segment) const {
	SVL_ASSERT((segment >= 0) && (segment < _numSegments));	
        return _segNormal[segment];
    }

    // helper functions
    inline static svlPoint3d pixelRay(int x, int y, double z = 1.0) {
        return svlPoint3d(z * (x - CC.x) / ALPHA, z * (y - CC.y) / BETA, z);
    }

 protected:
    void computeSegmentNormals();
};


