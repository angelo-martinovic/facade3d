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
** FILENAME:    svlDepthSegImage.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
**
*****************************************************************************/

#include <cstdlib>
#include <string>
#include <vector>
#include <set>
#include <map>

#include "cv.h"
#include "cxcore.h"

#include "svlBase.h"
#include "svlSegImage.h"
#include "svlDepthSegImage.h"
#include "svlVisionUtils.h"

using namespace std;

// Constants and Globals ------------------------------------------------------

// camera intrinsics
float svlDepthSegImage::ALPHA = 394.493f;
float svlDepthSegImage::BETA = 394.327f;
CvPoint2D32f svlDepthSegImage::CC = cvPoint2D32f(319.619, 247.207);

// depth filtering parameters
float svlDepthSegImage::MIN_DEPTH = 0.0f;
float svlDepthSegImage::MAX_DEPTH = 10.0f;

// svlDepthSegImage ----------------------------------------------------------

svlDepthSegImage::svlDepthSegImage(const IplImage *image, const CvMat *seg, 
    const CvMat *depths) : 
    svlSegImageBase(image, seg),  _pixelDepths(NULL)
{
    SVL_ASSERT((depths != NULL) &&
        (depths->rows == _segmentation->rows) &&
        (depths->cols == _segmentation->cols) && 
        (CV_MAT_TYPE(depths->type) == CV_32FC1));

    _pixelDepths = cvCloneMat(depths);

    computeSegmentNormals();    
}
    
svlDepthSegImage::svlDepthSegImage(const char *imgFilename, const char *segFilename,
    const char *depthFilename) :
    svlSegImageBase(imgFilename, segFilename), _pixelDepths(NULL)
{
    SVL_ASSERT(depthFilename != NULL);
    SVL_LOG(SVL_LOG_DEBUG, "svlDepthSegImage::svlDepthSegImage() reading depthmap...");

    _pixelDepths = cvCreateMat(_image->height, _image->width, CV_32FC1);
    if (!readMatrix(_pixelDepths, depthFilename)) {
        cvZero(_pixelDepths);
        SVL_LOG(SVL_LOG_FATAL, "failed to read depth file " << depthFilename);
    }

    computeSegmentNormals();
}

svlDepthSegImage::svlDepthSegImage(const svlDepthSegImage& segImg) :
    svlSegImageBase(segImg), _pixelDepths(NULL)
{
    _pixelDepths = cvCloneMat(segImg._pixelDepths);
    _segNormal = segImg._segNormal;
}

svlDepthSegImage::~svlDepthSegImage()
{
    if (_pixelDepths != NULL)
	cvReleaseMat(&_pixelDepths);
}


// functions for getting and setting depth
void svlDepthSegImage::setDepth(const CvMat *depths)
{
    SVL_ASSERT((depths != NULL) &&
        (depths->rows == _segmentation->rows) &&
        (depths->cols == _segmentation->cols) && 
        (CV_MAT_TYPE(depths->type) == CV_32FC1));

    cvCopy(depths, _pixelDepths);
    computeSegmentNormals();
}

void svlDepthSegImage::setSegDepths(const vector<double>& depths)
{
    SVL_ASSERT(depths.size() == (unsigned)_numSegments);

    // compute depth of all pixels based on superpixel depth and normal
    for (int i = 0; i < _numSegments; i++) {
        svlPoint3d r_i = pixelRay(_segCentroid[i].x, _segCentroid[i].y);
        double num = _segNormal[i].dot(r_i) * depths[i];
        for (vector<CvPoint>::const_iterator p = _segPixels[i].begin();
             p != _segPixels[i].end(); ++p) {
            double d = num / _segNormal[i].dot(pixelRay(p->x, p->y));
            if (d < MIN_DEPTH) d = MIN_DEPTH;
            if (d > MAX_DEPTH) d = MAX_DEPTH;
            CV_MAT_ELEM(*_pixelDepths, float, p->y, p->x) = (float)d;
        }
    }    
}

void svlDepthSegImage::setSegNormals(const vector<svlPoint3d>& normals)
{
    vector<double> depths(_numSegments);
    for (int i = 0; i < _numSegments; i++) {
        depths[i] = (double)CV_MAT_ELEM(*_pixelDepths, float,
            _segCentroid[i].y, _segCentroid[i].x);
    }
    setSegDepthsAndNormals(depths, normals);
}

void svlDepthSegImage::setSegDepthsAndNormals(const vector<double>& depths,
    const vector<svlPoint3d>& normals)
{
    SVL_ASSERT(depths.size() == (unsigned)_numSegments);
    SVL_ASSERT(normals.size() == (unsigned)_numSegments);

    _segNormal = normals;
    setSegDepths(depths);
}

// private member functions

void svlDepthSegImage::computeSegmentNormals()
{
    SVL_ASSERT(_pixelDepths != NULL);

    // first truncate depthmap to MIN_DEPTH..MAX_DEPTH
    CvMat *bound = cvCreateMat(height(), width(), CV_32FC1);
    cvSet(bound, cvScalar(MIN_DEPTH));
    cvMax(_pixelDepths, bound, _pixelDepths);
    cvSet(bound, cvScalar(MAX_DEPTH));
    cvMin(_pixelDepths, bound, _pixelDepths);
    cvReleaseMat(&bound);

    // estimate normal vectors
    _segNormal.resize(_numSegments);

    CvMat *X = cvCreateMat(height(), width(), CV_32FC1);
    CvMat *Y = cvCreateMat(height(), width(), CV_32FC1);

    for (int y = 0; y < height(); y++) {
        for (int x = 0; x < width(); x++) {
            svlPoint3d r = pixelRay(x, y, (double)getDepth(x, y));
            CV_MAT_ELEM(*X, float, y, x) = (float)r.x;
            CV_MAT_ELEM(*Y, float, y, x) = (float)r.y;
        }
    }

#if 1
    svlPoint3d n;
    svlPoint3d c;
    for (int i = 0; i < _numSegments; i++) {        
        estimatePlane(X, Y, _pixelDepths, _segPixels[i], _segNormal[i], c);        
        // normal must point towards camera
        if (_segNormal[i].z > 0) {
            //_segNormal[i] = svlPoint3d(0.0, 0.0, -1.0);
            _segNormal[i] = -1.0 * _segNormal[i];
        }
        // avoid very sharply sloped planes
        if (fabs(_segNormal[i].z) < 1.0e-1) {
            _segNormal[i] = svlPoint3d(0.0, 0.0, -1.0);
        }
    }
#else
    CvMat *nX = cvCreateMat(height(), width(), CV_32FC1);
    CvMat *nY = cvCreateMat(height(), width(), CV_32FC1);
    CvMat *nZ = cvCreateMat(height(), width(), CV_32FC1);
    
    estimatePointNormals(X, Y, _pixelDepths, nX, nY, nZ, 5);

    for (int i = 0; i < _numSegments; i++) {        
        _segNormal[i].x = CV_MAT_ELEM(*nX, float,
            _segCentroid[i].y, _segCentroid[i].x);
        _segNormal[i].y = CV_MAT_ELEM(*nY, float,
            _segCentroid[i].y, _segCentroid[i].x);
        _segNormal[i].z = CV_MAT_ELEM(*nZ, float,
            _segCentroid[i].y, _segCentroid[i].x);
    }

    cvReleaseMat(&nZ);
    cvReleaseMat(&nY);
    cvReleaseMat(&nX);
#endif

    // free memory
    cvReleaseMat(&Y);
    cvReleaseMat(&X);
}



