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
** FILENAME:    svlConvolution.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
**              Olga Russakovsky <olga@cs.stanford.edu>
**
*****************************************************************************/

#include <cstdlib>
#include <cassert>
#include <iostream>
#include <fstream>

#include "cv.h"
#include "cxcore.h"

#include "svlBase.h"
#include "svlConvolution.h"
#include "svlVisionUtils.h"

// svlConvolution --------------------------------------------------------------

bool svlConvolution::bUseDFTMethod = false;

svlConvolution::svlConvolution(const CvMat *k, const CvPoint a) : _kernel(NULL), _anchor(a)
{
    if (k != NULL) {
        _kernel = cvCloneMat(k);
    }
}

svlConvolution::svlConvolution(const svlConvolution& c) : _kernel(NULL), _anchor(c._anchor)
{
    if (c._kernel != NULL) {
        _kernel = cvCloneMat(c._kernel);
    }
}

svlConvolution::~svlConvolution()
{
    if (_kernel != NULL) {
        cvReleaseMat(&_kernel);
    }
}

bool svlConvolution::readKernel(const char *filename)
{
    SVL_NOT_IMPLEMENTED;
    return false;
}

bool svlConvolution::writeKernel(const char *filename) const
{
    SVL_NOT_IMPLEMENTED;
    return false;
}

void svlConvolution::filter(IplImage *src, IplImage *dst, bool padWithZeros)
{
    SVL_ASSERT((_kernel != NULL) && (src != NULL) && (dst != NULL));
    SVL_ASSERT((src->width == dst->width) && (src->height == dst->height));
    SVL_ASSERT((dst->nChannels == 1) && (dst->depth == IPL_DEPTH_32F));

    // convert source image to single channel 32F
    IplImage *srcCopy = NULL;
    if (src->nChannels == 3) {
        srcCopy = cvCreateImage(cvSize(src->width, src->height), src->depth, 1);
        cvCvtColor(src, srcCopy, CV_RGB2GRAY);
        if (src->depth != IPL_DEPTH_32F) {
            IplImage *tmp = cvCreateImage(cvSize(src->width, src->height), IPL_DEPTH_32F, 1);
            cvConvertScale(srcCopy, tmp, 1.0/255.0);
            cvReleaseImage(&srcCopy);
            srcCopy = tmp;
        }
        src = srcCopy;
    } else if (src->depth != IPL_DEPTH_32F) {
        srcCopy = cvCreateImage(cvSize(src->width, src->height), IPL_DEPTH_32F, 1);
        cvConvertScale(src, srcCopy, 1.0/255.0);
        src = srcCopy;
    }

    // do filtering
    if (bUseDFTMethod) {
        // O(n log(m))
        SVL_ASSERT(false);
    } else {
        // O(nm)
	// OpenCV workaround
	//if ((src->width % 8 != 0) || (src->height % 8 != 0)) {
	if (src->height % 8 != 0) {
	  if (src->height < 8 || padWithZeros) {
	    IplImage *tmp = cvCreateImage(cvSize(src->width + 8 - (src->width % 8),
		    src->height + 8 - (src->height % 8)), IPL_DEPTH_32F, 1);
	    cvZero(tmp);
	    cvSetImageROI(tmp, cvRect(0, 0, src->width, src->height));
	    cvCopy(src, tmp);
	    cvResetImageROI(tmp);
	    cvFilter2D(tmp, tmp, _kernel, _anchor);
	    cvSetImageROI(tmp, cvRect(0, 0, src->width, src->height));
	    cvCopy(tmp, dst);
	    cvReleaseImage(&tmp);
	  } else {
	    //cerr << "WARNING (svlConvolution::filter) OpenCV workaround" << endl;
	    IplImage *tmp = cvCreateImage(cvSize(src->width - (src->width % 8),
		    src->height - (src->height % 8)), IPL_DEPTH_32F, 1);
	    cvResize(src, tmp);
	    cvFilter2D(tmp, tmp, _kernel, _anchor);
	    cvResize(tmp, dst);
	    cvReleaseImage(&tmp);
	  }
  	} else {
	  cvFilter2D(src, dst, _kernel, _anchor);
	}
    }

    // free memory
    if (srcCopy != NULL) {
        cvReleaseImage(&srcCopy);
    }
}

void svlConvolution::normalize(float mu, float sigma)
{
    SVL_ASSERT(_kernel != NULL);

    CvScalar mean = cvAvg(_kernel);
    cvSubS(_kernel, mean, _kernel);

    cvScale(_kernel, _kernel, sigma / cvNorm(_kernel, NULL, CV_L2), mu);
}

IplImage *svlConvolution::visualize() const
{
    SVL_ASSERT(_kernel != NULL);

    IplImage *img = cvCreateImage(cvSize(_kernel->cols, _kernel->rows), IPL_DEPTH_8U, 1);
    
    double minVal, maxVal;
    cvMinMaxLoc(_kernel, &minVal, &maxVal);
    if (maxVal == minVal) {
        cvZero(img);
    } else {
        cvConvertScale(_kernel, img, 255.0 / (maxVal - minVal),
            -255.0 * minVal / (maxVal - minVal)); 
    }

    return img;
}

// svlGaborConvolution ---------------------------------------------------------

svlGaborConvolution::svlGaborConvolution(int w, int h, float theta, float gamma,
    float sigma, float lambda) : svlConvolution(NULL)
{
    _kernel = cvCreateMat(w, h, CV_32FC1);
    SVL_ASSERT(_kernel != NULL);

    // kernel center
    int cx = _kernel->cols / 2;
    int cy = _kernel->rows / 2;
	
    float gamma2 = gamma * gamma;
    float sigma2 = (float)2.0 * sigma * sigma;
    float invLambda = (float)(2.0 * M_PI / lambda);
    float sinTheta = sin(theta);
    float cosTheta = cos(theta);

    for (int y = 0; y < _kernel->rows; y++)  {
        float v = (float)y - cy;
        for (int x = 0; x < _kernel->cols; x++) {
            float u = (float)x - cx;
            float X = u * cosTheta + v * sinTheta;
            float Y = -u * sinTheta + v * cosTheta;
         
	    CV_MAT_ELEM(*_kernel, float, y, x) =              
		(float)exp(-(X * X + Y * Y * gamma2) / sigma2) *
                (float)cos(invLambda * X);
        }
    }
}

svlGaborConvolution::~svlGaborConvolution()
{
    // do nothing
}

// svlLoGConvolution ------------------------------------------------------------

svlLoGConvolution::svlLoGConvolution(int w, int h, float sigma)
{
    _kernel = cvCreateMat(w, h, CV_32FC1);
    SVL_ASSERT(_kernel != NULL);

    // kernel center
    int cx = _kernel->cols / 2;
    int cy = _kernel->rows / 2;
	
    float sigma2 = 2.0f * sigma * sigma;
    float invSigma4 = 1.0f / (float)(M_PI * sigma * sigma * sigma * sigma);

    for (int y = 0; y < _kernel->rows; y++)  {
        float v2 = (float)(y - cy) * (y - cy);
        for (int x = 0; x < _kernel->cols; x++) {
            float u2 = (float)(x - cx) * (x - cx);
	    float r2 = (v2 + u2) / sigma2;	    
	    CV_MAT_ELEM(*_kernel, float, y, x) =
		(float)(invSigma4 * (r2 - 1.0) * exp(-r2));
        }
    }    
}

svlLoGConvolution::~svlLoGConvolution()
{
    // do nothing
}

	

