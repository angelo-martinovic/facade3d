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
** FILENAME:    svlSoftEdgeMap.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
**              Olga Russakovsky <olga@cs.stanford.edu>
**
*****************************************************************************/

#include <cstdio>
#include <cassert>
#include <limits>

#include "../base/svlCompatibility.h"
#include "svlVisionUtils.h"
#include "svlSoftEdgeMap.h"

using namespace std;

svlSoftEdgeMap::svlSoftEdgeMap() :
  _imageBuffer(NULL), _hEdgeBuffer(NULL), _vEdgeBuffer(NULL), _resultBuffer(NULL)
{
    // do nothing
}

svlSoftEdgeMap::svlSoftEdgeMap(const svlSoftEdgeMap& m)  :
  _imageBuffer(NULL), _hEdgeBuffer(NULL), _vEdgeBuffer(NULL), _resultBuffer(NULL)
{
    if (m._imageBuffer != NULL) {
        _imageBuffer = cvCloneImage(m._imageBuffer);
    }
    
    if (m._hEdgeBuffer != NULL) {
        _hEdgeBuffer = cvCloneImage(m._hEdgeBuffer);
    }

    if (m._vEdgeBuffer != NULL) {
        _vEdgeBuffer = cvCloneImage(m._vEdgeBuffer);
    }

    if (m._resultBuffer != NULL) {
        _resultBuffer = cvCloneImage(m._resultBuffer);
    }
}

svlSoftEdgeMap::~svlSoftEdgeMap()
{
    if (_imageBuffer != NULL) {
        cvReleaseImage(&_imageBuffer);
        cvReleaseImage(&_hEdgeBuffer);
        cvReleaseImage(&_vEdgeBuffer);
	cvReleaseImage(&_resultBuffer);
    }
}

const IplImage *svlSoftEdgeMap::processImage(const unsigned char *data, int w, int h, bool bNormalize)
{
    allocateMemory(w, h);

    // copy data into image
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            _imageBuffer->imageData[y * _imageBuffer->widthStep + x] = 
                (unsigned char)(0.114 * data[3 * (y * _imageBuffer->width + x) + 0] +
                0.587 * data[3 * (y * _imageBuffer->width + x) + 1] +
                0.299 * data[3 * (y * _imageBuffer->width + x) + 2]);
        }
    }

    cvZero(_resultBuffer);
    processImageBuffer(1);

    if (bNormalize) {
	scaleToRange(_resultBuffer, 0.0, 255.0);
    }

    return _resultBuffer;
}

const IplImage *svlSoftEdgeMap::processImage(const IplImage *image, bool bNormalize)
{
  SVL_ASSERT(image->depth == IPL_DEPTH_8U);
  SVL_ASSERT(image != NULL);
  allocateMemory(image->width, image->height);

     //if (image->nChannels == 3) {
    //cvCvtColor(image, _imageBuffer, CV_BGR2GRAY);
    //} else {
    //  cvCopy(image, _imageBuffer);
    //}

    cvZero(_resultBuffer);

    if (image->nChannels == 1) {
      cvCopy(image, _imageBuffer);
      processImageBuffer(image->nChannels);
    } else {
      IplImage *buffers[4];
      for (int i = 0; i < 4; i++)
	buffers[i] = NULL;
      
      for (int ch = 0; ch < image->nChannels; ch++) {
	// extract just one channel at a time, without altering the image
	buffers[ch] = _imageBuffer;
	cvSplit(image, buffers[0], buffers[1], buffers[2], buffers[3]);
	buffers[ch] = NULL;
	
	processImageBuffer(image->nChannels);
      }
    }
    
    if (bNormalize) {
      scaleToRange(_resultBuffer, 0.0, 255.0);
    }

    return _resultBuffer;
}

void svlSoftEdgeMap::allocateMemory(int w, int h)
{
    if (_imageBuffer == NULL) {
      _imageBuffer = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 1);
      _hEdgeBuffer = cvCreateImage(cvSize(w, h), IPL_DEPTH_16S, 1);
      _vEdgeBuffer = cvCreateImage(cvSize(w, h), IPL_DEPTH_16S, 1);
      _resultBuffer = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 1);
    } else if ((_imageBuffer->width != w) ||
	       (_imageBuffer->height != h)) {
      cvReleaseImage(&_imageBuffer);
      cvReleaseImage(&_hEdgeBuffer);
      cvReleaseImage(&_vEdgeBuffer);
      cvReleaseImage(&_resultBuffer);
      _imageBuffer = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 1);
      _hEdgeBuffer = cvCreateImage(cvSize(w, h), IPL_DEPTH_16S, 1);
      _vEdgeBuffer = cvCreateImage(cvSize(w, h), IPL_DEPTH_16S, 1);
      _resultBuffer = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 1);
    }
}

void svlSoftEdgeMap::processImageBuffer(int numChannelsInOrigImage) {
  // horizontal and vertical edge filters
  cvSobel(_imageBuffer, _hEdgeBuffer, 1, 0, 3);
  cvSobel(_imageBuffer, _vEdgeBuffer, 0, 1, 3);
  
  // to bring it down to [0, 255] scale
  cvScale(_hEdgeBuffer, _hEdgeBuffer, 0.25);
  cvScale(_vEdgeBuffer, _vEdgeBuffer, 0.25);

  // combine and rescale
  for (int y = 0; y < _imageBuffer->height; y++) {
    const short *pGx = (const short *)&_hEdgeBuffer->imageData[y * _hEdgeBuffer->widthStep];
    const short *pGy = (const short *)&_vEdgeBuffer->imageData[y * _vEdgeBuffer->widthStep];
    unsigned char *p = (unsigned char *)&_resultBuffer->imageData[y * _resultBuffer->widthStep];
    for (int x = 0; x < _imageBuffer->width; x++) {
      //p[x] = MIN(255, (abs(pGx[x]) + abs(pGy[x])) / 2);
      //p[x] += (uchar)MIN(255, (float)sqrt((double)(pGx[x] * pGx[x]) + (pGy[x] * pGy[x])) * M_SQRT1_2) / numChannelsInOrigImage;
      p[x] = MAX(p[x], (uchar)MIN(255, (float)sqrt((double)(pGx[x] * pGx[x]) + (pGy[x] * pGy[x])) * M_SQRT1_2));
      //p[x] += abs(pGy[x]) / image->nChannels;
    }
  }
}

