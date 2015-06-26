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
** FILENAME:    svlTextonFilterBank.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
**
*****************************************************************************/

#include <cstdlib>
#include <vector>

#include "cv.h"
#include "cxcore.h"

#include "svlBase.h"
#include "svlTextonFilterBank.h"
#include "svlVisionUtils.h"

// svlTextonFilterBank ------------------------------------------------------

svlTextonFilterBank::svlTextonFilterBank(double k) :
    _kappa(k)
{
    // do nothing
}


svlTextonFilterBank::~svlTextonFilterBank()
{
    // do nothing
}

void svlTextonFilterBank::filter(IplImage *img, vector<IplImage *> &response)
{
    // check input
    SVL_ASSERT(img != NULL);
    if (response.empty()) {
        response.resize(NUM_FILTERS, NULL);
    }
    SVL_ASSERT((int)response.size() == NUM_FILTERS);
    SVL_ASSERT((img->nChannels == 3) && (img->depth == IPL_DEPTH_8U));
    for (int i = 0; i < NUM_FILTERS; i++) {
        if (response[i] == NULL) {
            response[i] = cvCreateImage(cvGetSize(img), IPL_DEPTH_32F, 1);
        } else if (response[i]->width != img->width ||
		   response[i]->height != img->height) {
	  cvReleaseImage(&response[i]);
	  response[i] = cvCreateImage(cvGetSize(img), IPL_DEPTH_32F, 1);
	}
        SVL_ASSERT((response[i] != NULL) && (response[i]->nChannels == 1) &&
            (response[i]->depth == IPL_DEPTH_32F));
    }

    int k = 0;

    // color convert
    SVL_LOG(SVL_LOG_DEBUG, "Color converting image...");
    IplImage *imgCIELab = cvCreateImage(cvGetSize(img), IPL_DEPTH_32F, 3);
    cvConvertScale(img, imgCIELab, 1.0 / 255.0, 0.0);
    cvCvtColor(imgCIELab, imgCIELab, CV_BGR2Lab);

    IplImage *greyImg = cvCreateImage(cvGetSize(img), IPL_DEPTH_32F, 1);
    cvSetImageCOI(imgCIELab, 1);
    cvCopyImage(imgCIELab, greyImg);
    cvSetImageCOI(imgCIELab, 0);
        
    // gaussian filter on all color channels
    SVL_LOG(SVL_LOG_DEBUG, "Generating gaussian filter responses...");
    IplImage *gImg32f = cvCreateImage(cvGetSize(img), IPL_DEPTH_32F, 3);
    for (double sigma = 1.0; sigma <= 4.0; sigma *= 2.0) {
        cvSmooth(imgCIELab, gImg32f, CV_GAUSSIAN, 2 * (int)(_kappa * sigma) + 1);
        for (int c = 1; c <= 3; c++) {
            cvSetImageCOI(gImg32f, c);
            cvCopyImage(gImg32f, response[k++]);
        }
        cvSetImageCOI(gImg32f, 0);
    }
    cvReleaseImage(&gImg32f);

    // derivatives of gaussians on just greyscale image
    SVL_LOG(SVL_LOG_DEBUG, "Generating derivative of gaussian filter responses...");
    for (double sigma = 2.0; sigma <= 4.0; sigma *= 2.0) {
        // x-direction
        cvSobel(greyImg, response[k++], 1, 0, 1);
        cvSmooth(response[k - 1], response[k - 1], CV_GAUSSIAN, 
            2 * (int)(_kappa * sigma) + 1, 2 * (int)(3.0 * _kappa * sigma) + 1);

        // y-direction
        cvSobel(greyImg, response[k++], 0, 1, 1);
        cvSmooth(response[k - 1], response[k - 1], CV_GAUSSIAN, 
            2 * (int)(3.0 * _kappa * sigma) + 1, 2 * (int)(_kappa * sigma) + 1);
    }

    // laplacian of gaussian on just greyscale image
    SVL_LOG(SVL_LOG_DEBUG, "Generating laplacian of gaussian filter responses...");
    IplImage *tmpImg = cvCreateImage(cvGetSize(greyImg), IPL_DEPTH_32F, 1);
    for (double sigma = 1.0; sigma <= 8.0; sigma *= 2.0) {
        cvSmooth(greyImg, tmpImg, CV_GAUSSIAN, 2 * (int)(_kappa * sigma) + 1);
        cvLaplace(tmpImg, response[k++]);
    }

    // free memory
    cvReleaseImage(&tmpImg);
    cvReleaseImage(&greyImg);
    cvReleaseImage(&imgCIELab);
    
    SVL_ASSERT_MSG(k == NUM_FILTERS, k << " != " << NUM_FILTERS);
}



