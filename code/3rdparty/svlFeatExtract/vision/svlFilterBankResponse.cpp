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
** FILENAME:    svlFilterBankResponse.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
**
*****************************************************************************/

#include <cstdlib>
#include <cassert>
#include <iostream>
#include <fstream>

#include "Eigen/Core"

#include "cv.h"
#include "cxcore.h"

#include "svlBase.h"
#include "svlFilterBankResponse.h"
#include "svlOpenCVUtils.h"

using namespace std;
USING_PART_OF_NAMESPACE_EIGEN

// svlFilterBankResponse -------------------------------------------------------

svlFilterBankResponse::svlFilterBankResponse()
{
    // do nothing
}

svlFilterBankResponse::svlFilterBankResponse(const svlFilterBankResponse& f)
{
    // deep copy
    _responses.resize(f._responses.size());
    _sum.resize(f._sum.size());
    _sqSum.resize(f._sqSum.size());
    for (unsigned i = 0; i < f._responses.size(); i++) {
        _responses[i] = cvCloneImage(f._responses[i]);
        _sum[i] = cvCloneMat(f._sum[i]);
        _sqSum[i] = cvCloneMat(f._sqSum[i]);
    }
}

svlFilterBankResponse::~svlFilterBankResponse()
{
    clear();
}

void svlFilterBankResponse::clear()
{
    svlReleaseImages(_responses);
    svlReleaseMatrices(_sum);
    svlReleaseMatrices(_sqSum);
    _responses.clear();
    _sum.clear();
    _sqSum.clear();
}

void svlFilterBankResponse::addResponseImage(IplImage *r)
{
    SVL_ASSERT((r != NULL) && (r->depth == IPL_DEPTH_32F));
    SVL_ASSERT(empty() || ((r->width == width()) && (r->height == height())));

    // add response image
    _responses.push_back(r);

    // allocate integral response images
    _sum.push_back(cvCreateMat(height() + 1, width() + 1, CV_64FC1));
    _sqSum.push_back(cvCreateMat(height() + 1, width() + 1, CV_64FC1));

    cvIntegral(_responses.back(), _sum.back(), _sqSum.back());
}

void svlFilterBankResponse::addResponseImages(const vector<IplImage *>& r)
{
    for (unsigned i = 0; i < r.size(); i++) {
        addResponseImage(r[i]);
    }
}

void svlFilterBankResponse::copyResponseImage(const IplImage *r)
{
    SVL_ASSERT(r != NULL);
    addResponseImage(cvCloneImage(r));
}

void svlFilterBankResponse::copyResponseImages(const vector<IplImage *>& r)
{
    for (unsigned i = 0; i < r.size(); i++) {
        copyResponseImage(r[i]);
    }
}

const IplImage * svlFilterBankResponse::getResponseImage(int i) const
{
    SVL_ASSERT((i >= 0) && (i < size()));
    return _responses[i];
}

void svlFilterBankResponse::deleteResponseImage(int i)
{
    SVL_ASSERT((i >= 0) && (i < size()));

    cvReleaseImage(&_responses[i]);
    cvReleaseMat(&_sum[i]);
    cvReleaseMat(&_sqSum[i]);

    _responses.erase(_responses.begin() + i);
    _sum.erase(_sum.begin() + i);
    _sqSum.erase(_sqSum.begin() + i);
}

// pixel and region features --- no index checking
VectorXd svlFilterBankResponse::value(int x, int y) const
{
    //SVL_ASSERT((x >= 0) && (x < width()) && (y >= 0) && (y < height()));
    VectorXd v(size());

    for (int i = 0; i < size(); i++) {
        v[i] = (double)CV_IMAGE_ELEM(_responses[i], float, y, x);
    }

    return v;
}

VectorXd svlFilterBankResponse::mean(int x, int y, int w, int h) const
{
    VectorXd m(size());

    for (int i = 0; i < size(); i++) {
        m[i] = CV_MAT_ELEM(*_sum[i], double, y, x) +
            CV_MAT_ELEM(*_sum[i], double, y + h, x + w) -
            CV_MAT_ELEM(*_sum[i], double, y + h, x) -
            CV_MAT_ELEM(*_sum[i], double, y, x + w);
    }

    m /= (double)(w * h);
    return m;
}

VectorXd svlFilterBankResponse::energy(int x, int y, int w, int h) const
{
    VectorXd e(size());

    for (int i = 0; i < size(); i++) {
        e[i] = CV_MAT_ELEM(*_sqSum[i], double, y, x) +
            CV_MAT_ELEM(*_sqSum[i], double, y + h, x + w) -
            CV_MAT_ELEM(*_sqSum[i], double, y + h, x) -
            CV_MAT_ELEM(*_sqSum[i], double, y, x + w);
    }

    return e;
}

VectorXd svlFilterBankResponse::variance(int x, int y, int w, int h) const
{
    return energy(x, y, w, h) / (double)(w * h) - mean(x, y, w, h).cwise().square();
}

VectorXd svlFilterBankResponse::mean(const list<CvPoint>& pixels) const
{
    VectorXd m = VectorXd::Zero(size());

    if (pixels.empty()) {
        return m;
    }

    for (list<CvPoint>::const_iterator ip = pixels.begin(); ip != pixels.end(); ip++) {
        m += value(ip->x, ip->y);
    }

    m /= (double)pixels.size();
    return m;
}

VectorXd svlFilterBankResponse::energy(const list<CvPoint>& pixels) const
{
    VectorXd e = VectorXd::Zero(size());

    if (pixels.empty()) {
        return e;
    }

    for (list<CvPoint>::const_iterator ip = pixels.begin(); ip != pixels.end(); ip++) {
        e += value(ip->x, ip->y).cwise().square();
    }

    return e;
}

VectorXd svlFilterBankResponse::variance(const list<CvPoint>& pixels) const
{
    return energy(pixels) / (double)pixels.size() - mean(pixels).cwise().square();
}

VectorXd svlFilterBankResponse::mean(const CvMat *mask) const
{
    SVL_ASSERT((mask != NULL) && (cvGetElemType(mask) == CV_8UC1));
    SVL_ASSERT((mask->rows == height()) && (mask->cols == width()));

    VectorXd m = VectorXd::Zero(size());

    const unsigned char *p = (const unsigned char *)CV_MAT_ELEM_PTR(*mask, 0, 0);
    int count = 0;
    for (int y = 0; y < height(); y++) {
        int xStart = 0;
        for (int x = 0; x < width(); x++, p++) {
            if (*p == 0) {
                if (xStart != x) {
                    if (xStart == x - 1) {
                        // singleton case
                        m += value(x - 1, y);
                    } else {
                        // line case
                        for (int i = 0; i < size(); i++) {
                            const double *q = (const double *)CV_MAT_ELEM_PTR(*_sum[i], y, 0);
                            m[i] +=  q[xStart] + q[x + width() + 1] - q[xStart + width() + 1] - q[x];
                        }
                    }
                }
                count += x - xStart;
                xStart = x + 1;
            }
        }

        // special case of region ends at boundary
        if (xStart != width()) {
            if (xStart == width() - 1) {
                // singleton case
                m += value(width() - 1, y);
            } else {
                // line case
                for (int i = 0; i < size(); i++) {
                    const double *q = (const double *)CV_MAT_ELEM_PTR(*_sum[i], y, 0);
                    m[i] +=  q[xStart] + q[2 * width() + 1] - q[xStart + width() + 1] - q[width()];
                }
            }
            count += width() - xStart;
        }
    }

    // normalize
    if (count != 0) {
        m /= (double)count;
    }

    return m;
}

VectorXd svlFilterBankResponse::energy(const CvMat *mask) const
{
    SVL_ASSERT((mask != NULL) && (cvGetElemType(mask) == CV_8UC1));
    SVL_ASSERT((mask->rows == height()) && (mask->cols == width()));

    VectorXd e = VectorXd::Zero(size());

    const unsigned char *p = (const unsigned char *)CV_MAT_ELEM_PTR(*mask, 0, 0);
    int count = 0;
    for (int y = 0; y < height(); y++) {
        int xStart = 0;
        for (int x = 0; x < width(); x++, p++) {
            if (*p == 0) {
                if (xStart != x) {
                    if (xStart == x - 1) {
                        // singleton case
                        e += value(x - 1, y).cwise().square();
                    } else {
                        // line case
                        for (int i = 0; i < size(); i++) {
                            const double *q = (const double *)CV_MAT_ELEM_PTR(*_sqSum[i], y, 0);
                            e[i] +=  q[xStart] + q[x + width() + 1] - q[xStart + width() + 1] - q[x];
                        }
                    }
                }
                count += x - xStart;
                xStart = x + 1;
            }
        }

        // special case of region ends at boundary
        if (xStart != width()) {
            if (xStart == width() - 1) {
                // singleton case
                e += value(width() - 1, y).cwise().square();
            } else {
                // line case
                for (int i = 0; i < size(); i++) {
                    const double *q = (const double *)CV_MAT_ELEM_PTR(*_sqSum[i], y, 0);
                    e[i] +=  q[xStart] + q[2 * width() + 1] - q[xStart + width() + 1] - q[width()];
                }
            }
            count += width() - xStart;
        }
    }

    return e;
}

VectorXd svlFilterBankResponse::variance(const CvMat *mask) const
{
    SVL_ASSERT((mask != NULL) && (cvGetElemType(mask) == CV_8UC1));
    SVL_ASSERT((mask->rows == height()) && (mask->cols == width()));

    VectorXd m = VectorXd::Zero(size());
    VectorXd e = VectorXd::Zero(size());

    const unsigned char *p = (const unsigned char *)CV_MAT_ELEM_PTR(*mask, 0, 0);
    int count = 0;
    for (int y = 0; y < height(); y++) {
        int xStart = 0;
        for (int x = 0; x < width(); x++, p++) {
            if (*p == 0) {
                if (xStart != x) {
                    if (xStart == x - 1) {
                        // singleton case
                        m += value(x - 1, y);
                        e += value(x - 1, y).cwise().square();
                    } else {
                        // line case
                        for (int i = 0; i < size(); i++) {
                            const double *q = (const double *)CV_MAT_ELEM_PTR(*_sum[i], y, 0);
                            m[i] +=  q[xStart] + q[x + width() + 1] - q[xStart + width() + 1] - q[x];
                            q = (const double *)CV_MAT_ELEM_PTR(*_sqSum[i], y, 0);
                            e[i] +=  q[xStart] + q[x + width() + 1] - q[xStart + width() + 1] - q[x];
                        }
                    }
                }
                count += x - xStart;
                xStart = x + 1;
            }
        }

        // special case of region ends at boundary
        if (xStart != width()) {
            if (xStart == width() - 1) {
                // singleton case
                m += value(width() - 1, y);
                e += value(width() - 1, y).cwise().square();
            } else {
                // line case
                for (int i = 0; i < size(); i++) {
                    const double *q = (const double *)CV_MAT_ELEM_PTR(*_sum[i], y, 0);
                    m[i] +=  q[xStart] + q[2 * width() + 1] - q[xStart + width() + 1] - q[width()];
                    q = (const double *)CV_MAT_ELEM_PTR(*_sqSum[i], y, 0);
                    e[i] +=  q[xStart] + q[2 * width() + 1] - q[xStart + width() + 1] - q[width()];
                }
            }
            count += width() - xStart;
        }
    }

    if (count == 0) {
        return VectorXd::Zero(size());
    }

    return e/(double)count - (m / (double)count).cwise().square();
}

