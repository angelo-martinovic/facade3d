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
** FILENAME:    svlFilterBankResponse.h
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
** DESCRIPTION:
**  Holds the results of running an image through a bank of filters and allows
**  for computation of features over rectangular regions.
**
*****************************************************************************/

#pragma once

#include <list>
#include <vector>

#include "Eigen/Core"

#include "cv.h"
#include "cxcore.h"

using namespace std;
USING_PART_OF_NAMESPACE_EIGEN

// svlFilterBankResponse -------------------------------------------------------

class svlFilterBankResponse {
 protected:
    vector<IplImage *> _responses;  // filter responses
    vector<CvMat *> _sum;           // sum of filter responses (integral image)
    vector<CvMat *> _sqSum;         // sum of filter responses squared

 public:
    svlFilterBankResponse();
    svlFilterBankResponse(const svlFilterBankResponse& f);
    ~svlFilterBankResponse();

    void clear();
    inline bool empty() const { return _responses.empty(); }
    inline int size() const { return (int)_responses.size(); }
    inline int width() const { return _responses.empty() ? 0 : _responses[0]->width; }
    inline int height() const { return _responses.empty() ? 0 : _responses[0]->height; }

    // The difference between addResponseImage and copyResponseImage is that
    // the former takes ownership of the images and is responsible for freeing
    // memory. Response images should be 32-bit floating point.
    void addResponseImage(IplImage *r);
    void addResponseImages(const vector<IplImage *>& r);
    void copyResponseImage(const IplImage *r);
    void copyResponseImages(const vector<IplImage *>& r);
    const IplImage *getResponseImage(int i) const;
    void deleteResponseImage(int i);

    // pixel and region features
    VectorXd value(int x, int y) const;
    VectorXd mean(int x, int y, int w, int h) const;
    VectorXd energy(int x, int y, int w, int h) const;
    VectorXd variance(int x, int y, int w, int h) const;

    VectorXd mean(const list<CvPoint>& pixels) const;
    VectorXd energy(const list<CvPoint>& pixels) const;
    VectorXd variance(const list<CvPoint>& pixels) const;

    VectorXd mean(const CvMat *mask) const;
    VectorXd energy(const CvMat *mask) const;
    VectorXd variance(const CvMat *mask) const;

    inline VectorXd mean() const { return mean(0, 0, width(), height()); }
    inline VectorXd energy() const { return energy(0, 0, width(), height()); }
    inline VectorXd variance() const { return variance(0, 0, width(), height()); }
};



