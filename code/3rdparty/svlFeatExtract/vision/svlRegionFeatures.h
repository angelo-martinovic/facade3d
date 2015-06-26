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
** FILENAME:    svlRegionFeatures.h
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
** DESCRIPTION:
**  Computes features over regions (image segments).
**
*****************************************************************************/

#pragma once

#include <vector>

#include "cv.h"
#include "cxcore.h"

#include "svlBase.h"
#include "svlConvolution.h"
#include "svlSegImage.h"

using namespace std;

// svlRegionFeatures --------------------------------------------------------

class svlRegionFeatures : public svlOptions {
 protected:
    vector<svlConvolution> _filterBank;
    bool _bIncludeStdev;
    bool _bIncludeSkewness;
    bool _bIncludeKurtosis;

 public:
    svlRegionFeatures();
    virtual ~svlRegionFeatures();

    // compute feature responses
    vector<vector<double> > computeFeatures(IplImage *image,
	const vector<vector<CvPoint> >& regions);
    vector<vector<double> > computeFeatures(const svlSegImageBase& image);

 protected:
    void populateFilterBank();
    inline vector<double> regionStatistics(const IplImage *response, 
	const vector<CvPoint>& region);
    inline vector<double> regionStatistics(const CvMat *response, 
	const vector<CvPoint>& region);
};


