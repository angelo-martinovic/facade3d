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
** FILENAME:    svlPatchFeatureSelector.h
** AUTHOR(S):   Olga Russakovsky <olga@cs.stanford.edu>
**
** DESCRIPTION:
**  Implements a way 
**
*****************************************************************************/

#pragma once

#include <cassert>
#include <iostream>
#include <vector>
#include <map>

#include "cv.h"
#include "cxcore.h"
#include "ml.h"

#include "svlBase.h"
#include "svlML.h"
#include "svlPatchDictionary.h"
using namespace std;

// svlPatchFeatureSelector ----------------------------------------------------------

class svlPatchFeatureSelector : public svlFeatureSelector {
 public:
  static double F_SCORE_THRESHOLD;
  static double CCORR_THRESHOLD;

  svlPatchFeatureSelector(svlPatchDictionary *d) : _dict(d) {}

 protected:
  virtual bool cacheExamples(const vector<vector<double> > &posExampleResponses,
			     const vector<vector<double> > &negExampleResponses);
  virtual bool chooseFeatures(vector<bool> &selectedFeatures);

 private:
  svlPatchDictionary *_dict;
  vector<pair<double, int> > _patchFScores;
};

