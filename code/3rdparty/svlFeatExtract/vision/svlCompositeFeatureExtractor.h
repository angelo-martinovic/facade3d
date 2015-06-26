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
** FILENAME:    svlCompositeFeatureExtractor.h
** AUTHOR(S):   Ethan Dreyfuss <ethan@cs.stanford.edu>
**              Ian Goodfellow <ia3n@cs.stanford.edu>
** DESCRIPTION:
**   Allows multiple detectorFeatureExtractors to be combined into a single
**   composite detector feature extractor.
**
*****************************************************************************/

#pragma once

#include "svlFeatureExtractor.h"

typedef vector<svlFeatureExtractor*> extractorVec;

class svlCompositeFeatureExtractor : public svlFeatureExtractor
{
 public:
  svlCompositeFeatureExtractor();
  svlCompositeFeatureExtractor(const extractorVec &vec);

  void extract(const vector<CvPoint> &locations,
	      const vector<svlDataFrame*> &frames,
	      vector< vector<double> > &output,
	      bool sparse,
	      int outputOffset = 0) const;


  virtual void windowResponse(const vector<IplImage *> & channels,
                              vector<double> & output,
                              int outputOffset = 0) const;

  virtual bool writeOut(ofstream &ofs);
  virtual bool load(XMLNode &root);
  virtual svlFeatureExtractor* clone() const;
  virtual svlFeatureExtractor* getPrunedExtractor
                                        (const vector<bool> &featureUsedBits) const;

  virtual unsigned numFeatures() const;

  string summary() const;

 private:
  extractorVec delegates;
};

SVL_AUTOREGISTER_H( CompositeFeatureExtractor , svlFeatureExtractor);
