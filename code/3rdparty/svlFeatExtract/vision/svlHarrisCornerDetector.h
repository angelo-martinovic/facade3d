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
** FILENAME:    svlHarrisCornerDetector.h
** AUTHOR(S):   Ian Goodfellow <ia3n@cs.stanford.edu>
** DESCRIPTION:
**   Implements svlInterestPointDetector using Harris corner detection
**
*****************************************************************************/

#pragma once

#include "svlInterestPointDetector.h"

class svlHarrisCornerDetector : public svlInterestPointDetector
{
 public:
  svlHarrisCornerDetector(int blockSize, int apertureSize, double k, int _maxWinWidth, int maxWinHeight, double responseThreshold);
  svlHarrisCornerDetector(XMLNode & node);
  virtual ~svlHarrisCornerDetector() { }
  void findInterestPoints(const IplImage * img, vector<CvPoint> & output);
  void setResponseShowing(bool show ) { _showResponse = show; }

  void write(ofstream & ofs) const;

  bool operator!=(const svlInterestPointDetector & other) const;

 protected:
  
  //Arguments to cvCornerHarris
  int _blockSize;
  int _apertureSize;
  double _k;

  //Harris response is filtered to find local maxima within this window size
  int _maxWinWidth;
  int _maxWinHeight;

  //Only interest points with at least this Harris threshold are returned
  double _responseThreshold; 

  bool _showResponse;
  
};
