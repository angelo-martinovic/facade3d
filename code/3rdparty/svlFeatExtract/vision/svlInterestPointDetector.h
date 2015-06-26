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
** FILENAME:    svlInterestPointDetector.h
** AUTHOR(S):   Ian Goodfellow <ia3n@cs.stanford.edu>
** DESCRIPTION: A pure virtual class describing the methods
**   an interest point detector should provide. Other classes
**   can then implement these methods with various methods such
**   as Harris corner detection, SIFT features, etc.
**
*****************************************************************************/

#pragma once

#include "cv.h"
#include "cxcore.h"
#include <fstream>
#include <vector>
#include "xmlParser/xmlParser.h"
using namespace std;

class svlInterestPointDetector
{
 public:
  virtual ~svlInterestPointDetector() {
      // do nothing
  }

  virtual void findInterestPoints(const IplImage * img,
				  vector<CvPoint> & output) = 0;

  virtual void write(ofstream & ofs) const = 0;

  static svlInterestPointDetector * readXMLNode(XMLNode & node);

  virtual bool operator!=(const svlInterestPointDetector & other) const = 0;
};

class svlScaledInterestPointDetector : public svlInterestPointDetector
{
 public:
    virtual ~svlScaledInterestPointDetector() {
        // do nothing
    }
    
    virtual void findInterestPointsScaled(const IplImage *img,
        vector<CvPoint> &output,
        vector<float> &outputScales) = 0;
};
