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
** FILENAME:    svlHaarFeatureExtractor.h
** AUTHOR(S):   Ian Goodfellow <ia3n@cs.stanford.edu>
** DESCRIPTION:
**   Extracts HAAR features
**
*****************************************************************************/

#pragma once

#include "svlFeatureExtractor.h"


//todo-- change this to use opencv haar features?
//todo-- change this to assume the image has been scaled rather than rescaling all boxes? (this code was just ripped out of the old person detector project where boxes were projected from the 3d so there were tons of different scales)

enum svlHaarFeatureType
  {
    SVL_HAAR_H=0,
    SVL_HAAR_V,
    SVL_HAAR_D,
    SVL_HAAR_TL, 
    SVL_HAAR_TR, 
    SVL_HAAR_BL,
    SVL_HAAR_BR,
    SVL_HAAR_FULL_AVE,
    SVL_HAAR_ERROR
  };

class svlHaarFeatureExtractor : public svlFeatureExtractor
{
 public:
  svlHaarFeatureExtractor() : _validChannel(0) {}
  // to suppress warnings
  virtual ~svlHaarFeatureExtractor() {}


  void extract(const vector<CvPoint> &locations,
	      const vector<svlDataFrame*> &frames,
	      vector< vector<double> > &output,
	      bool sparse,
	      int outputOffset = 0) const;

  bool writeOut(ofstream &ofs);

  bool load(XMLNode &root);

  svlFeatureExtractor* getPrunedExtractor
                                        (const vector<bool> &featureUsedBits) const;

  unsigned numFeatures() const ;

  svlFeatureExtractor * clone() const;

  string summary() const;


 protected:
  //featureArea should be a proportion rectangle (x,y,w,h are all in [0,1] and specify where to extract the feature within the window)
  static double calcHaarFeature(const IplImage * iImg, const CvRect & focus, const svlObject2d & featureArea, svlHaarFeatureType type);


//Uses the integral image trick to return the sum over pixel values in a rectangle
static double iImgSum(const IplImage * iImg, const CvRect & rect);

//Maps a space inside a rectangle to the corresponding rectangle in the image
//ie, propRect has x,y,w,y in [0,1] and is just defined as a proportion of imRect
//imRect is a rectangle in any image
//This returns the rectangle in the image that propRect of imRect occupies
static CvRect rectInRect(const svlObject2d & propRect, const CvRect & imRect);


 static string haarTypeToStr(svlHaarFeatureType type);
 static svlHaarFeatureType strToHaarType(const string & str);

 protected:

    unsigned _validChannel;
    vector<svlObject2d> _locations;//should be proportion rectangles
    vector<svlHaarFeatureType> _featureTypes;//one for each location
};


SVL_AUTOREGISTER_H( HaarFeatureExtractor , svlFeatureExtractor);
