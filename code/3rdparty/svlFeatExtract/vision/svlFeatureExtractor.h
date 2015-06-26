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
** FILENAME:    svlFeatureExtractor.h
** AUTHOR(S):   Ethan Dreyfuss <ethan@cs.stanford.edu>
**              Ian Goodfellow <ia3n@cs.stanford.edu>
** DESCRIPTION:
**     Abstract class for extracting features from rectangles in an image.
**     Examples subclasses include svlPatchDictionary, svlHaarFeatureExtractor,
**     etc.
**
*****************************************************************************/

//TODO: add some facility for different extractors t share integral
//images / any other common large intermediate data structures

#pragma once

#include <string>
#include <vector>
#include <fstream>
#include <cv.h>
#include "xmlParser/xmlParser.h"
#include "svlDataFrame.h"
#include "svlBase.h"

using namespace std;

class svlObject2d;

class svlFeatureExtractor
{
 public:
  // to suppress warnings
  virtual ~svlFeatureExtractor() {}

  void extract(const vector<CvPoint> &locations, 
	       const vector<IplImage*> &channels, 
	       vector< vector<double> > &output,
	       bool sparse,
	       int outputOffset = 0) const; 

  virtual void extract(const vector<CvPoint> &locations,
		       const vector<svlDataFrame*> &frames,
		       vector< vector<double> > &output,
		       bool sparse,
		       int outputOffset = 0) const = 0;

  //default is a wrapper around extract with the sparse flag set to true.
  //if this results in inefficient behavior for your feature, then implement
  //your own version
  virtual void windowResponse(const vector<IplImage *> & channels,
                              vector<double> & output,
                              int outputOffset = 0,
			      const vector<bool> *usedFeatures = NULL) const;


  bool write(const string &filename);
  bool write(const char *filename);
  virtual bool writeOut(ofstream &ofs) = 0;

  virtual bool load(XMLNode &root) = 0;
  bool load(const char * filepath);

  virtual svlFeatureExtractor* getPrunedExtractor
    (const vector<bool> &featureUsedBits) const = 0;

  inline unsigned windowWidth() const { return _windowSize.width; }
  inline unsigned windowHeight() const { return _windowSize.height; }

  virtual unsigned numFeatures() const = 0;

  virtual svlFeatureExtractor * clone() const = 0;

  virtual string summary() const;

  virtual void setSize(CvSize size);

 protected:

    CvSize _windowSize;

};

svlFactory<svlFeatureExtractor> & svlFeatureExtractorFactory();
