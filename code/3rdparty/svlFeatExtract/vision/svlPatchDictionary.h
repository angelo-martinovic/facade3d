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
** FILENAME:    svlPatchDictionary.h
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
**              Sid Batra <sidbatra@cs.stanford.edu>
**              Ian Goodfellow <ia3n@cs.stanford.edu>
**              Olga Russakovsky <olga@cs.stanford.edu>
** DESCRIPTION:
**   Stores a patch dictionary. Each entry is defined by a multichannel image
**   patch of size W-by-H, and a window defined relative to some image/region.
**   Also implements a simple multi-channel patch-based object detector.
** 
*****************************************************************************/

#pragma once

#include <cassert>
#include <iostream>
#include <vector>

#include "cv.h"
#include "cxcore.h"

#include "xmlParser/xmlParser.h"

#include "svlBase.h"
#include "svlVision.h"

using namespace std;

// svlPatchDictionary ---------------------------------------------------------

class svlPatchDictionary : public svlFeatureExtractor
{
 protected:
     vector<svlSmartPointer<svlPatchDefinition> > _entries;
     int _versionNum;

  public:
     svlPatchDictionary(unsigned width = 0, unsigned height = 0, 
			int versionNum = svlPatchDefinition::CURR_VERSION);
    svlPatchDictionary(const svlPatchDictionary& d);
    virtual ~svlPatchDictionary();

    inline unsigned numFeatures() const { return (unsigned)_entries.size(); }
 
    void clear();
    void truncate(unsigned n, bool shuffle = false);
    bool load(const char *filename);
    bool load(XMLNode &root);

    pair<int, int> readClassifierSize(const char *filename);
    
    //write(string filename) and  write(const char* filename)
    //are both defined in the superclass
    bool writeOut(ofstream &ofs);
    void filterEntries(const vector<bool> &toKeep);


   // Note: if input is sparse, then integralImages are never used
     void extract(const vector<CvPoint> &locations, 
		 const vector<svlDataFrame*> &frames,
		 vector< vector<double> > &output,
		 bool sparse,
		 int outputOffset = 0) const;

    vector<vector<double> > imageResponse(const vector<IplImage *> images,
					  const vector<CvPoint>& windows,
					  bool sparse = false,
					  bool useIntegralImages = true);


    virtual svlFeatureExtractor* getPrunedExtractor(const vector<bool> &featureUsedBits) const;

    bool entriesTooCorrelated(int idx1, int idx2, float threshold) {
      return _entries[idx1]->isCorrelatedWith(_entries[idx2], threshold);
    }

    // Builds dictionary by selecting n patches from each given samples.
    // All samples should be of size _windowSize.
    // If n = 0, then instead of picking patches randomly will do it
    // by interest point detection
    //If given a pointer to masks, will not draw samples containing 1s
    //in the masks
    void buildDictionary(const vector<vector<IplImage *> >& samples,
			 const vector<svlPatchDefinitionType>& imageTypes,
			 unsigned n, bool interestPoints = false,
			 const vector<CvMat *> * masks = NULL);

    //Alters the valid channel for each of the dictionary entries
    void alterValidChannel(int newChannel);

    //Merge all entries of sgiven dictionary with the current entries
    void mergeDictionary( const svlPatchDictionary &input );

    // Constructs a huge image with all the patches in it. Good for debugging.
    IplImage *visualizeDictionary() const;
    IplImage *visualizePatch(unsigned index, IplImage *image = NULL) const;

    svlFeatureExtractor * clone() const;

    string summary() const;
};

void computeNormalizationConstants(IplImage *image, _svlPatchDefinitionType patchType,
				   const vector<CvPoint> &windows,
				   const CvSize &windowSize,
				   vector<float> &normConstants,
				   const IplImage *imageSum);


SVL_AUTOREGISTER_H( PatchDictionary , svlFeatureExtractor);
