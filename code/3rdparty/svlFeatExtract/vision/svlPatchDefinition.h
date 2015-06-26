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
** FILENAME:    svlPatchDefinition.h
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
**              Sid Batra <sidbatra@cs.stanford.edu>
**              Olga Russakovsky <olga@cs.stanford.edu>
**              Ian Goodfellow <ia3n@cs.stanford.edu>
** DESCRIPTION:
**   Stores a generic definition of a patch. Which is inherited by patch
**   definitions to represent seperate channels. The patches are written
**   and read in XML format.
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
#include "svlVisionUtils.h"

using namespace std;

// svlPatchDefinitionType ----------------------------------------------------

typedef enum _svlPatchDefinitionType {
  SVL_INTENSITY_PATCH, // one-channel image patch
  SVL_DEPTH_PATCH,    // depthmap patch
} svlPatchDefinitionType;

inline string svlPatchDefinitionTypeToText(svlPatchDefinitionType type) {
    switch(type) {
    case SVL_INTENSITY_PATCH: return "INTENSITY";
    case SVL_DEPTH_PATCH: return "DEPTH";
    }
    return "";
}

// svlPatch struct ---------------------------------------------------------
// A small struct definition of a patch (used for cuda)
struct svlPatch {
  IplImage *patch;	// (Single-channel) patch template
  CvRect rect;		// Valid response region
  int channel;		// Channel information
  
  svlPatch() {}
  svlPatch(IplImage *patch, CvRect rect, int channel)
    : patch(patch), rect(rect), channel(channel) {}
  ~svlPatch() {
    if ( patch ) cvReleaseImage(&patch);
  }
  void print() {
    for ( int i=0; i<patch->height; ++i ) {
      for ( int j=0; j<patch->width; ++j ) {
	if ( patch->depth == IPL_DEPTH_8U )
	  printf(" %d", CV_IMAGE_ELEM(patch, unsigned char, i, j));
	else if ( patch->depth == (int)IPL_DEPTH_32S )
	  printf(" %d", CV_IMAGE_ELEM(patch, int, i, j));
	else if ( patch->depth == IPL_DEPTH_32F )
	  printf(" %0.4f", CV_IMAGE_ELEM(patch, float, i, j));
	else if ( patch->depth == IPL_DEPTH_64F )
	  printf(" %0.4f", CV_IMAGE_ELEM(patch, double, i, j));
      }
      cout << endl;
    }
  }
};

// svlPatchDefinition --------------------------------------------------------

class svlPatchDictionary;
struct svlPatch;

class svlPatchDefinition 
{
    friend class svlPatchDictionary;

 protected:
    IplImage *_template;    // (single channel) patch template
    CvRect _validRect;       // valid response region
    int _validChannel;      // channel information

    // precomputed for efficiency (deprecated version)
    float _templateSum;
    float _templateNormSq; // (sum of squares of pixel values)

    int _versionNum; // current version is CURR_VERSION

    static const int UNSPECIFIED_VERSION = -1;
  public:
    static const int CURR_VERSION = 2;

    svlPatchDefinition();
    svlPatchDefinition(XMLNode& node, int currVersion = CURR_VERSION);
    svlPatchDefinition(const IplImage *t, const CvRect& rect, int channel = 0, 
		       int versionNum = UNSPECIFIED_VERSION);

    virtual ~svlPatchDefinition();

    void setVersionNumber(int n) { _versionNum = n; }

    // constuction helper functions
    static svlSmartPointer<svlPatchDefinition>
      createPatchDefinition(svlPatchDefinitionType t);
    static svlSmartPointer<svlPatchDefinition>
      createPatchDefinition(XMLNode& node, int currVersion = CURR_VERSION);

    // i/o functions
    // these are no longer different for different patches, so will keep them
    // non-virtual for now (can be overwritten later)
    bool read(XMLNode& node);
    bool write(ostream& os);

    //Returns the valid channel index
    inline int validChannel() const { return _validChannel; }
    void setValidChannel( int newChannel ) { _validChannel = newChannel ; }

    inline const IplImage* getTemplate() { return _template; }
    inline int getValidChannel() { return _validChannel; }
    inline CvRect getValidRect() { return CvRect(_validRect); }
    inline int getTemplateWidth() { return _template->width; }
    inline int getTemplateHeight() { return _template->height; }
		inline int getTemplateSize() { return _template->width * _template->height; }

    bool isCorrelatedWith(const svlPatchDefinition *patch, double threshold) const;

    // Read in a patch as a struct.
    // todo: fix for cuda to be able to use the 8U patches again
    static svlPatch* readPatch(XMLNode &node, int versionNum = CURR_VERSION);

    //////////////////
    // Virtual functions
    /////////////////
    virtual svlPatchDefinition * clone() const = 0;
    virtual svlPatchDefinitionType patchType() const = 0;
    virtual bool isPatchValid() { return !svlImageUniform(_template); }
    virtual bool isSameType(const svlPatchDefinition *patch) const
    { return validChannel() == patch->validChannel(); }

    // Compute response image for input image. The caller is responsible for 
    // freeing memory. Input images should have the correct number of channels
    // for the patch feature. Non-vectorized inputs assume that the correct
    // channel is being passed in (usually channel 0).
    IplImage *responseImage(const IplImage *image,
			    const IplImage *imageSum,
			    const IplImage *imageSumSq) const;
       
    // Computes the response value for the patch at any given location of the
    // sliding window. responseImage should have been computed by responseImage()
    // function. The second is the unoptimized version, where everything
    // is computed directly from the original image
    double patchValue(const CvPoint &location, float normalizationConstant,
		      const IplImage *responseImage, 
		      const IplImage *imageSum, const IplImage *imageSumSq) const;
    double patchValue(const IplImage *image,
		      const CvPoint &location, float normalizationConstant) const;

    // Computes the response value for all the given patches; it is
    // optimized for sparsity, so if you want to compute values
    // for a dense set of locations, responseImage (and integral
    // image computations) followed by patchValue calls
    // are what you want
    vector<double> patchValues(const IplImage *image, 
			       const vector<CvPoint>& locations,
			       const vector<float> &normConstants) const;
    
 private:
    double patchValueHelper(IplImage *image, // can be NULL if optimized
			    const CvPoint& location,
			    // for the window where the feature is computed
			    float normalizationConstant,
			    // if optimized, can pass in these instead
			    // of the original image
			    const IplImage *responseImage, 
			    const IplImage *imageSum,
			    const IplImage *imageSumSq) const;

    // deprecated version
    double patchValueHelperVersion0(const IplImage *image,
				    const CvPoint& location,
				    const IplImage *responseImage, 
				    const IplImage *imageSum,
				    const IplImage *imageSumSq) const;

    double patchValueHelperVersion1(const IplImage *image,
				    const CvPoint& location,
				    float normalizationConstant,
				    const IplImage *responseImage, 
				    const IplImage *imageSum,
				    const IplImage *imageSumSq) const;

};

// Statistics on a patch.
struct svlPatchStats {
	float mean, std, *patch, *mask;
	int w, h;
	svlPatchStats() {}
	svlPatchStats(float mean, float std, float *patch, float *mask, int w, int h)
		: mean(mean), std(std), patch(patch), mask(mask), w(w), h(h) {}
	~svlPatchStats() {
		if ( patch ) delete patch;
		if ( mask ) delete mask;
	}
};

// svlIntensityPatchDefinition -----------------------------------------------

class svlIntensityPatchDefinition  : public svlPatchDefinition
{
 public:
    svlIntensityPatchDefinition();
    svlIntensityPatchDefinition(XMLNode& node, int versionNum = CURR_VERSION);
    svlIntensityPatchDefinition(const IplImage *t, const CvRect& rect, int channel = 0,
				int versionNum = CURR_VERSION);
    ~svlIntensityPatchDefinition();
    
    virtual svlPatchDefinition * clone() const 
    {
      return new svlIntensityPatchDefinition(_template, _validRect, _validChannel); 
    }

    svlPatchDefinitionType patchType() const { return SVL_INTENSITY_PATCH; }

 protected:
    static map<IplImage*,svlPatchStats*> patch_cache;
    static void clearPatchCache() {
      for ( map<IplImage*,svlPatchStats*>::iterator it=patch_cache.begin(); it!=patch_cache.end(); ++it )
	delete it->second;
      patch_cache.clear();
    }
};





// svlDepthPatchDefinition ---------------------------------------------------

class svlDepthPatchDefinition  : public svlPatchDefinition
{
 public:
    svlDepthPatchDefinition();
    svlDepthPatchDefinition(XMLNode& node, int versionNum = CURR_VERSION);
    svlDepthPatchDefinition(const IplImage *t, const CvRect& rect, int channel = 1,
			    int versionNum = CURR_VERSION);
    ~svlDepthPatchDefinition();

    virtual svlPatchDefinition * clone() const 
    {
      return new svlDepthPatchDefinition(_template, _validRect, _validChannel); 
    }
        // i/o functions
    svlPatchDefinitionType patchType() const { return SVL_DEPTH_PATCH; }
};



