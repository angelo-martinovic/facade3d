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
** FILENAME:    svlPatchDefinition.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
**              Sid Batra<sidbatra@cs.stanford.edu>
**              Olga Russakovsky <olga@cs.stanford.edu>
**              Ian Goodfellow <ia3n@cs.stanford.edu>
**              Adam Coates <acoates@stanford.edu>
**
*****************************************************************************/

#include <cassert>
#include <iostream>
#include <fstream>
#include <vector>
#include <limits>

#include "cv.h"
#include "cxcore.h"
#include "highgui.h"

#include "svlPatchDefinition.h"
#include "svlVision.h"

using namespace std;


// static function prototypes -------------------------------------------------------------
static void responseImageOrig(const IplImage* image, const IplImage* t,
			      IplImage* response, const IplImage* imageSum,
			      const IplImage* imageSumSq);
static void matchTemplateCoeffn(const IplImage* img, const IplImage* t,
				IplImage* response, const IplImage* intImg,
				const IplImage* intImgSqr);
static double correlationResponse(const IplImage *img, const IplImage *tmpl);
static double correlationResponseWithExpansion(const IplImage *imBiggerWidth,
					       const IplImage *imBiggerHeight);
static bool enoughOverlap(CvRect rect1, int w1, int h1, CvRect rect2, int w2, int h2);

map<IplImage*,svlPatchStats*> svlIntensityPatchDefinition::patch_cache = map<IplImage*,svlPatchStats*>();


// svlPatchDefinition class --------------------------------------------------

svlPatchDefinition::svlPatchDefinition() :
  _template(NULL), _validChannel(0),  _templateSum(0), _templateNormSq(0), _versionNum(CURR_VERSION)
{
    _validRect = cvRect(0, 0, 0, 0);
}

svlPatchDefinition::svlPatchDefinition(XMLNode& node, int versionNum) :
  _template(NULL), _validChannel(0), _templateSum(0), _templateNormSq(0), _versionNum(versionNum)
{
    _validRect = cvRect(0, 0, 0, 0);
    //read(node);
}

svlPatchDefinition::svlPatchDefinition(const IplImage *t, const CvRect& rect, int channel,
				       int versionNum)
{
  if (versionNum == UNSPECIFIED_VERSION) {
    _versionNum = (t->depth == IPL_DEPTH_8U) ? CURR_VERSION : CURR_VERSION-1;
    SVL_LOG(SVL_LOG_WARNING, "Inferring svlPatchDefinition version number from template type as " 
	    << _versionNum);
  } else {
    _versionNum = versionNum;
  }

  if (t != NULL) {
        _template = cvCloneImage(t);
    } else {
        _template = NULL;
    }
    _validRect.x = rect.x;
    _validRect.y = rect.y;
    _validRect.width = rect.width;
    _validRect.height = rect.height;
    _validChannel = channel;

    CvScalar sc = cvSum(t);
    _templateSum = sc.val[0];

    _templateNormSq = cvDotProduct(t, t);
}

svlPatchDefinition::~svlPatchDefinition()
{
  if (_template != NULL) {
    cvReleaseImage(&_template);
    
    _template = NULL;
  }
}

// constuction helper functions
svlSmartPointer<svlPatchDefinition> svlPatchDefinition::createPatchDefinition(svlPatchDefinitionType t)
{
  svlSmartPointer<svlPatchDefinition> p;
  switch (t) {
  case SVL_INTENSITY_PATCH:
    p = svlSmartPointer<svlPatchDefinition>(new svlIntensityPatchDefinition());
        break;
  case SVL_DEPTH_PATCH:
    p = svlSmartPointer<svlPatchDefinition>(new svlDepthPatchDefinition());
    break;
  default:
    break; // to suppress compiler warnings
  }
  
  return p;
}

svlSmartPointer<svlPatchDefinition> svlPatchDefinition::createPatchDefinition(XMLNode &node,
									      int versionNum)
{
    SVL_ASSERT(!node.isEmpty());
    svlSmartPointer<svlPatchDefinition> p;

    const char *t = node.getAttribute("type");
    SVL_ASSERT(t != NULL);

    if (!strcasecmp(t, "intensity")) {
      p = svlSmartPointer<svlPatchDefinition>(new svlIntensityPatchDefinition(node, versionNum));
    } else if (!strcasecmp(t, "depth")) { 
      p = svlSmartPointer<svlPatchDefinition>(new svlDepthPatchDefinition(node, versionNum));
     } else {
        SVL_LOG(SVL_LOG_FATAL, "unknown patch type " << t);
    }

    return p;
}

bool svlPatchDefinition::read(XMLNode &node)
{		
    SVL_ASSERT(!node.isEmpty());
    if (_template != NULL) {
        cvReleaseImage(&_template);
    }

    svlPatch *tmp = readPatch(node, _versionNum);
    _validChannel = tmp->channel;
    _validRect = tmp->rect;
    _template = cvCloneImage(tmp->patch);

    CvScalar sc = cvSum(_template);
    _templateSum = sc.val[0];

    _templateNormSq = cvDotProduct(_template, _template);
    delete tmp;

    return true;
}

// Read a patch and return as a struct pointer.
svlPatch* svlPatchDefinition::readPatch(XMLNode &node, int versionNum)
{
	svlPatch *ret = new svlPatch();

	// Read valid region
	if ( node.getAttribute("validRect") ) {
		vector<int> v;
		parseString<int>(node.getAttribute("validRect"), v);
		SVL_ASSERT(v.size() == 4);
		ret->rect = cvRect(v[0], v[1], v[2], v[3]);
	} else {
		ret->rect = cvRect(0, 0, 0, 0);
	}

	// Read channel
	if ( node.getAttribute("channel") ) {
		ret->channel = atoi(node.getAttribute("channel"));
	} else {
		ret->channel = 0;
	}

	// Read patch size
	int w = 0;
	int h = 0;
	if (node.getAttribute("size")) {
		vector<int> v;
		parseString<int>(node.getAttribute("size"), v);
		SVL_ASSERT(v.size() == 2);
		w = v[0]; h = v[1];
	}

	// Read patch data
	if ( w > 0 && h > 0 ) {
	  if (versionNum == 0 || versionNum == 2) {
	    ret->patch = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 1);
	    vector<int> v;
	    parseString<int>(node.getText(), v);
	    SVL_ASSERT(v.size() == (unsigned)(w * h));
	    unsigned n = 0;
	    for (unsigned y = 0; y < (unsigned)ret->patch->height; y++) {
	      uchar *p = &CV_IMAGE_ELEM(ret->patch, uchar, y, 0);
	      for (unsigned x = 0; x < (unsigned)ret->patch->width; x++) {
		p[x] = v[n++];
	      }
	    }
	  } else {
	    ret->patch = cvCreateImage(cvSize(w, h), IPL_DEPTH_32F, 1);
	    vector<float> v;
	    parseString<float>(node.getText(), v);
	    SVL_ASSERT(v.size() == (unsigned)(w * h));
	    unsigned n = 0;
	    for (unsigned y = 0; y < (unsigned)ret->patch->height; y++) {
	      float *p = &CV_IMAGE_ELEM(ret->patch, float, y, 0);
	      for (unsigned x = 0; x < (unsigned)ret->patch->width; x++) {
		p[x] = v[n++];
	      }
	    }
	  }
	}

	return ret;
}

bool svlPatchDefinition::write(ostream& os)
{
    SVL_ASSERT(_template != NULL);
    if (_versionNum == 2 || _versionNum == 0) {
      SVL_ASSERT_MSG(_template->depth == IPL_DEPTH_8U, "All patches now expected to have 8U depth");
    } else {
      SVL_ASSERT_MSG(_template->depth == IPL_DEPTH_32F, "All patches now expected to have 32F depth");
    }

    os << "  <PatchDefinition type=\"";
    switch (this->patchType()) 
	{
    case SVL_INTENSITY_PATCH:
        os << "intensity";
        break;
    case SVL_DEPTH_PATCH:
        os << "depth";
        break;
    default:
        SVL_LOG(SVL_LOG_FATAL, "unknown patch type");
    }
    os << "\"\n"
       << "    validRect=\"" << _validRect.x << " " << _validRect.y << " "
       << _validRect.width << " " << _validRect.height << "\"\n"
       << "    channel=\"" << _validChannel << "\"\n"
       << "    size=\"" << _template->width << " " << _template->height << "\">\n";
    
    for (unsigned y = 0; y < (unsigned)_template->height; y++) {
      os << "   ";
      for (unsigned x = 0; x < (unsigned)_template->width; x++) {
	if (_versionNum == 0 || _versionNum == 2) {
	  os << " " << (int)(CV_IMAGE_ELEM(_template, uchar, y, x));
	} else {
	  os << " " << CV_IMAGE_ELEM(_template, float, y, x);
        }        
      }
      os << "\n";
    }
    
    os << "  </PatchDefinition>\n";

    return true;
}

bool svlPatchDefinition::isCorrelatedWith(const svlPatchDefinition *patch, double threshold) const {
  // checks if they correspond to the same region within the image; if they don't,
  // then they're definitely not correlated

  // assumes that both are valid entries, and thus are non-uniform
  // also assumes that they're of the same type

  if (!isSameType(patch)) {
    //cerr << "not same type" << endl;
    return false;
  }

  if (_template->nChannels != patch->_template->nChannels) {
    return false;
  }

  int a_width = _template->width;
  int b_width = patch->_template->width;

  int a_height = _template->height;
  int b_height = patch->_template->height;

  //cerr << a_width << "x" << a_height << ", " << b_width << "x" << b_height << ": ";

  // if the templates come from completely different areas of the image,
  // they're obviously not correlated
  // ("completely different" is defined as even if the two patches
  // are positioned with as much overlap as possible within the valid
  // rectangle, they still overlap by less than min(w1, w2) x min(h1, h2))
  if (!enoughOverlap(_validRect, a_width, a_height,
		     patch->_validRect, b_width, b_height)) {
    //cerr << "not enough overlap" << endl;
    return false;
  }

  double value = 0;

  if (a_width <= b_width && a_height <= b_height) {
    // patch->_template strictly bigger
    //cerr << "patch strictly bigger, ";
    value = correlationResponse(patch->_template, _template);
  
  } else if (b_width <= a_width && b_height <= a_height) {
    // template strictly bigger
    //cerr << "templ strictly bigger, ";
    value = correlationResponse(_template, patch->_template);
  
  } else if (a_width < b_width) {
    // patch is bigger in width, but template is bigger in height
    //cerr << "patch bigger in w, ";
    value = correlationResponseWithExpansion(patch->_template, _template);
  
  } else if (b_width < a_width) {
    // template is strictly bigger in width
    //cerr << "patch bigger in h, ";
    value = correlationResponseWithExpansion(_template, patch->_template);
  
  } else {
    SVL_ASSERT(false); // debugging to make sure all cases are considered
  }

  //cerr << value << endl;
  return (value > threshold);
}

IplImage *svlPatchDefinition::responseImage(const IplImage *image,
					    const IplImage *imageSum,
					    const IplImage *imageSumSq) const
{
  SVL_ASSERT_MSG(image->nChannels == 1, "responseImage not implemented yet for multiple channels");

  static int handle = svlCodeProfiler::getHandle("svlPatchDefinition::responseImage");
  svlCodeProfiler::tic(handle);

  SVL_ASSERT((_template != NULL));
  SVL_ASSERT((image != NULL));
  
  int width = image->width - _template->width +1;
  int height = image->height - _template->height+1;
  
  IplImage *response = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, 1);
  if (_versionNum == 2) {
    matchTemplateCoeffn(image, _template, response, imageSum, imageSumSq);
  } else if (_versionNum == 1) { 
    cvMatchTemplate(image, _template, response, CV_TM_CCORR);
  } else if (_versionNum == 0) {
    responseImageOrig(image, _template, response, imageSum, imageSumSq);
    //cvMatchTemplate(image, _template, response, CV_TM_CCOEFF_NORMED);
  } else {
    SVL_LOG(SVL_LOG_FATAL, "Unsupported svlPatchDefinition versionNum " << _versionNum);
  }

  svlCodeProfiler::toc(handle);
  return response;
}

// same as setting the ROI of the image and then calling cvMinMaxLoc but thread-safe
static float svlLocalMax(const IplImage *image, CvRect roi)
{
  SVL_ASSERT(image->depth == IPL_DEPTH_32F);
  SVL_ASSERT(image->nChannels == 1);
  float result = CV_IMAGE_ELEM(image, float, roi.y, roi.x);
  for (int y = roi.y; y < roi.y + roi.height; y++) {
    float *ptr = &CV_IMAGE_ELEM(image, float, y, roi.x);
    float *end_ptr = &CV_IMAGE_ELEM(image, float, y, roi.x + roi.width);
    for (; ptr < end_ptr; ++ptr) {
      if (*ptr > result)
	result = *ptr;
    }
  }
  return result;
}
//#define DEBUG

double svlPatchDefinition::patchValueHelperVersion1(const IplImage *image,
						    const CvPoint& location, float normConstant,
						    const IplImage *responseImage,
						    const IplImage *imageSum,
						    const IplImage *imageSumSq) const
{
  bool optimized = responseImage && imageSum && imageSumSq;
  SVL_ASSERT(optimized || image);
  
  if (!optimized)
    SVL_ASSERT(image->depth == IPL_DEPTH_32F);
  
  SVL_ASSERT(_template->depth == IPL_DEPTH_32F);
  
  int startX = location.x + _validRect.x;
  int startY = location.y + _validRect.y;
  int endX = startX + _validRect.width;
  int endY = startY + _validRect.height;
  
  int N = _template->width * _template->height;
  float normConstantSq = normConstant * normConstant;
  float NnormConstantSq = N * normConstantSq;
  float normConstant2 = 2 * normConstant;
  float wTemplateSum = normConstant * _templateSum;
  
  const double *ps = NULL;
  const double *ps2 = NULL;
  const double *psSq = NULL;
  const double *psSq2 = NULL;
  const float *psR = NULL;
  if (optimized) {
    ps = &CV_IMAGE_ELEM(imageSum, double, startY, 0);
    ps2 = &CV_IMAGE_ELEM(imageSum, double, startY + _template->height, 0);
    psSq = &CV_IMAGE_ELEM(imageSumSq, double, startY, 0);
    psSq2 = &CV_IMAGE_ELEM(imageSumSq, double, startY + _template->height, 0);
    psR = &CV_IMAGE_ELEM(responseImage, float, startY, 0);
  }
    
    float windowSum = 0;
    float windowSumSq = 0;
    float ccorr = 0;
    
    // iterate over the _validRect region
    float maxValSq = -numeric_limits<float>::max();
    for (int y = startY; y < endY; ++y) {
      for (int x = startX; x < endX; ++x) {
	if (optimized) {
	  ccorr = psR[x];
	  windowSum = (float)(ps[x] + ps2[x + _template->width] - 
			      ps[x + _template->width] - ps2[x]);
	  windowSumSq = (float)(psSq[x] + psSq2[x + _template->width] - 
				psSq[x + _template->width] - psSq2[x]);
	} 
	
	// not optimized, everything has to be computed from scratch
	else {
	  const float *pw = &CV_IMAGE_ELEM(image, float, y, x);
	  const float *pp = (const float *)_template->imageData;
	  ccorr = windowSum = windowSumSq = 0.0;
	  for (int v = 0; v < _template->height; v++) {
	    for (int u = 0; u < _template->width; u++) {
	      windowSum += pw[u];
	      windowSumSq += pw[u] * pw[u];
	      ccorr += pp[u] * pw[u];
	    }
	    pp += (_template->widthStep / sizeof(float));
	    pw += (image->widthStep / sizeof(float));
	  }
	}
	
	// the sum over the template: sqrt(\sum_{x',y'} (I(x+x', y+y') - normConstant)^2)
	float windowNormSq = windowSumSq - normConstant2 * windowSum + NnormConstantSq;
	
	// the cross-correlation of _template with (I - normConstant)
	// (wTemplateSum = weightedTemplateSum = _templateSum * normConstant)
	float normedCcorr = (ccorr - wTemplateSum);
	
	// for efficiency, will work with the square of everything
	float normedCcorrSq = (normedCcorr > 0) ? 
	  (normedCcorr * normedCcorr) : 
	  (-1 * normedCcorr * normedCcorr);
	
	// ... normalized 
	// (at the end of the loop will also be normalized by _templateNormSq)
	if (normedCcorrSq > maxValSq * windowNormSq)
	  maxValSq = normedCcorrSq / windowNormSq;
	
	if (isinf(normedCcorr) || isnan(normedCcorr) || isnan(ccorr) || isinf(ccorr) ||
	    isnan(windowNormSq)) { // || abs(maxValSq) > _templateNormSq) {
	  cerr << "windowSum: " << windowSum << ", windowSumSq: " << windowSumSq << ", normConstant: " << normConstant << ", N: " << N << endl;
	  cerr << "ccorr: " << ccorr << ", windowNormSq: " << windowNormSq << ", normedCcorr: " << normedCcorr << ", _templateNormSq: " << _templateNormSq << endl;
	  cerr << "_templateSum: " << _templateSum << endl;
	  if (optimized) {
	    cerr << "ps values: " << ps[x] << " " << ps[x+_template->width] << " " << ps2[x] << " " << ps2[x+_template->width] << endl;
	    cerr << "ps2 values: " << psSq[x] << " " << psSq[x+_template->width] << " " << psSq2[x] << " " << psSq2[x+_template->width] << endl;
	  } else {
	    cerr << "Full patch (32x32):" << endl;
	    for (int j = 0; j < 32; j++) {
	      cerr << "\t";
	      for (int i = 0; i < 32; i++) {
		cerr << CV_IMAGE_ELEM(image, float, location.y+j, location.x+i) << " ";
	      }
	      cerr << endl;
	    }
	    
	    cerr << "Patch:" << endl;
	    for (int j = 0; j < _template->height; j++) {
	      cerr << "\t";
	      for (int i = 0; i < _template->width; i++) {
		cerr << CV_IMAGE_ELEM(image, float, y+j, x+i) << " ";
	      }
	      cerr << endl;
	    }
	  }
	  cerr << "Template:" << endl;
	  for (int j = 0; j < _template->height; j++) {
	    cerr << "\t";
	    for (int i = 0; i < _template->width; i++) {
	      cerr << CV_IMAGE_ELEM(_template, float, j, i) << " ";
	    }
	    cerr << endl;
	  }
	  cerr << "Cross-correlation:" << endl;
	  const float *pw = &CV_IMAGE_ELEM(image, float, y, x);
	  const float *pp = (const float *)_template->imageData;
	  for (int v = 0; v < _template->height; v++) {
	    cerr << "\t";
	    for (int u = 0; u < _template->width; u++) {
	      cerr << pp[u] * pw[u] << " ";
	    }
	    pp += (_template->widthStep / sizeof(float));
	    pw += (image->widthStep / sizeof(float));
	    cerr << endl;
	  }
	} 
	
      } // end x iter
      
      if (optimized) {
        ps += imageSum->widthStep / sizeof(double);
        ps2 += imageSum->widthStep / sizeof(double);
        psSq += imageSumSq->widthStep / sizeof(double);
        psSq2 += imageSumSq->widthStep / sizeof(double);
        psR += responseImage->widthStep / sizeof(float);
      }
      
    } // end y iter
    
    maxValSq /= _templateNormSq;
    
    // take care to zero out any invalid values (will occur e.g. if the patch is uniform)
    if (isnan(maxValSq) || isinf(maxValSq)) return 0;
    return (maxValSq > 0) ? sqrt(maxValSq) : -1 * sqrt(-1 * maxValSq);


}

double svlPatchDefinition::patchValueHelperVersion0(const IplImage *image,
						    const CvPoint& location,
						    const IplImage *responseImage,
						    const IplImage *imageSum,
						    const IplImage *imageSumSq) const
{
  if (responseImage) {
    SVL_ASSERT_MSG(responseImage->depth == IPL_DEPTH_32F,"in svlPatchDefinition::patchValue fn");
    SVL_ASSERT_MSG(responseImage->nChannels == 1, "in svlPatchDefinition::patchValue fn");

    int startX = location.x + _validRect.x;
    int startY = location.y + _validRect.y;
      
    CvRect region = cvRect(startX, startY, _validRect.width, _validRect.height);

    double max_value = svlLocalMax(responseImage, region);    

    // know normalized cross-correlation is never greater than 1
    if (max_value > 1.0) {
      //SVL_LOG(SVL_LOG_WARNING, "patchValue: responseImage value is " << max_value << ", setting to 1");
      return 1.0;
    } else if (max_value < -1.0) {
      //SVL_LOG(SVL_LOG_WARNING, "patchValue: responseImage value is " << max_value << ", setting to -1");
      return -1.0;
    }

    return max_value;
  } else {
    SVL_ASSERT(image->depth == IPL_DEPTH_8U);

    // compute patch statistics
    float patchSum = _templateSum;
    float patchNorm = _templateNormSq;
    float N = (float)(_template->width * _template->height);
    float patchMean = patchSum / N;
    patchNorm -= patchSum * patchMean;

    float maxVal = -numeric_limits<float>::max();
    for (int dy = _validRect.y; dy < _validRect.y + _validRect.height; dy++) {
      for (int dx = _validRect.x; dx < _validRect.x + _validRect.width; dx++) {
	const unsigned char *pw = &CV_IMAGE_ELEM(image, unsigned char, location.y + dy, location.x + dx);
	const unsigned char *pp = (const unsigned char *)_template->imageData;
	unsigned int windowSum = 0;
	unsigned int windowNorm = 0;
	unsigned int val = 0;
	for (int v = 0; v < _template->height; v++) {
	  for (int u = 0; u < _template->width; u++) {
	    windowSum += pw[u];
	    windowNorm += pw[u] * pw[u];
	    val += pp[u] * pw[u];
	  }
	  pp += _template->widthStep;
	  pw += image->widthStep;
	}
	
	float fWindowNorm = (float)windowNorm - (float)(windowSum * windowSum) / N;
	float fVal = (float)val - patchMean * (float)windowSum;
	float denom = sqrt(patchNorm * fWindowNorm);

	if (denom == 0) {
	  // this generates a ton of warninigs
	  //SVL_LOG(SVL_LOG_WARNING, "patchValues: region is uniform, setting to value to 0");
	  fVal = 0;
	}
	else {
	  fVal /= denom;
	}	    	  
	
	if (fVal > maxVal && !isnan(fVal) && !isinf(fVal)) {
	  maxVal = fVal;
	}
      }
    }

    SVL_ASSERT(maxVal != -numeric_limits<float>::max());
	
    return (double)maxVal;
  }
}

// Combines all the logic into one private function; either the responseImage and
// integral images are precomputed and the optimization is easy, or they aren't
// and the original image is passed in instead so everything happens on the spot
double svlPatchDefinition::patchValueHelper(IplImage *image,
					    const CvPoint& location, float normConstant,
					    const IplImage *responseImage,
					    const IplImage *imageSum,
					    const IplImage *imageSumSq) const 
{
  if (_versionNum == 2) {
    if (responseImage) {
      // area of the convolution response
      int startX = location.x + _validRect.x;
      int startY = location.y + _validRect.y;
      
      CvRect region = cvRect(startX, startY, _validRect.width, _validRect.height);

      return  svlLocalMax(responseImage, region);
    } else {
      CvRect region = cvRect(location.x + _validRect.x,
			     location.y + _validRect.y,
			     _validRect.width + _template->width-1,
			     _validRect.height + _template->height-1);
      cvSetImageROI(image, region);
      
      int width = _validRect.width; //image->width - _template->width +1;
      int height = _validRect.height; //image->height - _template->height+1;
      IplImage *response = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, 1);
      
      matchTemplateCoeffn(image, _template, response, NULL, NULL);
      cvResetImageROI(image);

      double min, max;
      cvMinMaxLoc(response, &min, &max);
      cvReleaseImage(&response);
      
      return max;
    }
  } else if (_versionNum == 1) {
    return patchValueHelperVersion1(image, location, normConstant, responseImage, imageSum, imageSumSq);    
  } else if (_versionNum == 0) {
    return patchValueHelperVersion0(image, location, responseImage, imageSum, imageSumSq);
  } else {
    SVL_LOG(SVL_LOG_FATAL, "Unsupported svlPatchDefinition versionNum " << _versionNum);
    // to suppress warnings
    return 0.0;
  }
}

double svlPatchDefinition::patchValue(const CvPoint& location, float normConstant,
				      const IplImage *responseImage,
				      const IplImage *imageSum, const IplImage *imageSumSq) const 
{
  return patchValueHelper(NULL, location, normConstant, responseImage, imageSum, imageSumSq);
}

double svlPatchDefinition::patchValue(const IplImage *image, const CvPoint& location, float normConstant) const
{
  IplImage *buffer = cvCloneImage(image);
  double result = patchValueHelper(buffer, location, normConstant, NULL, NULL, NULL);
  cvReleaseImage(&buffer);
  return result;
}

vector<double> svlPatchDefinition::patchValues(const IplImage *image,
					       const vector<CvPoint>& locations,
					       const vector<float>& normConstants) const
{
  vector<double> values;
  values.reserve(locations.size());

  IplImage *buffer = cvCloneImage(image); // to enforce const image
  
  for (unsigned i = 0; i < locations.size(); i++) {
    values.push_back(patchValueHelper(buffer, locations[i], normConstants[i], NULL, NULL, NULL));
  }

  cvReleaseImage(&buffer);
  return values;
}









// svlIntensityPatchDefinition class --------------------------------------------------

svlIntensityPatchDefinition::svlIntensityPatchDefinition() :
    svlPatchDefinition()
{
    // do nothing
}

svlIntensityPatchDefinition::svlIntensityPatchDefinition(XMLNode& node, int versionNum) :
  svlPatchDefinition(node, versionNum)
{
    // do nothing
	read(node);
}

svlIntensityPatchDefinition::svlIntensityPatchDefinition(const IplImage *t, const CvRect& rect, int channel,
							 int versionNum) :
  svlPatchDefinition(t, rect, channel, versionNum)
{
    // do nothing

}

svlIntensityPatchDefinition::~svlIntensityPatchDefinition()
{
    // do nothing
}










// svlDepthPatchDefinition class --------------------------------------------------

svlDepthPatchDefinition::svlDepthPatchDefinition() :
  svlPatchDefinition()
{
    // do nothing
}

svlDepthPatchDefinition::svlDepthPatchDefinition(XMLNode& node, int versionNum) :
  svlPatchDefinition(node, versionNum)
{
    // do nothing
	read(node);
}

svlDepthPatchDefinition::svlDepthPatchDefinition(const IplImage *t, const CvRect& rect, int channel,
						 int versionNum) :
  svlPatchDefinition(t, rect, channel, versionNum)
{
    // do nothing
}

svlDepthPatchDefinition::~svlDepthPatchDefinition()
{
    // do nothing
}



// static function definitions ----------------------------------------------------

static void responseImageOrig(const IplImage* image, const IplImage* t,
			      IplImage* response, const IplImage* imageSum,
			      const IplImage* imageSumSq) 
{
  SVL_ASSERT_MSG(image->nChannels == 1,
		 "in svlPatchDefinition::responseImageOrig actually " << image->nChannels << " channels");
  SVL_ASSERT_MSG(imageSum->nChannels == 1,
		 "in svlPatchDefinition::responseImageOrig fn");
  SVL_ASSERT_MSG(imageSumSq->nChannels == 1,
		 "in svlPatchDefinition::responseImageOrig fn");
  SVL_ASSERT_MSG(response->width == image->width - t->width + 1, 
		 "in svlPatchDefinition::responseImageOrig fn");
  SVL_ASSERT_MSG(response->height == image->height - t->height + 1, 
		 "in svlPatchDefinition::responseImageOrig fn");
  SVL_ASSERT_MSG(response->depth == IPL_DEPTH_32F, 
		 "in svlPatchDefinition::responseImageOrig fn");

  SVL_ASSERT((t != NULL) && (image != NULL) && (imageSum != NULL) && (imageSumSq != NULL));

  float patchSum = 0.0;
  float patchNorm = 0.0;
  unsigned char *pp = (unsigned char *)t->imageData;
  for (int y = 0; y < t->height; y++) {
    for (int x = 0; x < t->width; x++) {
      patchSum += (float)pp[x];
      patchNorm += (float)(pp[x] * pp[x]);
    }
    pp += t->widthStep;
  }
  float N = (float)(t->width * t->height);
  float patchMean = patchSum / N;
  patchNorm -= patchSum * patchMean; 
  
  IplImage *convolution_response = cvCreateImage(cvSize(image->width - t->width + 1,
							image->height - t->height + 1), IPL_DEPTH_32F, 1);
  cvMatchTemplate(image, t, convolution_response, CV_TM_CCORR);

  for (int y = 0; y < image->height - t->height + 1; y++) {
    const unsigned char *p = &CV_IMAGE_ELEM(image, unsigned char, y, 0);
    const double *ps = &CV_IMAGE_ELEM(imageSum, double, y, 0);
    const double *ps2 = &CV_IMAGE_ELEM(imageSum, double, y + t->height, 0);
    const double *psSq = &CV_IMAGE_ELEM(imageSumSq, double, y, 0);
    const double *psSq2 = &CV_IMAGE_ELEM(imageSumSq, double, y + t->height, 0);
    float *q = &CV_IMAGE_ELEM(response, float, y, 0);
    for (int x = 0; x < image->width - t->width + 1; x++, p++) {
      float windowSum = (float)(ps[x] + ps2[x + t->width] - 
				ps[x + t->width] - ps2[x]);
      float windowNorm = (float)(psSq[x] + psSq2[x + t->width] - 
				 psSq[x + t->width] - psSq2[x]);

      float convolutionSum = CV_IMAGE_ELEM(convolution_response, float, y, x);
      windowNorm -= windowSum * windowSum / N;

      float numerator = (float)convolutionSum - patchMean * windowSum;
      float denominator = sqrt(patchNorm * windowNorm);
      
      if (denominator == 0) {
	// these give tons of warnings :'(
	//	SVL_LOG(SVL_LOG_WARNING, "responseImage involves division by 0, setting to value to 0 [(num, denom): (" 
	//<< numerator << ", " << denominator << ")");
	*q = 0;
      }
      else {
	*q = numerator / denominator;
      }
      
      // know that the normalized cross-correlation is never greater than 1
      if (*q > 1.0) {
	//SVL_LOG(SVL_LOG_WARNING, "responseImage value is " << *q << ", setting to 1 [(num, denom): (" 
	//		<< numerator << ", " << denominator << ")");
	*q = 1.0;
      } else if (*q < -1.0) {
      //SVL_LOG(SVL_LOG_WARNING, "responseImage value is " << *q << ", setting to -1 [(num, denom): ("
	//<< numerator << ", " << denominator << ")");
	*q = -1.0;
      }
      
      q++;
    }
  }
  
  cvReleaseImage(&convolution_response);
}

static void matchTemplateCoeffnSq(const IplImage* img, const IplImage* t,
				  IplImage* response, const IplImage* intImg,
				  const IplImage* intImgSqr) {
   CvRect roi = cvGetImageROI(img);
   CvRect troi = cvGetImageROI(t);
   if (roi.width - t->width + 1 != response->width ||
       roi.height - t->height + 1 != response->height) {
     SVL_ASSERT_MSG(false, "Mismatched template, image, response sizes.");
   }
   if (troi.width != t->width || troi.height != t->height) {
     SVL_LOG(SVL_LOG_WARNING, "Template has ROI set but it will be ignored.");
   }

   // compute cross correlation for entire image.
   cvMatchTemplate(img, t, response, CV_TM_CCORR);

   // compute patch normalization values
   double tMean, tVar, tVarTimesN, tVarSq;
   CvScalar tMean_, tVar_;
   cvAvgSdv(t, &tMean_, &tVar_);
   double N = t->width*t->height;
   tMean = tMean_.val[0]; tVar = N*tVar_.val[0]*tVar_.val[0];
   tVarTimesN = tVar * N; tVarSq = tVar * tVar;

   // Compute integral images if necessary.
   CvPoint intImgOffset, intImgSqrOffset;
   IplImage *myIntImg = 0, *myIntImgSqr = 0;
   unsigned intImgStride, intImgSqrStride;
   if (!intImg || !intImgSqr) { // if no intImgSqr, OpenCV computes intImg anyway...
     myIntImg = cvCreateImage(cvSize(roi.width+1, roi.height+1), IPL_DEPTH_64F, 1);
     intImg = myIntImg;
   }
   if (!intImgSqr) {
     myIntImgSqr = cvCreateImage(cvSize(roi.width+1, roi.height+1), IPL_DEPTH_64F, 1);
     intImgSqr = myIntImgSqr;
   }
   if (myIntImg) {
     cvIntegral(img, myIntImg, myIntImgSqr);
   }
   // init integral image params
   CvRect intImgRoi = cvGetImageROI(intImg);
   CvRect intImgSqrRoi = cvGetImageROI(intImgSqr);
   intImgOffset.x = intImgRoi.x - roi.x;
   intImgOffset.y = intImgRoi.y - roi.y;
   intImgSqrOffset.x = intImgSqrRoi.x - roi.x;
   intImgSqrOffset.y = intImgSqrRoi.y - roi.y;
   intImgStride = intImg->widthStep / sizeof(double);
   intImgSqrStride = intImgSqr->widthStep / sizeof(double);

   // check integral image constraints
   if (intImgRoi.width != roi.width + 1 ||
       intImgRoi.height != roi.height + 1 ||
       intImgSqrRoi.width != roi.width + 1 ||
       intImgSqrRoi.height != roi.height + 1) {
     SVL_ASSERT_MSG(false, "An integral image has the wrong size.");
   }

   // compute normalization for response image.
   for (int y = roi.y; y < roi.y + roi.height - t->height + 1; y++) {
     float* resp_row = &CV_IMAGE_ELEM(response, float, y - roi.y, 0);

     for (int x = roi.x; x < roi.x + roi.width - t->width + 1; x++) {
       // compute sum and sumSq for image from integrals
       double imgSum, imgSumSq;

       const double* imgSumPtr = &CV_IMAGE_ELEM(intImg, double, y + intImgOffset.y, x + intImgOffset.x);
       imgSum = imgSumPtr[0] - imgSumPtr[t->height*intImgStride] - imgSumPtr[t->width] +
         imgSumPtr[t->height*intImgStride + t->width];
       const double* imgSumSqrPtr = &CV_IMAGE_ELEM(intImgSqr, double,
                                                   y + intImgSqrOffset.y, x + intImgSqrOffset.x);
       imgSumSq = imgSumSqrPtr[0] - imgSumSqrPtr[t->height*intImgSqrStride]
         - imgSumSqrPtr[t->width] + imgSumSqrPtr[t->height*intImgSqrStride + t->width];

       // now compute normalized response
       double imgVarTimesN = imgSumSq * N - imgSum * imgSum;
       float numerator = (resp_row[x - roi.x] - tMean * imgSum);
       float numeratorSq = (numerator > 0) ? numerator * numerator : -1 * numerator * numerator;
       if (imgVarTimesN > tVarTimesN)
	 resp_row[x - roi.x] =  numeratorSq / (tVar * imgVarTimesN/N);
       else
	 resp_row[x - roi.x] = numeratorSq / tVarSq;
       //double imgVar = std::max(imgSumSq - imgSum * imgSum / N, tVar);
       //resp_row[x - roi.x] = (resp_row[x - roi.x] - tMean * imgSum) / sqrt(tVar * imgVar);
     }
   }

   cvReleaseImage(&myIntImg);
   cvReleaseImage(&myIntImgSqr);
}


static void matchTemplateCoeffn(const IplImage* img, const IplImage* t,
				IplImage* response, const IplImage* intImg,
				const IplImage* intImgSqr) {
   CvRect roi = cvGetImageROI(img);
   CvRect troi = cvGetImageROI(t);
   if (roi.width - t->width + 1 != response->width ||
       roi.height - t->height + 1 != response->height) {
     SVL_ASSERT_MSG(false, "Mismatched template, image, response sizes.");
   }
   if (troi.width != t->width || troi.height != t->height) {
     SVL_LOG(SVL_LOG_WARNING, "Template has ROI set but it will be ignored.");
   }

   // compute cross correlation for entire image.
   cvMatchTemplate(img, t, response, CV_TM_CCORR);

   // compute patch normalization values
   double tMean, tVar, tVarTimesN;
   CvScalar tMean_, tVar_;
   cvAvgSdv(t, &tMean_, &tVar_);
   double N = t->width*t->height;
   tMean = tMean_.val[0]; tVar = N*tVar_.val[0]*tVar_.val[0]; tVarTimesN = tVar * N;

   // Compute integral images if necessary.
   CvPoint intImgOffset, intImgSqrOffset;
   IplImage *myIntImg = 0, *myIntImgSqr = 0;
   unsigned intImgStride, intImgSqrStride;
   if (!intImg || !intImgSqr) { // if no intImgSqr, OpenCV computes intImg anyway...
     myIntImg = cvCreateImage(cvSize(roi.width+1, roi.height+1), IPL_DEPTH_64F, 1);
     intImg = myIntImg;
   }
   if (!intImgSqr) {
     myIntImgSqr = cvCreateImage(cvSize(roi.width+1, roi.height+1), IPL_DEPTH_64F, 1);
     intImgSqr = myIntImgSqr;
   }
   if (myIntImg) {
     cvIntegral(img, myIntImg, myIntImgSqr);
   }
   // init integral image params
   CvRect intImgRoi = cvGetImageROI(intImg);
   CvRect intImgSqrRoi = cvGetImageROI(intImgSqr);
   intImgOffset.x = intImgRoi.x - roi.x;
   intImgOffset.y = intImgRoi.y - roi.y;
   intImgSqrOffset.x = intImgSqrRoi.x - roi.x;
   intImgSqrOffset.y = intImgSqrRoi.y - roi.y;
   intImgStride = intImg->widthStep / sizeof(double);
   intImgSqrStride = intImgSqr->widthStep / sizeof(double);

   // check integral image constraints
   if (intImgRoi.width != roi.width + 1 ||
       intImgRoi.height != roi.height + 1 ||
       intImgSqrRoi.width != roi.width + 1 ||
       intImgSqrRoi.height != roi.height + 1) {
     SVL_ASSERT_MSG(false, "An integral image has the wrong size.");
   }

   // compute normalization for response image.
   for (int y = roi.y; y < roi.y + roi.height - t->height + 1; y++) {
     float* resp_row = &CV_IMAGE_ELEM(response, float, y - roi.y, 0);

     for (int x = roi.x; x < roi.x + roi.width - t->width + 1; x++) {
       // compute sum and sumSq for image from integrals
       double imgSum, imgSumSq;

       const double* imgSumPtr = &CV_IMAGE_ELEM(intImg, double, y + intImgOffset.y, x + intImgOffset.x);
       imgSum = imgSumPtr[0] - imgSumPtr[t->height*intImgStride] - imgSumPtr[t->width] +
         imgSumPtr[t->height*intImgStride + t->width];
       const double* imgSumSqrPtr = &CV_IMAGE_ELEM(intImgSqr, double,
                                                   y + intImgSqrOffset.y, x + intImgSqrOffset.x);
       imgSumSq = imgSumSqrPtr[0] - imgSumSqrPtr[t->height*intImgSqrStride]
         - imgSumSqrPtr[t->width] + imgSumSqrPtr[t->height*intImgSqrStride + t->width];

       // now compute normalized response
       double imgVarTimesN = imgSumSq * N - imgSum * imgSum;
       if (imgVarTimesN > tVarTimesN)
	 resp_row[x - roi.x] = (resp_row[x - roi.x] - tMean * imgSum) / sqrt(tVar * imgVarTimesN/N);
       else
	 resp_row[x - roi.x] = (resp_row[x - roi.x] - tMean * imgSum) / tVar;
       //double imgVar = std::max(imgSumSq - imgSum * imgSum / N, tVar);
       //resp_row[x - roi.x] = (resp_row[x - roi.x] - tMean * imgSum) / sqrt(tVar * imgVar);
     }
   }

   cvReleaseImage(&myIntImg);
   cvReleaseImage(&myIntImgSqr);
}



static double correlationResponse(const IplImage *img, const IplImage *tmpl) {
  IplImage *response = cvCreateImage(cvSize(img->width - tmpl->width + 1,
					    img->height - tmpl->height + 1),
				     IPL_DEPTH_32F, 1);
  // do not subtract the mean of each patch, just do
  // normalized cross-correlation (so expansion doesn't
  // affect it negatively)
  cvMatchTemplate(img, tmpl, response, CV_TM_CCORR_NORMED);  

  double min_value, max_value;
  cvMinMaxLoc(response, &min_value, &max_value);

  cvReleaseImage(&response);
  return max_value;
}

static double correlationResponseWithExpansion(const IplImage *imBiggerWidth, const IplImage *imBiggerHeight) {

  int w_small = imBiggerHeight->width;
  int w_large = imBiggerWidth->width;

  int h_small = imBiggerWidth->height;
  int h_large = imBiggerHeight->height;

  // first, expand in height
  IplImage *expanded = cvCreateImage(cvSize(w_large, h_large * 2 - h_small),
				     imBiggerWidth->depth, imBiggerWidth->nChannels);

  cvZero(expanded);
  cvSetImageROI(expanded, cvRect(0, h_large - h_small, w_large, h_small));
  cvCopy(imBiggerWidth, expanded);
  cvResetImageROI(expanded);

  double val1 = correlationResponse(expanded, imBiggerHeight);
  cvReleaseImage(&expanded);

  // now expand in width
  expanded = cvCreateImage(cvSize(w_large * 2 - w_small, h_large),
			   imBiggerHeight->depth, imBiggerHeight->nChannels);
  cvZero(expanded);
  cvSetImageROI(expanded, cvRect(w_large - w_small, 0, w_small, h_large));
  cvCopy(imBiggerHeight, expanded);
  cvResetImageROI(expanded);

  double val2 = correlationResponse(expanded, imBiggerWidth);
  cvReleaseImage(&expanded);

  //cerr << " (" << val1 << ", " << val2 << ")";

  return max(val1, val2);
}

static bool enoughOverlap(CvRect rect1, int w1, int h1, CvRect rect2, int w2, int h2) {
  // coordinates of the overlap area
  int x1 = max(rect1.x, rect2.x);
  int y1 = max(rect1.y, rect2.y);
  int x2 = min(rect1.x + rect1.width + w1, rect2.x + rect2.width + w2);
  int y2 = min(rect1.y + rect1.height + h1, rect2.y + rect2.height + h2);

  return ((x2 - x1) >= min(w1, w2)) && ((y2-y1) >= min(h1, h2));
}


