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
** FILENAME:    svlPatchDictionary.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
**              Sid Batra <sidbatra@cs.stanford.edu>
**              David Breeden <breeden@cs.stanford.edu>
**              Olga Russakovsky <olga@cs.stanford.edu>
**              Ian Goodfellow <ia3n@cs.stanford.edu>
**
*****************************************************************************/


#include <cassert>
#include <iostream>
#include <fstream>
#include <vector>
#include <list>
#include <limits>
#include <algorithm>
#include "cv.h"
#include "cxcore.h"
#include "highgui.h"

#include "svlBase.h"
#include "svlVision.h"

#define TOTAL_THREADS 8

#if defined(_WIN32)||defined(WIN32)||defined(__WIN32__)
#undef max
#endif

using namespace std;

static float computeNormalizationConstant(const IplImage *img, svlPatchDefinitionType t,
					  CvRect *roi = NULL, const IplImage *imageSum = NULL);

// svlPatchDictionary class --------------------------------------------------


svlPatchDictionary::svlPatchDictionary(unsigned width, unsigned height, int versionNum)
{
    _windowSize = cvSize(width, height);
    _versionNum = versionNum;
    if (_versionNum > 2 || _versionNum < 0) {
      SVL_LOG(SVL_LOG_FATAL, "Unknown version number in patch dictionary: "
	      << _versionNum);
    }
}

svlPatchDictionary::svlPatchDictionary(const svlPatchDictionary& d)
{
  _windowSize = d._windowSize;

  for (unsigned i = 0; i < d._entries.size(); i++) {
    _entries.push_back(d._entries[i]->clone());
  }

  _versionNum = d._versionNum;
}

svlPatchDictionary::~svlPatchDictionary()
{
    // do nothing
}

void svlPatchDictionary::clear()
{
    _entries.clear();
}

void svlPatchDictionary::truncate(unsigned n, bool shuffle)
{
  if (shuffle)
    random_shuffle(_entries.begin(), _entries.end());

  _entries.resize(n);
}

void svlPatchDictionary::filterEntries(const vector<bool> &toKeep)
{
  SVL_ASSERT_MSG(toKeep.size() == _entries.size(), \
		 "The mask passed in must have the same number of entries as are present in the dictionary");

  for(int i = (int)_entries.size() - 1; i >= 0; i--) {
    if (!toKeep[i])
      _entries.erase(_entries.begin() + i);
  }
}

void svlPatchDictionary::alterValidChannel(int newChannel)
{
    for (unsigned i=0 ; i<_entries.size() ; i++)
    {
        _entries[i]->setValidChannel(newChannel);
    }
}

void svlPatchDictionary::mergeDictionary( const svlPatchDictionary &input )
{
    for (unsigned i=0 ; i<input.numFeatures() ; i++)
    {
        _entries.push_back( input._entries[i] );
    }
}

bool svlPatchDictionary::load(const char *filename)
{
  XMLNode root = XMLNode::parseFile(filename);//don't specify the desired
  //root name-- otherwise if there is a patch dictionary embedded in a 
  //a different XML node this will find that node and pretend it was the root
  //We don't want that to happen, because then this method could be passed
  //a composite feature extractor on accident and would not trigger an error
    bool rval = load(root);
    if(!rval)
      SVL_LOG(SVL_LOG_WARNING, "could not read patch dictionary file " << filename);
    return rval;
}

pair<int, int> svlPatchDictionary::readClassifierSize(const char *filename) {
  XMLNode root = XMLNode::parseFile(filename, "PatchDictionary");

  pair<int, int> result;
  result.first = atoi(root.getAttribute("width"));
  result.second = atoi(root.getAttribute("height"));

  return result;
}

bool svlPatchDictionary::load(XMLNode &root)
{

    _entries.clear();

    if (root.isEmpty()) {
        SVL_LOG(SVL_LOG_WARNING, "Patch dictionary file is missing or has incorrect root.");
	return false;
    }

    if (string(root.getName()) != "PatchDictionary" ) {
        SVL_LOG(SVL_LOG_WARNING, "Attempt to read patch dictionary from XMLNode that is not a patch dictionary.");
	return false;
    }

    _windowSize.width = atoi(root.getAttribute("width"));
    _windowSize.height = atoi(root.getAttribute("height"));
    int n = atoi(root.getAttribute("numEntries"));
    
    _versionNum = atoi(root.getAttribute("version"));
    if (_versionNum > 2 || _versionNum < 0) {
      SVL_LOG(SVL_LOG_WARNING, "Unknown version number in patch dictionary: "
	      << _versionNum);
      return false;
    }

    SVL_ASSERT(root.nChildNode("PatchDefinition") == n);
    _entries.resize(n, NULL);

    for (unsigned i = 0; i < _entries.size(); i++) 
    {		
        XMLNode node = root.getChildNode("PatchDefinition", i);
        _entries[i] = svlPatchDefinition::createPatchDefinition(node, _versionNum);
        SVL_ASSERT(_entries[i] != NULL);
    }

    return true;
}

bool svlPatchDictionary::writeOut(ofstream &ofs)
{
    if (ofs.fail()) { return false; }

    ofs << "<PatchDictionary version=\"" << _versionNum << "\"\n"
        << "  width=\"" <<  _windowSize.width << "\"\n"
        << "  height=\"" << _windowSize.height << "\"\n"
        << "  numEntries=\"" << _entries.size() << "\">\n";

    for (unsigned i = 0; i < _entries.size(); i++) {
        _entries[i]->write(ofs);
    }

    ofs << "</PatchDictionary>\n";

    return true;
}

//Will not place rectangle anywhere where mask contains 1s
static void sampleLocationsRandomly(int minWidth, int minHeight,
				    int maxWidth, int maxHeight,
				    int windowWidth, int windowHeight,
				    unsigned n, CvRNG &rngState, 
				    vector<CvRect> &rectangles,
				    CvMat * mask = NULL) {

  if (mask)
    {
      SVL_ASSERT(windowWidth == mask->cols);
      SVL_ASSERT(windowHeight == mask->rows);
    }

  rectangles.clear();
  for (unsigned j = 0; j < n; j++) {
    CvRect r;

    bool accepted = false;;
    int triesRemaining = 10;

    while ( (!accepted) && (triesRemaining > 0) )
      {
	r.width = (cvRandInt(&rngState) % (maxWidth - minWidth)) + minWidth;
	r.height = (cvRandInt(&rngState) % (maxHeight - minHeight)) + minHeight;
	r.x = cvRandInt(&rngState) % (windowWidth - r.width);
	r.y = cvRandInt(&rngState) % (windowHeight - r.height);

	if (mask)
	  {
	    triesRemaining--;

	    accepted = true;

	    for (int y = r.y; accepted && y < r.y + r.height; y++)
	      {
		for (int x = r.x; x < r.x + r.width; x++)
		  {
		    if (cvmGet(mask,y,x) > 0.5)
		      {
			accepted = false;
			break;
		      }
		  }
	      }
	    
	  }
	else
	  {
	    accepted = true;
	  }
      }
    
    if (accepted)
      rectangles.push_back(r);
  } // patches
}


// currently we were assuming that the images for intensity and depth
// patches are greyscale, and they are passed in as such; the channels
// are represented by the vector of images in samples[i].
void svlPatchDictionary::buildDictionary(const vector<vector<IplImage *> >& samples,
					 const vector<svlPatchDefinitionType>& imageTypes, 
					 unsigned n,
					 bool interestPoints, const vector<CvMat *> * masks)
{
    SVL_ASSERT(imageTypes.size() == samples[0].size());

    CvRNG rngState(0xffffffff);

    int numChannels = (int)samples[0].size();    

    unsigned numDepthAdded = 0;
    unsigned numDepthSubtracted = 0;

    for (int c = 0; c < numChannels; c++) {
       for (unsigned i = 0; i < samples.size(); i++) {
	SVL_ASSERT(samples[i].size() == (unsigned)numChannels);
	if (samples[i][c] == NULL)
	  continue;
	SVL_ASSERT((samples[i][c]->width == _windowSize.width) && 
	       (samples[i][c]->height == _windowSize.height));

	IplImage *curSample = samples[i][c];
	if (_versionNum == 1) {
	  curSample = create32Fimage(samples[i][c]);
	  float normalizationConstant = computeNormalizationConstant(curSample, imageTypes[c]);
	  cvSubS(curSample, cvScalar(normalizationConstant), curSample);
	}

	// minimum patch size is MIN(4, _windowSize / 8)
	// maximum patch size is _windowSize / 2
	int minWidth = MIN(4, _windowSize.width / 8);
	int minHeight = MIN(4, _windowSize.height / 8);

	vector<CvRect> rectangles;

	CvMat * mask = NULL;

	if (masks)
	  mask = (*masks)[i];

	sampleLocationsRandomly(minWidth, minHeight,
				_windowSize.width/2, _windowSize.height/2,
				_windowSize.width, _windowSize.height,
				n, rngState, rectangles, mask);


	for (unsigned j = 0; j < rectangles.size(); j++) {
	  CvRect r = rectangles[j];
	  IplImage *t = cvCreateImage(cvSize(r.width, r.height), curSample->depth, curSample->nChannels);
	  cvSetImageROI(curSample, r);
	  cvCopyImage(curSample, t);
	  cvResetImageROI(curSample);
	  
	  // patch is valid over 7x7 pixel region around source location
	  r.x -= 3; r.y -=3; r.width = 7; r.height = 7;

	  svlClipRect(r, _windowSize.width - t->width + 1, 
	        _windowSize.height - t->height + 1);

	  switch (imageTypes[c]) {
	  case SVL_INTENSITY_PATCH:
	    _entries.push_back(new svlIntensityPatchDefinition(t, r, c));
	    break;
	  case SVL_DEPTH_PATCH:
	    _entries.push_back(new svlDepthPatchDefinition(t, r, c));
	    SVL_ASSERT(_entries.back()->patchType() == SVL_DEPTH_PATCH);

	    numDepthAdded++;
	    break;
	  default:
	    SVL_LOG(SVL_LOG_FATAL, "unrecognized channel type");
	  }
	  
	  cvReleaseImage(&t);
	  
	  // check that this will be a valid dictionary entry
	  if (!_entries.back()->isPatchValid()) {
	    if (_entries.back()->patchType() == SVL_DEPTH_PATCH)
	      {
		numDepthSubtracted++;
	      }
	    _entries.erase(_entries.begin() + _entries.size() - 1);
	  }
	} // patches
	if (_versionNum == 1) 
	  cvReleaseImage(&curSample);
      } // images
    } // channel


    if (numDepthAdded)
      {
	if (numDepthSubtracted == numDepthAdded)
	  SVL_LOG(SVL_LOG_WARNING, "All depth patches were rejected!");
      }
  
}

IplImage *svlPatchDictionary::visualizeDictionary() const
{
    int n = (int)ceil(sqrt((double)_entries.size()));
    IplImage *allPatches = cvCreateImage(cvSize(n * _windowSize.width, n * _windowSize.height),
        IPL_DEPTH_8U, 3);
    cvZero(allPatches);

    for (unsigned i = 0; i < _entries.size(); i++) {		
	CvRect r = cvRect((i % n) * _windowSize.width, ((int)(i / n)) * _windowSize.height,
            _windowSize.width, _windowSize.height);
	cvRectangle(allPatches, cvPoint(r.x, r.y), cvPoint(r.x + r.width, r.y + r.height),
            CV_RGB(0, 0, 255), 1);

	cvSetImageROI(allPatches, cvRect(r.x + _entries[i]->_validRect.x, r.y + _entries[i]->_validRect.y,
		_entries[i]->_template->width, _entries[i]->_template->height));
	IplImage *templateCopy = cvCloneImage(_entries[i]->_template);

	if (templateCopy->nChannels != 1) {
	  IplImage *grayTemplateCopy = cvCreateImage(cvGetSize(templateCopy), templateCopy->depth, 1);
	  cvCvtColor(templateCopy, grayTemplateCopy, CV_BGR2GRAY);
	  cvReleaseImage(&templateCopy);
	  templateCopy = grayTemplateCopy;
	}

	scaleToRange(templateCopy, 0.0, 255.0);
        if (templateCopy->depth != IPL_DEPTH_8U) {
            IplImage *tmpImg = cvCreateImage(cvGetSize(templateCopy), IPL_DEPTH_8U, 1);
            cvConvertScale(templateCopy, tmpImg);
            cvReleaseImage(&templateCopy);
            templateCopy = tmpImg;
        }
	for (unsigned k = 0; k < 3; k++) 
	{
	    cvSetImageCOI(allPatches, k + 1);
	    //cvCopyImage(_entries[i]->_template, allPatches);
	    cvCopyImage(templateCopy, allPatches);
	}
	cvReleaseImage(&templateCopy);
	cvSetImageCOI(allPatches, 0);
	cvResetImageROI(allPatches);

	cvRectangle(allPatches, cvPoint(r.x + _entries[i]->_validRect.x, r.y + _entries[i]->_validRect.y),
            cvPoint(r.x + _entries[i]->_validRect.x + _entries[i]->_validRect.width + _entries[i]->_template->width,
                r.y + _entries[i]->_validRect.y + _entries[i]->_validRect.height + _entries[i]->_template->height),
            CV_RGB(255, 0, 0), 1);
    }

    return allPatches;
}

IplImage *svlPatchDictionary::visualizePatch(unsigned index, IplImage *image) const
{
    // create image of the right size (unless one has been passed in)
    if ((image == NULL) || (image->width != _windowSize.width) ||
	(image->height != _windowSize.height) || (image->depth != IPL_DEPTH_8U) ||
	(image->nChannels != 3)) {
	if (image != NULL) {
	    cvReleaseImage(&image);
	}
	image = cvCreateImage(cvSize(_windowSize.width, _windowSize.height),
	    IPL_DEPTH_8U, 3);
    }
    cvZero(image);
    
    // copy template into image
    cvSetImageROI(image, cvRect(_entries[index]->_validRect.x, _entries[index]->_validRect.y,
	    _entries[index]->_template->width, _entries[index]->_template->height));

    IplImage *templateCopy = cvCloneImage(_entries[index]->_template);
    if (templateCopy->nChannels != 1) {
      IplImage *grayTemplateCopy = cvCreateImage(cvGetSize(templateCopy), templateCopy->depth, 1);
      cvCvtColor(templateCopy, grayTemplateCopy, CV_BGR2GRAY);
      cvReleaseImage(&templateCopy);
      templateCopy = grayTemplateCopy;
    }

    scaleToRange(templateCopy, 0.0, 255.0);
    if (templateCopy->depth != IPL_DEPTH_8U) {
        IplImage *tmpImg = cvCreateImage(cvGetSize(templateCopy), IPL_DEPTH_8U, 1);
        cvConvertScale(templateCopy, tmpImg);
        cvReleaseImage(&templateCopy);
        templateCopy = tmpImg;
    }

    for (unsigned k = 0; k < 3; k++) {
	cvSetImageCOI(image, k + 1);
	cvCopyImage(templateCopy, image);
    }
    
    cvReleaseImage(&templateCopy);
    cvSetImageCOI(image, 0);
    cvResetImageROI(image);
    
    // draw box around valid region
    cvRectangle(image, cvPoint(_entries[index]->_validRect.x, _entries[index]->_validRect.y),
	cvPoint(_entries[index]->_validRect.x + _entries[index]->_validRect.width + _entries[index]->_template->width,
	    _entries[index]->_validRect.y + _entries[index]->_validRect.height + _entries[index]->_template->height),
	CV_RGB(255, 0, 0), 1);
    
    // indicate channel index
    for (int j = 0; j <= _entries[index]->_validChannel; j++) {
	CvPoint pt =  cvPoint(_entries[index]->_validRect.x + _entries[index]->_validRect.width + _entries[index]->_template->width,
	    _entries[index]->_validRect.y + _entries[index]->_validRect.height + _entries[index]->_template->height);
	cvLine(image, cvPoint(pt.x - 2 * (j + 1), pt.y - 2), cvPoint(pt.x - 2 * (j + 1), pt.y - 3), CV_RGB(255, 0, 0)); 
    }
    
    return image;
}

// Threading functions -------------------------------------------------------

struct NormalizePatchArgs {
  NormalizePatchArgs(IplImage* const p, _svlPatchDefinitionType t, CvRect r,
		     vector<float> *n, unsigned i, const IplImage *is) : 
    patch(p), type(t), region(r), normConstants(n), index(i), imageSum(is) {}

  IplImage* const patch;
  _svlPatchDefinitionType type;
  CvRect region;
  vector<float> *normConstants;
  unsigned index;
  const IplImage *imageSum;
  
};

void * normalizePatch(void *argsP, unsigned tid) {
  NormalizePatchArgs *args = (NormalizePatchArgs*) argsP;
  
  vector<float> *normConstants = args->normConstants;
  unsigned index = args->index;

  normConstants->at(index) = computeNormalizationConstant(args->patch,
							  args->type,
							  &(args->region),
							  args->imageSum);
  delete args;

  return NULL;
}



struct CalcFVArgs {
  CalcFVArgs(const IplImage *i,
	     const IplImage *sI,
	     const IplImage *sI2,
	     vector<float> *n,
	     const vector<svlSmartPointer<svlPatchDefinition> > *e,
	     const int eI, const vector<CvPoint> *w,
	     vector<vector<double> > *fv, unsigned outputOffset) :
    image(i), sumImage(sI), sumImage2(sI2),
    norms(n), entries(e), entryIndex(eI), windows(w), v(fv), outputOffset(outputOffset) {}
  const IplImage *image;
  const IplImage *sumImage;
  const IplImage *sumImage2;
  const vector<float> *norms;
  const vector<svlSmartPointer<svlPatchDefinition> > *entries;
  const int entryIndex;
  const vector<CvPoint> *windows;
  vector<vector<double> > *v;
  unsigned outputOffset;
};

void *calcFV(void *argP, unsigned tid) 
{
  CalcFVArgs *args = (CalcFVArgs*) argP;

  const int entryIndex = args->entryIndex;
  const svlPatchDefinition *entry = args->entries->at(entryIndex);

  bool sparse = (args->sumImage == NULL || args->sumImage2 == NULL);

  const vector<float> *norms = args->norms;

  // if not sparse, then want to first compute the response image and then call
  // patchValue to extract the highest response within the appropriate neighborhood
  if (!sparse) {
    IplImage *response = entry->responseImage(args->image, args->sumImage, args->sumImage2);

    const vector<CvPoint> *windows = args->windows;

    for (unsigned j = 0; j < windows->size(); j++) {
      args->v->at(j)[entryIndex+args->outputOffset] = 
	entry->patchValue(windows->at(j), norms->at(j), response,
			  args->sumImage, args->sumImage2);
    }
    cvReleaseImage(&response);
  }

  // if sparse, want to do as few computations as possible, in particular avoiding making
  // the entire response image 
  else {
    vector<double> p = entry->patchValues(args->image, *(args->windows), *norms);
      for (unsigned j = 0; j < args->windows->size(); j++) {
	args->v->at(j)[entryIndex + args->outputOffset] = p[j];
      }
  }

  delete args;

  return NULL;
}


void svlPatchDictionary::extract(const vector<CvPoint> &windows, 
				 const vector<svlDataFrame*> &frames,
				 vector< vector<double> > &output,
				 bool sparse,
				 int outputOffset) const 
{
  //Convert list of frames to list of IplImages, using NULL for frames that are not images
  vector<IplImage *> images(frames.size(), NULL);

  for (unsigned i = 0; i < frames.size(); i++)
    {
      SVL_ASSERT(frames[i]);
      svlImageFrame * temp = dynamic_cast<svlImageFrame *>(frames[i]);
      if (temp) {
	if (_versionNum != 1) {
	  images[i] = temp->image;
	} else {
	  // also, convert them to be 32F so can be processed
	  //  by the extract functions
	  images[i] = create32Fimage(temp->image);
	}
      }
    }

  // allocate memory for return vectors
  if (output.size() != windows.size())
    output.resize(windows.size());

  unsigned neededSize = outputOffset + _entries.size();

  for (unsigned i = 0; i < windows.size(); i++)  {
    if (output[i].size() < neededSize)
      output[i].resize(neededSize);
  }

  int numChannels = (int)images.size();

  vector<_svlPatchDefinitionType> channelsToTypes(images.size());

  for (unsigned i = 0; i < _entries.size(); i++)
    {
      if (_entries[i]->_validChannel >= (int)images.size())
	{
	  SVL_LOG(SVL_LOG_FATAL, "Entry "<<i<<" requested channel "<<_entries[i]->_validChannel
		  <<"; only "<<images.size()<<" channels exist");
	}

      channelsToTypes[_entries[i]->validChannel()] = _entries[i]->patchType();
    }

  IplImage **imageSum = NULL;
  IplImage **imageSumSq = NULL;

  // integral images are now always used unless input is sparse
  if (!sparse) {
    // pre-compute integral images for use by patch responses
    imageSum = new IplImage* [numChannels];
    imageSumSq = new IplImage* [numChannels];

    const int hIntegralImages = svlCodeProfiler::getHandle("svlPatchDictionary::Precompute integral images");
    svlCodeProfiler::tic(hIntegralImages);

    for (int c = 0; c < numChannels; c++) {
	imageSum[c] = NULL;
	imageSumSq[c] = NULL;
   
	if (images[c] == NULL)
	    continue; 
	
	imageSum[c] = cvCreateImage(cvSize(images[c]->width + 1, images[c]->height + 1),
				    IPL_DEPTH_64F, images[c]->nChannels);
	imageSumSq[c] = cvCreateImage(cvSize(images[c]->width + 1, images[c]->height + 1),
				      IPL_DEPTH_64F, images[c]->nChannels);
	cvIntegral(images[c], imageSum[c], imageSumSq[c]);
    }
    
    svlCodeProfiler::toc(hIntegralImages);
  }

  // todo: deprecate entirely
  vector<vector<float> > normConstants(images.size());
  for (unsigned i = 0; i < frames.size(); i++) {
    if (_versionNum != 1) {
      normConstants[i].resize(windows.size(), 0.0);
    } else {
      computeNormalizationConstants(images[i], channelsToTypes[i],
				    windows, _windowSize, normConstants[i],
				    sparse ? NULL : imageSum[i]);
    }
  }

  int handle = svlCodeProfiler::getHandle("svlPatchDictionary::calcFV");
  svlCodeProfiler::tic(handle);
  svlThreadPool vThreadPool(TOTAL_THREADS);
  vThreadPool.start();

  for (unsigned i = 0; i < _entries.size(); i++) {
    IplImage *img = images[_entries[i]->_validChannel];
    IplImage *imgSum = NULL;
    IplImage *imgSumSq = NULL;
 
    if (!sparse) {
      imgSum = imageSum[_entries[i]->_validChannel];
      imgSumSq = imageSumSq[_entries[i]->_validChannel];
    }

    vector<float> *norms = &normConstants[_entries[i]->_validChannel];
    CalcFVArgs *args = new CalcFVArgs(img, imgSum, imgSumSq, norms,
				      &_entries, i, &windows, &output, outputOffset);
    vThreadPool.addJob(calcFV, args);
  }

  vThreadPool.finish();
  svlCodeProfiler::toc(handle);

  // Free all memory from integral images

  if (imageSum && imageSumSq) {
    // release integral images
    for (int i = 0; i < numChannels; i++) 
      {
        if (imageSumSq[i] != NULL)
	  cvReleaseImage(&imageSumSq[i]);
        if (imageSum[i] != NULL)
	  cvReleaseImage(&imageSum[i]);
      }
    
    delete[] imageSumSq;
    delete[] imageSum;
  }

  if (_versionNum == 1) {
    for (unsigned i = 0; i < images.size(); i++) {
      if (images[i])
	cvReleaseImage(&images[i]);
    }
  }
}


svlFeatureExtractor * svlPatchDictionary::clone() const
{
  return dynamic_cast<svlFeatureExtractor *>(new svlPatchDictionary(*this));
}


svlFeatureExtractor* svlPatchDictionary::getPrunedExtractor
      (const vector<bool> &featureUsedBits) const
{
  svlPatchDictionary * temp = new svlPatchDictionary(*this);
  SVL_ASSERT(temp);
  temp->filterEntries(featureUsedBits);
  return dynamic_cast<svlFeatureExtractor *>(temp);
}


string svlPatchDictionary::summary() const
{
  unsigned numIntensity = 0;
  unsigned numDepth = 0;

  for (unsigned i = 0; i < _entries.size(); i++)
    {
      svlPatchDefinitionType t = _entries[i]->patchType();
      switch (t)
	{
	case SVL_DEPTH_PATCH:
	  numDepth++;
	  break;
	case SVL_INTENSITY_PATCH:
	  numIntensity++;
	  break;
	default:
	  SVL_LOG(SVL_LOG_FATAL, "svlPatchDictionary::summarize encountered unknown patchtype");
	}
    }

  stringstream s;
  s << "[";

  bool first = true;

  if (numIntensity)
    {
      s << numIntensity << " intensity patches";
      first = false;
    }

  if (numDepth)
    {
      if (first) {
	first = false;
      } else {
	s << ", ";
      }

      s << numDepth << " depth patches";
    }
  s << "]";


  return s.str();
}

/* Helper functions for normalization constant computations */
float computeNormalizationConstant(const IplImage *img, svlPatchDefinitionType t,
				   CvRect *roi, const IplImage *imageSum)
{
    SVL_ASSERT_MSG(img->nChannels == 1, "others not implemented yet");
    SVL_ASSERT((roi == NULL) || ((roi->width > 0) && (roi->height > 0)));

    int startX = 0;
    int startY = 0;
    int endX = img->width;
    int endY = img->height;
    if (roi) {
        startX = roi->x;
        startY = roi->y;
        endX = roi->x + roi->width;
        endY = roi->y + roi->height;
    }
    
    const float *ptr = NULL;
    float sum;
    vector<float> depths;
    
    float constant;

    switch(t) {
    case SVL_INTENSITY_PATCH:
        SVL_ASSERT_MSG(img->depth == IPL_DEPTH_32F, "other depths not implemented");

        if (!imageSum) {
            ptr = &CV_IMAGE_ELEM(img, float, startY, 0);
            sum = 0.0;
            for (int y = startY; y < endY; y++) {
                for (int x = startX; x < endX; x++)
                    sum += ptr[x];
                ptr += (img->widthStep / sizeof(float));
                // SVL_ASSERT(ptr == &CV_IMAGE_ELEM(img, float, y + 1, 0));
            } 
        } else {
            sum = (float)(CV_IMAGE_ELEM(imageSum, double, endY, endX) - 
                CV_IMAGE_ELEM(imageSum, double, endY, startX) - 
                CV_IMAGE_ELEM(imageSum, double, startY, endX) +
                CV_IMAGE_ELEM(imageSum, double, startY, startX));
        }

        constant = sum / ((float)(endX - startX) * (endY - startY));
        if (isnan(constant)) {
            if (roi == NULL) {
                cerr << "NormConstant: over image" << endl;
            } else {
                cerr << "NormConstant: over ROI " << toString(*roi) << endl;
            }
            ptr = &CV_IMAGE_ELEM(img, float, startY, 0);
            for (int y = startY; y < endY; y++) {
                cerr << "\t";
                for (int x = startX; x < endX; x++)
                    cerr << ptr[x] << " ";
                ptr += (img->widthStep / sizeof(float));
                cerr << endl;
            }
        }
    
        return constant;
  
    case SVL_DEPTH_PATCH:
        SVL_ASSERT_MSG(img->depth == IPL_DEPTH_32F, "other depths not implemented");
        
        ptr = &CV_IMAGE_ELEM(img, float, startY, 0);
        for (int y = startY; y < endY; y++) {
            for (int x = startX; x < endX; x++)
                depths.push_back( ptr[x] );
            ptr += (img->widthStep / sizeof(float));
        }  
        SVL_LOG(SVL_LOG_DEBUG, "I am a depth patch!");
        //return median(depths);
	return destructive_median(depths);
        
    default:
        SVL_LOG(SVL_LOG_FATAL, "Normalization on this patch type not implemented yet");
    }
    
    return 0.0; // to suppress warnings
}

void computeNormalizationConstants(IplImage *image, _svlPatchDefinitionType patchType,
				   const vector<CvPoint> &windows,
				   const CvSize &windowSize,
				   vector<float> &normConstants,
				   const IplImage *imageSum) {
  normConstants.clear();
  normConstants.resize(windows.size());
  
  svlThreadPool threadPool(TOTAL_THREADS);
  const int hThreadNormalization = svlCodeProfiler::getHandle("svlPatchDictionary::Normalize patches");
  svlCodeProfiler::tic(hThreadNormalization);
  threadPool.start();
  for (unsigned j = 0; j < windows.size(); j++)  {
    CvRect region = cvRect(windows[j].x, windows[j].y, windowSize.width, windowSize.height);
    NormalizePatchArgs *args = 
      new NormalizePatchArgs(image, patchType, region, &normConstants, j, imageSum);
      threadPool.addJob(normalizePatch, args);
  } 
  threadPool.finish();
  svlCodeProfiler::toc(hThreadNormalization);
}


