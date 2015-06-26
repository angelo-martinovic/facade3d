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
** FILENAME:    svlImageSequence.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
**              Olga Russakovsky <olga@cs.stanford.edu>
**
*****************************************************************************/

#include <cstdlib>
#include <iostream>
#include <fstream>
#include <vector>
#include <cassert>
#include <string>
#include <algorithm>

#include "cv.h"
#include "cxcore.h"
#include "highgui.h"

#if defined(_WIN32)||defined(WIN32)||defined(__WIN32__)
#include "win32/dirent.h"
#else
#include <dirent.h>
#endif

#include "xmlParser/xmlParser.h"

#include "svlBase.h"
#include "svlImageSequence.h"

// svlImageSequence class --------------------------------------------------

svlImageSequence::svlImageSequence() : 
    _directoryName("."), _imageBuffer(NULL)
{
    // do nothing
}

svlImageSequence::svlImageSequence(const svlImageSequence& s) :
  _directoryName(s._directoryName), _imageNames(s._imageNames), _imageBuffer(NULL)
{
    // do nothing
}

svlImageSequence::~svlImageSequence()
{
    if (_imageBuffer != NULL)
	cvReleaseImage(&_imageBuffer);
}

void svlImageSequence::dir(const char *directory, const char *extension)
{
    SVL_ASSERT(directory != NULL);

    clear();

    DIR *dir = opendir(directory);
    SVL_ASSERT(dir != NULL);

    _directoryName = string(directory) + string("/");
    struct dirent *e = readdir(dir);
    while (e != NULL) {	
        if (strstr(e->d_name, extension) != NULL) {
            _imageNames.push_back(string(e->d_name));
        }
        e = readdir(dir);
    }

    // sort image names, so that sequence corresponds to frame index
    sort(_imageNames.begin(), _imageNames.end());

    closedir(dir);
}

void svlImageSequence::load(const char *filename)
{
    SVL_ASSERT(filename != NULL);

    XMLNode root = XMLNode::parseFile(filename, "ImageSequence");
    if (root.isEmpty()) {
        SVL_LOG(SVL_LOG_WARNING, "image sequence " << filename << " is empty");
        return;
    }

    clear();

    if (root.getAttribute("dir") != NULL) {
      _directoryName = string(root.getAttribute("dir"));
      if (_directoryName[_directoryName.length()-1] != '/')
	_directoryName += string("/");
    } else {
        _directoryName = "";
    }

    bool warn = false;
    int numImages = root.nChildNode("Image");
    int imageNodeIndx = 0;
    for (int i = 0; i < numImages; i++) {
	    XMLNode node = root.getChildNode("Image", &imageNodeIndx);
	    string name = node.getAttribute("name");
	    _imageNames.push_back(name);

	    if (node.nChildNode("region") > 0) 
	        warn = true;
    }
    if (warn)
      SVL_LOG(SVL_LOG_WARNING, "regions not suppported within this application");
}

void svlImageSequence::save(const char *filename) const
{
  save(filename, -1);
}

void svlImageSequence::save(const char *filename, const int numFileToSave) const
{
    SVL_ASSERT(filename != NULL);
    ofstream ofs(filename);
    ofs << "<ImageSequence\n"
        << "  dir=\"" << _directoryName << "\"\n"
        << "  version=\"1.0\">\n";
    unsigned totalFile = 0;
    if (numFileToSave > (int)_imageNames.size() || numFileToSave < 0)
      totalFile = _imageNames.size();
    else
      totalFile = numFileToSave;

    for (unsigned i = 0; i < totalFile; i++)
      ofs << "  <Image name=\"" << _imageNames[i] << "\"/>\n";
   
    ofs << "</ImageSequence>\n";
    ofs.close();
}

void svlImageSequence::shuffleImages(unsigned groupSize)
{
  if (_imageNames.size() % groupSize != 0) {
    SVL_LOG(SVL_LOG_WARNING, "Can't shuffle an image sequence in groups of "
	    << groupSize << " when there are " << _imageNames.size()
	    << " images total");
    return;
  }
 
  vector<int> perm(_imageNames.size()/groupSize);
  for (unsigned i = 0; i < perm.size(); i++)
    perm[i] = i*groupSize;
  
  vector<string> names = _imageNames;
  for (unsigned i = 0; i < _imageNames.size(); i++) 
    _imageNames[i] = names[perm[i/groupSize] + i%groupSize];
}

void svlImageSequence::add(const svlImageSequence &s)
{
  if (strWithoutEndSlashes(s._directoryName) != strWithoutEndSlashes(_directoryName)) {
    SVL_LOG(SVL_LOG_WARNING, "Can't merge image sequences corresponding to different directories");
    return;
  }

  // not very efficient
  vector<string>::const_iterator first = _imageNames.begin();
  vector<string>::const_iterator last = _imageNames.end();
  for (unsigned i = 0; i < s._imageNames.size(); i++) {
    string name = s._imageNames[i];
    if (find(first, last, name) == last)
      _imageNames.push_back(name);
  }
}

void svlImageSequence::subtract(const svlImageSequence &s)
{
  if (strWithoutEndSlashes(s._directoryName) != strWithoutEndSlashes(_directoryName)) {
    SVL_LOG(SVL_LOG_WARNING, "Can't subtract image sequences corresponding to different directories");
    return;
  }

  // not very efficient
  for (unsigned i = 0; i < s._imageNames.size(); i++) {
    string name = s._imageNames[i];
    vector<string>::iterator it = find(_imageNames.begin(), _imageNames.end(), name);
    if (it != _imageNames.end())
      _imageNames.erase(it);
  }
}

const IplImage *svlImageSequence::image(unsigned index)
{
    SVL_ASSERT(index < _imageNames.size());
    if (_imageBuffer != NULL)
	cvReleaseImage(&_imageBuffer);
    string filename = _directoryName + _imageNames[index];
    _imageBuffer = cvLoadImage(filename.c_str());

    if (!_imageBuffer)
      {
	SVL_LOG(SVL_LOG_WARNING, "svlImageSequence::image: Unabled to load "<<filename);
      }

#if defined(_WIN32)||defined(WIN32)||defined(__WIN32__)
    cvFlip(_imageBuffer, NULL, 0);
#endif

#if 0
    cvNamedWindow("debug", 1);
    cvShowImage("debug", _imageBuffer);
    cvWaitKey(-1);
    cvDestroyWindow("debug");
#endif

    return _imageBuffer;
}

void svlImageSequence::setDirectory(string directoryName)
{
  int len = directoryName.length();
  if (len > 0 && directoryName[len-1] != '/')
    directoryName.push_back('/');

  _directoryName = directoryName;
}

// svlImageRegionsSequence class ----------------------------------------------------

static const CvRect FULL_ROI = cvRect(0, 0, -1, -1);

svlImageRegionsSequence::svlImageRegionsSequence() :
  _numRegions(0), _fullImageBuffer(NULL), _loadedFilename("")
{
  // do nothing
}

svlImageRegionsSequence::svlImageRegionsSequence(const svlImageRegionsSequence& s) :
  svlImageSequence(s), _imageROIs(s._imageROIs), _numRegionsSoFar(s._numRegionsSoFar),
  _numRegions(s._numRegions), _fullImageBuffer(NULL), _loadedFilename("")
{
    // do nothing
}

svlImageRegionsSequence::svlImageRegionsSequence(const svlImageSequence& s) :
  svlImageSequence(s), _imageROIs(_imageNames.size(), vector<CvRect>(1, FULL_ROI)),
  _fullImageBuffer(NULL), _loadedFilename("")
{
  updateRegionCounts();
}

svlImageRegionsSequence::~svlImageRegionsSequence()
{
    if (_fullImageBuffer != NULL)
	cvReleaseImage(&_fullImageBuffer);
}

void svlImageRegionsSequence::updateRegionCounts()
{
 _numRegionsSoFar.resize(_imageROIs.size());
 unsigned count = 0;
 for (unsigned i = 0; i < _numRegionsSoFar.size(); i++) {
   _numRegionsSoFar[i] = count;
   count += _imageROIs[i].size();
 }

 _numRegions = count;
}

void svlImageRegionsSequence::dir(const char *directory, const char *extension)
{
  clear();
  svlImageSequence::dir(directory, extension);

  _imageROIs.resize(_imageNames.size(), vector<CvRect>(1, FULL_ROI));  
  updateRegionCounts();
}

void svlImageRegionsSequence::load(const char *filename)
{
    SVL_ASSERT(filename != NULL);

    XMLNode root = XMLNode::parseFile(filename, "ImageSequence");
    if (root.isEmpty()) {
        SVL_LOG(SVL_LOG_WARNING, "image sequence " << filename << " is empty");
        return;
    }

    clear();

    if (root.getAttribute("dir") != NULL) {
        _directoryName = string(root.getAttribute("dir")) + string("/");
    } else {
        _directoryName = "";
    }

    int numImages = root.nChildNode("Image");
    _imageNames.resize(numImages);
    _imageROIs.resize(numImages);

    for (int i = 0; i < numImages; i++) {
	XMLNode node = root.getChildNode("Image", i);
	string name = node.getAttribute("name");
	_imageNames[i] = name;

	int n = node.nChildNode("region");
	if (n == 0) {
	  _imageROIs[i].push_back(FULL_ROI);
	}
	for (int j = 0; j < n; j++) {
	  XMLNode region = node.getChildNode("region", j);
	  
	  int x = atoi(region.getAttribute("x"));
	  int y = atoi(region.getAttribute("y"));
	  int w = atoi(region.getAttribute("w"));
	  int h = atoi(region.getAttribute("h"));
	  _imageROIs[i].push_back(cvRect(x, y, w, h));
	}
    }

    updateRegionCounts();
}

void svlImageRegionsSequence::save(const char *filename) const
{
  save(filename, -1);
}

void svlImageRegionsSequence::save(const char *filename, const int numFileToSave) const
{
    SVL_ASSERT(filename != NULL);
    ofstream ofs(filename);
    ofs << "<ImageSequence\n"
        << "  dir=\"" << _directoryName << "\"\n"
        << "  version=\"1.0\">\n";
    unsigned totalFile = 0;
    if (numFileToSave > (int)_numRegions || numFileToSave < 0)
      totalFile = _numRegions;
    else
      totalFile = numFileToSave;

    unsigned numSaved = 0;
    for (unsigned i = 0; i < _imageNames.size() && numSaved < totalFile; i++) {
      if ((_imageROIs[i].size() == 0) ||
	  (_imageROIs[i].size() == 1 && _imageROIs[i][0].width < 0)) {
	ofs << "  <Image name=\"" << _imageNames[i] << "\"/>\n";
	numSaved++;
      } else {
	ofs << "  <Image name=\"" << _imageNames[i] << "\">\n";
	for (unsigned j = 0; j < _imageROIs[i].size() && numSaved < totalFile; j++, numSaved++) {
	  CvRect r = _imageROIs[i][j];
	  ofs << "    <region x=\"" << r.x << "\" y=\"" << r.y
	      << "\" w=\"" << r.width << "\" h=\"" << r.height << "\"/>\n";
	}
	ofs << "  </Image>\n";
      }
    }
    ofs << "</ImageSequence>\n";
    ofs.close();
}

void svlImageRegionsSequence::clear()
{
  _imageNames.clear();
  _imageROIs.clear();
  _numRegionsSoFar.clear();
  _numRegions = 0;
}

void svlImageRegionsSequence::push_back(const string &s) {
  push_back(s, FULL_ROI);
}

void svlImageRegionsSequence::push_back(const string &s, const CvRect &region) {
  _imageNames.push_back(s);
  _imageROIs.push_back(vector<CvRect>(1, region));
  _numRegionsSoFar.push_back(_numRegions++);
}

void svlImageRegionsSequence::push_back(const string &s, const svlObject2dFrame &regions)
{
  SVL_ASSERT(regions.size() > 0);
  _imageNames.push_back(s);
  _imageROIs.push_back(vector<CvRect>());
  _numRegionsSoFar.push_back(_numRegions);

  push_back(_imageNames.size()-1, regions);
}

void svlImageRegionsSequence::push_back(unsigned imgIndex, const CvRect &region)
{
  SVL_ASSERT(imgIndex >= 0 && imgIndex < _imageNames.size());
  _imageROIs[imgIndex].push_back(region);

  for (unsigned j = imgIndex+1; j < _numRegionsSoFar.size(); j++)
    _numRegionsSoFar[j]++;
  _numRegions++;
}

void svlImageRegionsSequence::push_back(unsigned imgIndex, const svlObject2dFrame &regions)
{
  SVL_ASSERT(imgIndex >= 0 && imgIndex < _imageNames.size());
  unsigned currSize = _imageROIs[imgIndex].size();
  unsigned numToAdd = regions.size();

  _imageROIs[imgIndex].reserve(currSize + numToAdd);
  for (unsigned i = 0; i < numToAdd; i++)
    _imageROIs[imgIndex].push_back(regions[i].getRect());
  
  for (unsigned j = imgIndex+1; j < _numRegionsSoFar.size(); j++)
    _numRegionsSoFar[j] += numToAdd;
  _numRegions += numToAdd;
}


pair<unsigned, unsigned> svlImageRegionsSequence::regionIndices(unsigned index) const
{ 
  unsigned image = (unsigned)(upper_bound(_numRegionsSoFar.begin(), _numRegionsSoFar.end(), index) - 
			      _numRegionsSoFar.begin() - 1);
  unsigned region = index - _numRegionsSoFar[image];
  return make_pair<unsigned, unsigned>(image, region);
}

void svlImageRegionsSequence::erase(unsigned index) {
  if (index >= _numRegions) {
    SVL_LOG(SVL_LOG_WARNING, "Provided index is greater than the number of regions, ignoring");
    return;
  }

  pair<unsigned, unsigned> indices = regionIndices(index);
  unsigned im = indices.first;
  _imageROIs[im].erase(_imageROIs[im].begin() + indices.second);

  // if no regions are left for that image, will erase the image
  if (_imageROIs[im].size() == 0) {
    _imageNames.erase(_imageNames.begin() + im);
    _imageROIs.erase(_imageROIs.begin() + im);
    _numRegionsSoFar.erase(_numRegionsSoFar.begin() + im);
    _numRegionsSoFar[im]--;
  }

  for (unsigned i = im+1; i < _numRegionsSoFar.size(); i++)
      _numRegionsSoFar[i]--;
  _numRegions--;
}



void svlImageRegionsSequence::truncate(unsigned index) {
  if (index >= _numRegions) return;

  pair<unsigned, unsigned> indices = regionIndices(index);
  unsigned im = indices.first;
  _imageROIs[im].erase(_imageROIs[im].begin() + indices.second, _imageROIs[im].end());

  unsigned eraseFrom = (indices.second > 0) ? im+1 : im;

  _imageNames.erase(_imageNames.begin() + eraseFrom, _imageNames.end());
  _imageROIs.erase(_imageROIs.begin() + eraseFrom, _imageROIs.end());
  _numRegionsSoFar.erase(_numRegionsSoFar.begin() + eraseFrom, _numRegionsSoFar.end());

  if (eraseFrom == 0) {
    _numRegions = 0;
  } else {
    unsigned lastImage = eraseFrom-1;
    _numRegions = _numRegionsSoFar[lastImage] + _imageROIs[lastImage].size();
  }
}


void svlImageRegionsSequence::truncateWithRemainder(unsigned index, svlImageRegionsSequence &rest) {
  if (index >= _numRegions) return;

  rest.clear();

  pair<unsigned, unsigned> indices = regionIndices(index);
  unsigned im = indices.first;
  
  rest._imageNames.insert(rest._imageNames.begin(), _imageNames.begin() + im, _imageNames.end());
  rest._imageROIs.insert(rest._imageROIs.end(), _imageROIs.begin() + im, _imageROIs.end());

  SVL_ASSERT(rest._imageROIs.size() > 0);

  vector<CvRect> &newROIs = rest._imageROIs[0];
  newROIs.erase(newROIs.begin(), newROIs.begin() + indices.second);
  rest.updateRegionCounts();
  
  truncate(index);
}


void svlImageRegionsSequence::shuffleImages(unsigned groupSize)
{
  if (_imageNames.size() % groupSize != 0) {
    SVL_LOG(SVL_LOG_WARNING, "Can't shuffle an image sequence in groups of "
	    << groupSize << " when there are " << _imageNames.size()
	    << " images total");
    return;
  }
 
  vector<int> perm(_imageNames.size()/groupSize);
  for (unsigned i = 0; i < perm.size(); i++)
    perm[i] = i*groupSize;
  random_shuffle(perm.begin(), perm.end());  

  vector<string> names = _imageNames;
  vector<vector<CvRect> > regions = _imageROIs;
  for (unsigned i = 0; i < _imageNames.size(); i++) {
    _imageNames[i] = names[perm[i/groupSize] + i%groupSize];
    _imageROIs[i] = regions[perm[i/groupSize] + i%groupSize];
  }

  for (unsigned i = 0; i < _imageROIs.size(); i++)
    random_shuffle(_imageROIs[i].begin(), _imageROIs[i].end());

  updateRegionCounts();
}


static inline bool isRectEqual(const CvRect &r1, const CvRect &r2) {
  return (r1.x == r2.x && r1.y == r2.y && r1.width == r2.width && r1.height == r2.height);
}

static bool contains(const vector<CvRect> &regions, CvRect r) {
  for (unsigned i = 0; i < regions.size(); i++)
    if (isRectEqual(regions[i], r)) return true;
  return false;
}

void svlImageRegionsSequence::add(const svlImageRegionsSequence &s)
{
  if (strWithoutEndSlashes(s._directoryName) != strWithoutEndSlashes(_directoryName)) {
    SVL_LOG(SVL_LOG_WARNING, "Can't merge image sequences corresponding to different directories");
    return;
  }

  // not very efficient
  for (unsigned i = 0; i < s._imageNames.size(); i++) {
    string name = s._imageNames[i];
    const vector<CvRect> &regions = s._imageROIs[i];
    unsigned imgIndex = find(_imageNames.begin(), _imageNames.end(), name) -  _imageNames.begin();
    if (imgIndex >= _imageNames.size()) {
      // image name doesn't exist
      _imageNames.push_back(name);
      _imageROIs.push_back(regions);
      _numRegionsSoFar.push_back(_numRegions);
      _numRegions += regions.size();
    } else {
      // image name exists already but maybe not all regions do
      vector<CvRect> &curr_regions = _imageROIs[imgIndex];

      int numAdded = 0;
      for (unsigned j = 0; j < regions.size(); j++) {
	if (!contains(curr_regions, regions[j])) {
	  curr_regions.push_back(regions[j]);
	  numAdded++;
	}
      }

      for (unsigned j = imgIndex+1; j < _numRegionsSoFar.size(); j++)
	_numRegionsSoFar[j] += numAdded;
      _numRegions += numAdded;
    }
  }
}

const IplImage *svlImageRegionsSequence::image(unsigned index)
{
    SVL_ASSERT(index < _numRegions);

    pair<unsigned, unsigned> indices = regionIndices(index);

    string filename = _directoryName + _imageNames[indices.first];
    if (filename != _loadedFilename) {
      // load the image fully if not already loaded
      if (_fullImageBuffer != NULL)
	cvReleaseImage(&_fullImageBuffer);
      _fullImageBuffer = cvLoadImage(filename.c_str());
#if defined(_WIN32)||defined(WIN32)||defined(__WIN32__)
      cvFlip(_fullImageBuffer, NULL, 0);
#endif
      _loadedFilename = filename;
    }

    cvSetImageROI(_fullImageBuffer,_imageROIs[indices.first][indices.second]);
    if (_imageBuffer != NULL)
      cvReleaseImage(&_imageBuffer);
    _imageBuffer = cvCreateImage(cvGetSize(_fullImageBuffer), _fullImageBuffer->depth,
				 _fullImageBuffer->nChannels);
    cvCopy(_fullImageBuffer, _imageBuffer);
    cvResetImageROI(_fullImageBuffer);

#if 0
    cvNamedWindow("debug", 1);
    cvShowImage("debug", _imageBuffer);
    cvWaitKey(-1);
    cvDestroyWindow("debug");
#endif

    return _imageBuffer;
}

pair<string, CvRect> svlImageRegionsSequence::get(unsigned index) const
{
  pair<unsigned, unsigned> indices = regionIndices(index);
  string name = _directoryName + _imageNames[indices.first];
  CvRect region = _imageROIs[indices.first][indices.second];
  return make_pair<string, CvRect>(name, region);
}

////////////////////////////////////




bool hasHomogeneousExtensions(const svlImageSequence & s)
{
  if (s.empty())
    {
      return false;
    }

  string ext = getExtension(s);

  for (unsigned int i = 1; i < s.numImages(); i++)
    {
      string name = s[i];
      size_t lastDot = name.find_last_of(".");
      if (lastDot == string::npos)
	return false;

      string thisExt = name.substr(lastDot);

      if ( strcmp(thisExt.c_str(), ext.c_str()) != 0)
	{
	  return false;
	}
    }

  return true;
}

string getExtension(const svlImageSequence & s)
{
  SVL_ASSERT(! s.empty());

  string first = s[0];
  
  size_t lastDot = first.find_last_of(".");
  
  SVL_ASSERT(lastDot != string::npos);

  return first.substr(lastDot);
}

