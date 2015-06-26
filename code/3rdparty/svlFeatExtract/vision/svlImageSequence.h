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
** FILENAME:    svlImageSequence.h
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
**              Ian Goodfellow <ian3@cs.stanford.edu>
**              Olga Russakovsky <olga@cs.stanford.edu>
** DESCRIPTION:
**  Defines an image sequence. The idea is that this can be used as a
**  surrogate video.
**  Defines some operations on image sequences.
**
*****************************************************************************/

#pragma once

#include <vector>
#include <iostream>
#include <string>
#include <algorithm>

#include "svlBase.h"
#include "svlObjectList.h"

using namespace std;

class svlImageSequence {
 public:
    svlImageSequence();
    svlImageSequence(const svlImageSequence& s);
    virtual ~svlImageSequence();

    // Note: these functions now clear the previous contents! Use the
    // new add function if you want to merge two image sequences together
    virtual void dir(const char *directory, const char *extension = ".jpg");
    virtual void load(const char *filename);
    virtual void save(const char *filename, const int numFileToSave) const;
    virtual void save(const char *filename) const;

    virtual void clear() { _imageNames.clear(); }
    bool empty() const { return _imageNames.empty(); }
    virtual unsigned size() const { return _imageNames.size(); }
    unsigned numImages() const { return _imageNames.size(); }
    
    virtual void push_back(const string& s) { _imageNames.push_back(s); }
    void pop_back() { _imageNames.pop_back(); }
    void pop_front() { _imageNames.erase(_imageNames.begin()); }
    string & front() { return _imageNames.front(); }
    string & back() { return _imageNames.back(); }
    string back() const { return _imageNames.back(); }
    virtual void erase(unsigned index) { _imageNames.erase(_imageNames.begin() + index); }
    virtual void truncate(unsigned index) { _imageNames.erase(_imageNames.begin() + index, _imageNames.end()); }
    virtual void shuffleImages(unsigned groupSize = 1);

    // adds the contents of the other image sequence, eliminating duplicates
    virtual void add(const svlImageSequence &s);
    // computes the set difference
    void subtract(const svlImageSequence &s);

    // Returns the i-th image. Calling function should not free the
    // memory.
    virtual const IplImage *image(unsigned index);

    // Returns the directory name.
    const string& directory() const { return _directoryName; }
    void setDirectory(string directoryName);

    // Returns the i-th's image name.
    const string operator[](unsigned i) const { return _imageNames[i]; }
    string& operator[](unsigned i) { return _imageNames[i]; }

 protected:
    string _directoryName;
    vector<string> _imageNames;
    IplImage *_imageBuffer;

};

class svlImageRegionsSequence : public svlImageSequence {
 public:
  svlImageRegionsSequence();
  svlImageRegionsSequence(const svlImageRegionsSequence& s);
  svlImageRegionsSequence(const svlImageSequence& s);
  ~svlImageRegionsSequence();

  void dir(const char *directory, const char *extension = ".jpg");
  void load(const char *filename);
  void save(const char *filename) const;
  void save(const char *filename, const int numFileToSave) const;
  
  void clear();
  unsigned size() const { return _numRegions; }
  
  void push_back(const string& s); // will assume the full area of the image is used
  void push_back(const string& s, const CvRect &region);
  void push_back(const string& s, const svlObject2dFrame &region);
  void push_back(unsigned imageIndex, const CvRect &region);
  void push_back(unsigned imageIndex, const svlObject2dFrame &regions);

  void erase(unsigned index); // index refers to the i^th region now
  void truncate(unsigned index);
  void truncateWithRemainder(unsigned index, svlImageRegionsSequence &rest);

  // will shuffle the image names and randomize the order of regions within
  // them, but will still keep the regions from the same scene together
  void shuffleImages(unsigned groupSize = 1);

  // adds the contents of the other image sequence, eliminating duplicates
  void add(const svlImageRegionsSequence &s);

  // Returns the i-th image region. Calling function should not free the
  // memory.
  const IplImage *image(unsigned index);

  // Returns the *full* image name (ready for loading from a file) and
  // the region of interest (w and h will be -1 if the entire image is
  // meant to be used!)
  pair<string, CvRect> get(unsigned index) const;

 protected:
    vector<vector<CvRect> > _imageROIs;
    vector<unsigned> _numRegionsSoFar;
    unsigned _numRegions;
    IplImage *_fullImageBuffer;
    string _loadedFilename;

 private:
    void updateRegionCounts();
    // first is the image index, second is the regions index
    // within that image
    pair<unsigned, unsigned> regionIndices(unsigned index) const;
};

//Operations on image sequences (Feel free to move or rename these if
//the location offends you, but be aware that the new command line
//system for the object detector apps depends on them)

//Checks that a sequence contains a non-zero number of images, all with the same
//file extension
bool hasHomogeneousExtensions(const svlImageSequence & s);

//Returns the image extension used by the sequence, if it has
//homogenous extensions
//Throws an assert if sequence is empty
//If sequence is non-empty but not homogenous then this extension will not
//be correct for all images... obviously
string getExtension(const svlImageSequence & s);
