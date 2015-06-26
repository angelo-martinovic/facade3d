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
** FILENAME:    svlMultiSeg.h
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
** DESCRIPTION:
**  Holds multiple over-segmentations (superpixel sets) for an image.
**
*****************************************************************************/

#pragma once

#include <string>
#include <vector>
#include <set>

#include "cv.h"
#include "cxcore.h"

#include "svlBase.h"

using namespace std;

// svlImageSegmentation ------------------------------------------------------

class svlImageSegmentation {
 protected:
    CvMat *_seg;                              // segmentation mapping (0 ... _numSegments - 1)
    int _numSegments;                         // number of segments
    vector<set<int> > _neighbours;            // neighbourhood list (segments)
    vector<set<pair<int, int> > > _nbrPixels; // neighbouring pixels
    vector<vector<CvPoint> > _pixels;         // pixels

 public:
    svlImageSegmentation(const CvMat *seg);
    svlImageSegmentation(const char *filename, int w, int h);
    svlImageSegmentation(const svlImageSegmentation &imgSeg);
    virtual ~svlImageSegmentation();

    // accessors
    inline int width() const { return _seg->cols; }
    inline int height() const { return _seg->rows; }
    inline int size() const { return width() * height(); }
    inline int numSegments() const { return _numSegments; }
    inline const CvMat *segmentation() const { return _seg; }

    inline const set<int>& neighbours(int i) const { return _neighbours[i]; }
    inline const vector<CvPoint>& pixels(int i) const { return _pixels[i]; }
    inline const set<pair<int, int> >& neighbourPixels(int i) const { return _nbrPixels[i]; }

    // return segment mask (caller must free memory)
    CvMat *segmentMask(int segId) const;

    // remove small segments
    void removeSmallSegments(int minSize = 10);

    // merge two (or more) segments
    void mergeSegments(int segIdA, int segIdB);
    void mergeSegments(const set<int>& segIds);

    // find the intersection (pixel) between three or more segments
    set<CvPoint> findVertices() const;

    // find segments inside (or partially overlapping) with bounding box
    set<int> findSegments(const CvRect &r, double insideRatio = 1.0) const;
    CvRect boundingBox(int segId, double dilate = 0.0) const;
    CvRect boundingBox(const set<int>& segIds, double dilate = 0.0) const;

    // tag each segment with highest occuring pixel label
    vector<int> labelSegments(const CvMat *labels) const;

    // return all segments and combinations thereof which are in the same region
    set<set<int> > segmentPowerSet(const CvMat *labels, int maxSize = 1000) const;

    // return random re-segmentation
    svlImageSegmentation randomResegmentation() const;
    svlImageSegmentation randomResegmentation(const CvMat *labels) const;

    // operators
    svlImageSegmentation& operator=(const svlImageSegmentation& imgSeg);

 protected:
    void updateSegmentation(const CvMat *seg);
};

// svlMultiSeg ---------------------------------------------------------------

class svlMultiSeg {
 protected:
    int _width;                               // base image width
    int _height;                              // base image height

    vector<svlImageSegmentation> _segs;       // segmentations

 public:
    svlMultiSeg();
    svlMultiSeg(int w, int h);
    svlMultiSeg(const svlMultiSeg& multiSeg);
    virtual ~svlMultiSeg();

    inline int width() const { return _width; }
    inline int height() const { return _height; }
    inline int count() const { return (int)_segs.size(); }

    inline int numSegments(int i) const {
        return _segs[i].numSegments();
    }
    inline const CvMat *segmentation(int i) const {
        return _segs[i].segmentation();
    }
    inline const set<int>& neighbours(int i, int j) const {
        return _segs[i].neighbours(j);
    }
    inline const vector<CvPoint>& pixels(int i, int j) const {
        return _segs[i].pixels(j);
    }
    inline const set<pair<int, int> >& neighbourPixels(int i, int j) const {
        return _segs[i].neighbourPixels(j);
    }

    void clear();
    void clear(int w, int h);
    void addSegmentation(const CvMat *s);
    void addSegmentation(const svlImageSegmentation& s);
    void addSegmentation(const char *filename);
    void delSegmentation(int i);

    // return the segmentation which is an intersection of all other segmentations
    svlImageSegmentation intersection() const;

    const svlImageSegmentation& operator[](unsigned i) const { 
        return _segs[i];
    }
};
