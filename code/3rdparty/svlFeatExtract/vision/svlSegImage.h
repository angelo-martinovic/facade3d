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
** FILENAME:    svlSegImage.h
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
** DESCRIPTION:
**  Holds an image and its over segmentation (superpixel description). The
**  segmentation should be numbered from 0 to K-1. Also contains groundtruth
**  labels (evidence) for training and evaluation. Groundtruth labels can be
**  negative, indicating void (i.e. ignore during training and evaluation).
**  The svlSegImageBase class is also used by svlDepthSegImage for an over
**  segmented image with depth (3D) information.
**
**  Images are assumed to be 8-bit colour (3-channel). Matrices (segmentation
**  and labels) are assumed to be 32-bit signed (integers).
*****************************************************************************/

#pragma once

#include <string>
#include <vector>
#include <set>

#include "cv.h"
#include "cxcore.h"

#include "svlBase.h"
#include "svlVisionUtils.h"
#include "svlOpenCVUtils.h"

using namespace std;

// svlSegImageBase -----------------------------------------------------------

class svlSegImageBase {
 protected:
    string _name;                         // base filename for the image
    IplImage *_image;                     // the base RGB image
    CvMat *_segmentation;                 // image segmentation (same size as _image)

    int _numSegments;                     // number of segments
    vector<vector<CvPoint> > _segPixels;  // pixels in each segment
    vector<CvPoint> _segCentroid;         // centroid of each segment
    vector<set<int> > _segNeighbors;      // neighbors for each segment

 public:
    svlSegImageBase(const IplImage *image, const CvMat *seg);
    svlSegImageBase(const char *imgFilename, const char *segFilename);
    svlSegImageBase(const svlSegImageBase& segImg);
    virtual ~svlSegImageBase();

    // returns the base image width and height
    inline int width() const { return _image->width; }
    inline int height() const { return _image->height; }
    // returns the number of segments
    inline int numSegments() const { return _numSegments; }
    // accessor for the image
    inline IplImage *image() const { return _image; }
    inline const CvMat *segments() const { return _segmentation; }
    
    // returns the string identifier for the image
    inline string getName() const { return _name; }
    inline void setName(string& s) { _name = s; }
    inline void setName(const char *s) { _name = string(s); }

    // returns the segment that a given pixel belongs to
    inline int getSegment(int x, int y) const {
	SVL_ASSERT((x >= 0) && (x < width()) && (y >= 0) && (y < height()));
	return CV_MAT_ELEM(*_segmentation, int, y, x);
    }
    inline int getSegment(const CvPoint& p) const {
	return getSegment(p.x, p.y);
    }
    // returns all pixels in a given segment
    inline const vector<CvPoint>& getPixels(int segment) const {
	SVL_ASSERT((segment >= 0) && (segment < _numSegments));
	return _segPixels[segment];
    }
    // returns centroid for a given segment
    inline const CvPoint getCentroid(int segment) const {
	SVL_ASSERT((segment >= 0) && (segment < _numSegments));
        return _segCentroid[segment];
    }
    // returns neighbors for given segment
    inline const set<int>& getNeighbors(int segment) const {
	SVL_ASSERT((segment >= 0) && (segment < _numSegments));
	return _segNeighbors[segment];
    }

    // returns adjacency list (egdes)
    vector<pair<int, int> > getAdjacencyList() const;
    // returns NxN matrix of edge indices (-1 for no edge)
    vector<vector<int> > getAdjacencyMap() const;

    // returns a list of edgels (edge pixels) for each pair of adjacent segments
    vector<vector<CvPoint> > getEdgels() const;

    // returns a list of vertices (intersection of 3 or more segments)
    vector<pair<CvPoint, set<int> > > getVertices() const;

    // debugging and visualization -- caller must free image
    IplImage *visualize() const;
    void colorSegment(IplImage *canvas, int segment, unsigned char grey) const;
    void colorSegment(IplImage *canvas, int segment, unsigned char red,
	unsigned char green, unsigned char blue, double alpha = 1.0) const;
    void colorBoundaries(IplImage *canvas, unsigned char red = 0x00,
	unsigned char green = 0x00, unsigned char blue = 0x00) const;

 protected:
    void computeSegmentData();
};

// svlSegImage ---------------------------------------------------------------

class svlSegImage : public svlSegImageBase {
 public:
    static bool bAllowVoidAsMaxClassLabel;

 protected:
    CvMat *_pixelLabels;                  // image pixel labels (same size as _image)
    vector<int> _segLabels;               // segment labels

 public:
    svlSegImage(const IplImage *image, const CvMat *seg, 
	const CvMat *labels = NULL);
    svlSegImage(const char *imgFilename, const char *segFilename, 
	const char *labelFilename = NULL);
    svlSegImage(const svlSegImage& segImg);
    virtual ~svlSegImage();

    // accessor for the image
    inline CvMat *labels() const { return _pixelLabels; }

    // functions for getting and setting ground truth labels (evidence)
    inline bool hasLabels() const {
	return (!_segLabels.empty());
    }
    void setLabels(const CvMat *labels);
    void setLabels(const vector<int>& labels);
    inline int getLabel(int x, int y) const {
	if (_pixelLabels == NULL) return -1;
	assert((x >= 0) && (x < width()) && (y >= 0) && (y < height()));
	return CV_MAT_ELEM(*_pixelLabels, int, y, x);
    }
    inline int getLabel(const CvPoint& p) const {
	return getLabel(p.x, p.y);
    }
    inline int getLabel(int segment) const {
	assert((segment >= 0) && (segment < _numSegments));	
	return _segLabels[segment];
    }

 protected:
    void computeSegmentLabels();
};


