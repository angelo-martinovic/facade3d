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
** FILENAME:    svlSegImage.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
**
*****************************************************************************/

#include <cstdlib>
#include <string>
#include <vector>
#include <set>
#include <map>

#include "cv.h"
#include "cxcore.h"

#include "svlBase.h"
#include "svlSegImage.h"
#include "svlVisionUtils.h"
#include "svlOpenCVUtils.h"

using namespace std;

// svlSegImageBase -----------------------------------------------------------

svlSegImageBase::svlSegImageBase(const IplImage *image, const CvMat *seg) :
    _image(NULL), _segmentation(NULL)
{
    SVL_ASSERT((image != NULL) && (seg != NULL));
    SVL_ASSERT((image->width == seg->cols) && (image->height == seg->rows));
    SVL_ASSERT((image->nChannels == 3) && (image->depth == IPL_DEPTH_8U));
    SVL_ASSERT(CV_MAT_TYPE(seg->type) == CV_32SC1);

    _image = cvCloneImage(image);
    _segmentation = cvCloneMat(seg);

    computeSegmentData();
}
    
svlSegImageBase::svlSegImageBase(const char *imgFilename, const char *segFilename) :
    _image(NULL), _segmentation(NULL)
{
    SVL_ASSERT((imgFilename != NULL) && (segFilename != NULL));
    
    _name = strBaseName(imgFilename);

    SVL_LOG(SVL_LOG_DEBUG, "svlSegImageBase::svlSegImageBase() reading image " 
        << imgFilename << "...");
    _image = cvLoadImage(imgFilename, CV_LOAD_IMAGE_COLOR);
    if (!_image)
      SVL_LOG(SVL_LOG_FATAL, "Could not open "<<imgFilename);

    SVL_LOG(SVL_LOG_DEBUG, "svlSegImageBase::svlSegImageBase() reading segmentation "
        << segFilename << "...");
    _segmentation = cvCreateMat(_image->height, _image->width, CV_32SC1);
    bool success = readMatrix(_segmentation, segFilename);
    SVL_ASSERT(success);

    computeSegmentData();
}

svlSegImageBase::svlSegImageBase(const svlSegImageBase& segImg) :
    _name(segImg._name), _image(NULL), _segmentation(NULL)
{
    _image = cvCloneImage(segImg._image);
    _segmentation = cvCloneMat(segImg._segmentation);
    
    _numSegments = segImg._numSegments;
    _segPixels = segImg._segPixels;
    _segCentroid = segImg._segCentroid;
    _segNeighbors = segImg._segNeighbors;
}

svlSegImageBase::~svlSegImageBase()
{
    cvReleaseMat(&_segmentation);
    cvReleaseImage(&_image);
}

// returns adjacency list (egdes)
vector<pair<int, int> > svlSegImageBase::getAdjacencyList() const
{
    vector<pair<int,int> > edges;
    SVL_ASSERT(_segNeighbors.size() == (unsigned)_numSegments);
    for (int i = 0; i < _numSegments; i++) {
	for (set<int>::const_iterator j = _segNeighbors[i].begin();
	     j != _segNeighbors[i].end(); j++) {
	    // only insert edges once (i < j)
	    if (*j > i) {
		edges.push_back(make_pair(i, *j));
	    }
	}
    }

    sort(edges.begin(), edges.end());
    
    return edges;    
}

// returns NxN matrix of edge indices (-1 for no edge)
vector<vector<int> > svlSegImageBase::getAdjacencyMap() const
{
    vector<vector<int> > edgeMap(_numSegments);
    for (unsigned i = 0; i < (unsigned)_numSegments; i++) {
	edgeMap[i] = vector<int>(_numSegments, -1);
    }

    vector<pair<int,int> > edges = getAdjacencyList();
    for (unsigned i = 0; i < edges.size(); i++) {
	edgeMap[edges[i].first][edges[i].second] = i;
	edgeMap[edges[i].second][edges[i].first] = i;
    }

    return edgeMap;
}

// returns a list of edgels (boundary pixels) for each pair of adjacent segments
vector<vector<CvPoint> > svlSegImageBase::getEdgels() const
{
    vector<vector<CvPoint> > boundaryMap;
    vector<pair<int, int> > adjList = getAdjacencyList();

    boundaryMap.resize(adjList.size());
    map<pair<int, int>, int> boundaryIndx;
    for (int i = 0; i < (int)adjList.size(); i++) {
        boundaryIndx[adjList[i]] = i;
        boundaryIndx[make_pair(adjList[i].second, adjList[i].first)] = i;
    }

    // find neighbors
    for (int y = 0; y < height(); y++) {
	for (int x = 0; x < width(); x++) {
	    int segId = CV_MAT_ELEM(*_segmentation, int, y, x);
            int nbrId;

	    if (x > 0) {
                nbrId = CV_MAT_ELEM(*_segmentation, int, y, x - 1);
                if (nbrId != segId) {
                    int bi = boundaryIndx[make_pair(segId, nbrId)];
                    boundaryMap[bi].push_back(cvPoint(x, y));
                }
            }
            if (y > 0) {
                nbrId = CV_MAT_ELEM(*_segmentation, int, y - 1, x);
                if (nbrId != segId) {
                    int bi = boundaryIndx[make_pair(segId, nbrId)];
                    boundaryMap[bi].push_back(cvPoint(x, y));
                }
            }
	}
    }

    return boundaryMap;
}

// returns a list of vertices (intersection of 3 or more segments)
vector<pair<CvPoint, set<int> > > svlSegImageBase::getVertices() const
{
    // find vertices and index by size (largest first)
    multimap<int, pair<CvPoint, set<int> > > vCandidates;
    for (int y = 1; y < height() - 1; y++) {
        for (int x = 1; x < width() - 1; x++) {
            set<int> segIds;
            for (int i = y - 1; i <= y + 1; i++) {
                for (int j = x - 1; j <= x + 1; j++) {
                    segIds.insert(getSegment(j, i));
                }
            }
            if (segIds.size() >= 3) {
                vCandidates.insert(make_pair(0 - (int)segIds.size(), 
                        make_pair(cvPoint(x, y), segIds)));
            }
        }
    }

    // add vetrices in order of size
    vector<pair<CvPoint, set<int> > > vertexList;
    set<set<int> > found;

    for (multimap<int, pair<CvPoint, set<int> > >::const_iterator it = vCandidates.begin();
         it != vCandidates.end(); ++it) {
        set<int> segIds = it->second.second;
        if (found.find(segIds) == found.end()) {
            vertexList.push_back(make_pair(it->second.first, segIds));
            set<set<int> > ps = powerset(segIds);
            for (set<set<int > >::const_iterator jt = ps.begin(); jt != ps.end(); ++jt) {
                if (jt->size() >= 3) {
                    found.insert(*jt);
                }
            }
        }
    }

    return vertexList;
}

// debugging and visualization -- caller must free image
IplImage *svlSegImageBase::visualize() const
{
    IplImage *debugImage = cvCloneImage(_image);
    colorBoundaries(debugImage, 0x00, 0x00, 0x00);
    
    return debugImage;
}

void svlSegImageBase::colorSegment(IplImage *canvas, int segment, unsigned char grey) const
{
    colorSegment(canvas, segment, grey, grey, grey, 0.0);
}

void svlSegImageBase::colorSegment(IplImage *canvas, int segment, unsigned char red,
    unsigned char green, unsigned char blue, double alpha) const
{
    SVL_ASSERT(canvas != NULL);
    SVL_ASSERT((canvas->width == width()) && (canvas->height == height()));
    SVL_ASSERT((canvas->depth == IPL_DEPTH_8U) && (canvas->nChannels == 3));
    if (alpha < 0.0) alpha = 0.0;
    if (alpha > 1.0) alpha = 1.0;
    
    for (int y = 0; y < height(); y++) {
        unsigned char * const p = (unsigned char *)&canvas->imageData[y * canvas->widthStep];
	for (int x = 0; x < width(); x++) {
	    if (CV_MAT_ELEM(*_segmentation, int, y, x) == segment) {
                p[3 * x + 2] = (unsigned char)((1.0 - alpha) * p[3 * x + 2] + alpha * red);
                p[3 * x + 1] = (unsigned char)((1.0 - alpha) * p[3 * x + 1] + alpha * green);
                p[3 * x + 0] = (unsigned char)((1.0 - alpha) * p[3 * x + 0] + alpha * blue);
	    }
	}
    }
}

void svlSegImageBase::colorBoundaries(IplImage *canvas, unsigned char red,
    unsigned char green, unsigned char blue) const
{
    SVL_ASSERT(canvas != NULL);
    SVL_ASSERT((canvas->width == width()) && (canvas->height == height()));
    SVL_ASSERT((canvas->depth == IPL_DEPTH_8U) && (canvas->nChannels == 3));
    
    for (int y = 1; y < height(); y++) {
	for (int x = 1; x < width(); x++) {
	    if ((CV_MAT_ELEM(*_segmentation, int, y, x) != 
		CV_MAT_ELEM(*_segmentation, int, y - 1, x)) ||
		(CV_MAT_ELEM(*_segmentation, int, y, x) != 
		    CV_MAT_ELEM(*_segmentation, int, y, x - 1))) {

		int baseIndx = y * canvas->widthStep + 3 * x;
		canvas->imageData[baseIndx + 2] = red;
		canvas->imageData[baseIndx + 1] = green;
		canvas->imageData[baseIndx + 0] = blue;		
	    }
	}
    }
}

// private member functions

void svlSegImageBase::computeSegmentData()
{
    // set segment pixels
    _numSegments = 0;
    _segPixels.clear();
    CvPoint p;
    for (p.y = 0; p.y < height(); p.y++) {
	for (p.x = 0; p.x < width(); p.x++) {
	    int segId = CV_MAT_ELEM(*_segmentation, int, p.y, p.x);
	    SVL_ASSERT(segId >= 0);
	    if (_numSegments <= segId) {
		_numSegments = segId + 1;
		_segPixels.resize(_numSegments);
	    }
	    _segPixels[segId].push_back(p);
	}
    }

    // we expect at least two segments
    SVL_ASSERT(_numSegments > 1);

    // compute centroids
    _segCentroid.resize(_numSegments);
    for (int segId = 0; segId < _numSegments; segId++) {
        if (_segPixels[segId].empty()) {
            _segCentroid[segId].x = width() / 2;
            _segCentroid[segId].y = height() / 2;
            continue;
        }

        _segCentroid[segId].x = _segCentroid[segId].y = 0;
        for (unsigned i = 0; i < _segPixels[segId].size(); i++) {
            _segCentroid[segId].x += _segPixels[segId][i].x;
            _segCentroid[segId].y += _segPixels[segId][i].y;
        }
        _segCentroid[segId].x /= _segPixels[segId].size();
        _segCentroid[segId].y /= _segPixels[segId].size();
    }

    // set segment neighbours
    _segNeighbors.clear();
    _segNeighbors.resize(_numSegments);
    for (int y = 0; y < height(); y++) {
	for (int x = 0; x < width(); x++) {
            int segId = CV_MAT_ELEM(*_segmentation, int, y, x);
            int nbrId;
            if (x > 0) {
                nbrId = CV_MAT_ELEM(*_segmentation, int, y, x - 1);
                if (nbrId != segId) {
                    _segNeighbors[segId].insert(nbrId);
                    _segNeighbors[nbrId].insert(segId);
                }
            }
            if (y > 0) {
                nbrId = CV_MAT_ELEM(*_segmentation, int, y - 1, x);
                if (nbrId != segId) {
                    _segNeighbors[segId].insert(nbrId);
                    _segNeighbors[nbrId].insert(segId);
                }
            }
	}
    }
}

// svlSegImage ---------------------------------------------------------------

bool svlSegImage::bAllowVoidAsMaxClassLabel = true;

svlSegImage::svlSegImage(const IplImage *image, const CvMat *seg, 
    const CvMat *labels) : 
    svlSegImageBase(image, seg),  _pixelLabels(NULL)
{
    if (labels != NULL) {
	SVL_ASSERT((labels->rows == _segmentation->rows) &&
	    (labels->cols == _segmentation->cols) &&
	    (CV_MAT_TYPE(labels->type) == CV_32SC1));
	_pixelLabels = cvCloneMat(labels);
    }

    computeSegmentLabels();    
}
    
svlSegImage::svlSegImage(const char *imgFilename, const char *segFilename,
    const char *labelFilename) :
    svlSegImageBase(imgFilename, segFilename), _pixelLabels(NULL)
{
    SVL_LOG(SVL_LOG_DEBUG, "svlSegImage::svlSegImage() reading labels...");
    if (labelFilename != NULL) {
	_pixelLabels = cvCreateMat(_image->height, _image->width, CV_32SC1);
	if (!readMatrix(_pixelLabels, labelFilename)) {
            SVL_LOG(SVL_LOG_WARNING, "failed to read groundtruth file " << labelFilename);
	    cvReleaseMat(&_pixelLabels);
	    _pixelLabels = NULL;
	}
    }

    computeSegmentLabels();
}

svlSegImage::svlSegImage(const svlSegImage& segImg) :
    svlSegImageBase(segImg), _pixelLabels(NULL)
{
    if (segImg._pixelLabels != NULL) {
	_pixelLabels = cvCloneMat(segImg._pixelLabels);
    }

    _segLabels = segImg._segLabels;
}

svlSegImage::~svlSegImage()
{
    if (_pixelLabels != NULL)
	cvReleaseMat(&_pixelLabels);
}

void svlSegImage::setLabels(const CvMat *labels)
{
    if (_pixelLabels)
	cvReleaseMat(&_pixelLabels);

    if (labels == NULL) {
	_pixelLabels = NULL;
    } else {
	SVL_ASSERT((labels->cols == width()) && (labels->rows == height()));
	SVL_ASSERT(CV_MAT_TYPE(labels->type) == CV_32SC1);    
	_pixelLabels = cvCloneMat(labels);
    }
   
    computeSegmentLabels();
}

void svlSegImage::setLabels(const vector<int>& labels)
{
    SVL_ASSERT(labels.size() == (unsigned)_numSegments);
    _segLabels = labels;
    if (_pixelLabels == NULL) {
	_pixelLabels = cvCreateMat(height(), width(), CV_32SC1);
    }
    for (int y = 0; y < height(); y++) {
	for (int x = 0; x < width(); x++) {
	    int seg = CV_MAT_ELEM(*_segmentation, int, y, x);
	    CV_MAT_ELEM(*_pixelLabels, int, y, x) = _segLabels[seg];
	}
    }
}

// private member functions

void svlSegImage::computeSegmentLabels()
{
    if (_pixelLabels == NULL) {
	_segLabels.clear();
	_segLabels.resize(_numSegments, -1);
	return;
    }

    _segLabels.resize(_numSegments, -1);

    map<int, unsigned> counts;
    unsigned maxCount;
    for (int i = 0; i < _numSegments; i++) {
	counts.clear();
	maxCount = 0;
	_segLabels[i] = -1;
	
	for (vector<CvPoint>::const_iterator it = _segPixels[i].begin();
	     it != _segPixels[i].end(); it++) {
	    int lbl = CV_MAT_ELEM(*_pixelLabels, int, it->y, it->x);
	    
	    if (!bAllowVoidAsMaxClassLabel && (lbl < 0))
		continue;

	    if (counts.find(lbl) == counts.end()) {
		counts[lbl] = 0;
	    }
	    counts[lbl] = counts[lbl] + 1;
	    if (counts[lbl] > maxCount) {
		_segLabels[i] = lbl;
		maxCount = counts[lbl];
	    }
	}

	SVL_ASSERT(bAllowVoidAsMaxClassLabel || (maxCount != 0));
    }
}



