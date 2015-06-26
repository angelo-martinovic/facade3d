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
** FILENAME:    svlMultiSeg.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
**
*****************************************************************************/

#include <string>
#include <vector>
#include <set>

#include "cv.h"
#include "cxcore.h"

#include "svlBase.h"
#include "svlMultiSeg.h"
#include "svlVisionUtils.h"
#include "svlOpenCVUtils.h"

using namespace std;

// svlImageSegmentation ------------------------------------------------------

svlImageSegmentation::svlImageSegmentation(const CvMat *seg) :
    _seg(NULL), _numSegments(0)
{
    updateSegmentation(seg);
}

svlImageSegmentation::svlImageSegmentation(const char *filename, int w, int h) :
    _seg(NULL), _numSegments(0)
{
    CvMat *s = cvCreateMat(h, w, CV_32SC1);
    SVL_ASSERT(s != NULL);
    bool b = readMatrix(s, filename);
    SVL_ASSERT_MSG(b, "failed to read segmentation from " << filename);

    updateSegmentation(s);
    cvReleaseMat(&s);
}

svlImageSegmentation::svlImageSegmentation(const svlImageSegmentation &imgSeg) :
    _seg(NULL), _numSegments(imgSeg._numSegments),
    _neighbours(imgSeg._neighbours), _nbrPixels(imgSeg._nbrPixels),
    _pixels(imgSeg._pixels)
{
    if (imgSeg._seg != NULL) {
        _seg = cvCloneMat(imgSeg._seg);
    }
}

svlImageSegmentation::~svlImageSegmentation()
{
    if (_seg != NULL)
        cvReleaseMat(&_seg);
}

void svlImageSegmentation::updateSegmentation(const CvMat *seg)
{
    SVL_ASSERT((seg != NULL) || (cvGetElemType(&seg) == CV_32SC1));

    if (_seg != seg) {
        if (_seg != NULL) {
            cvReleaseMat(&_seg);
        }
        _seg = cvCloneMat(seg);
    }

    _numSegments = 0;
    _neighbours.clear();
    _nbrPixels.clear();
    _pixels.clear();

    // re-label segments (0 to _numSegments - 1)
    map<int, int> segMapping;
    for (int y = 0; y < _seg->rows; y++) {
        for (int x = 0; x < _seg->cols; x++) {
            int segId = CV_MAT_ELEM(*_seg, int, y, x);
            map<int, int>::iterator it = segMapping.find(segId);
            if (it == segMapping.end()) {
                it = segMapping.insert(it, make_pair(segId, _numSegments));
                _numSegments += 1;
                _neighbours.push_back(set<int>());
                _nbrPixels.push_back(set<pair<int, int> >());
                _pixels.push_back(vector<CvPoint>());
            }

            // add to pixels
            int newSegId = it->second;
            _pixels[newSegId].push_back(cvPoint(x, y));

            CV_MAT_ELEM(*_seg, int, y, x) = newSegId;
        }
    }

    // update neighborhoods
    for (int y = 0; y < _seg->rows; y++) {
        for (int x = 0; x < _seg->cols; x++) {
            int segId = CV_MAT_ELEM(*_seg, int, y, x);

            // add neighbours (4-connected)
            if ((x > 0) && (CV_MAT_ELEM(*_seg, int, y, x - 1) != segId)) {
                _neighbours[segId].insert(CV_MAT_ELEM(*_seg, int, y, x - 1));
                _nbrPixels[segId].insert(make_pair(x - 1, y));
            }

            if ((x < _seg->cols - 1) && (CV_MAT_ELEM(*_seg, int, y, x + 1) != segId)) {
                _neighbours[segId].insert(CV_MAT_ELEM(*_seg, int, y, x + 1));
                _nbrPixels[segId].insert(make_pair(x + 1, y));
            }

            if ((y > 0) && (CV_MAT_ELEM(*_seg, int, y - 1, x) != segId)) {
                _neighbours[segId].insert(CV_MAT_ELEM(*_seg, int, y - 1, x));
                _nbrPixels[segId].insert(make_pair(x, y - 1));
            }

            if ((y < _seg->rows - 1) && (CV_MAT_ELEM(*_seg, int, y + 1, x) != segId)) {
                _neighbours[segId].insert(CV_MAT_ELEM(*_seg, int, y + 1, x));
                _nbrPixels[segId].insert(make_pair(x, y + 1));
            }
        }
    }
}

// return segment mask (caller must free memory)
CvMat *svlImageSegmentation::segmentMask(int segId) const
{
    SVL_ASSERT(_seg != NULL);
    CvMat *m = cvCreateMat(_seg->rows, _seg->cols, CV_8UC1);

    const int *p = (const int *)CV_MAT_ELEM_PTR(*_seg, 0, 0);
    unsigned char *q = (unsigned char *)CV_MAT_ELEM_PTR(*m, 0, 0);
    for (int i = 0; i < _seg->rows * _seg->cols; i++, p++, q++) {
        *q = (*p == segId) ? 0x01 : 0x00;
    }

    return m;
}

// remove small segments
void svlImageSegmentation::removeSmallSegments(int minSize)
{
    SVL_ASSERT((int)_pixels.size() == _numSegments);
    for (int i = 0; i < _numSegments; i++) {
        if ((int)_pixels[i].size() < minSize) {
            SVL_ASSERT(!_neighbours[i].empty());
            set<int>::const_iterator nit = _neighbours[i].begin();
            int biggestNeighbor = *nit;
            while (++nit != _neighbours[i].end()) {
                if (_pixels[*nit].size() > _pixels[biggestNeighbor].size()) {
                    biggestNeighbor = *nit;
                }
            }

            for (vector<CvPoint>::const_iterator ip = _pixels[i].begin();
                 ip != _pixels[i].end(); ip++) {
                CV_MAT_ELEM(*_seg, int, ip->y, ip->x) = biggestNeighbor;
            }
            _pixels[biggestNeighbor].insert(_pixels[biggestNeighbor].end(),
                _pixels[i].begin(), _pixels[i].end());
            _pixels[i].clear();

            for (set<int>::const_iterator it = _neighbours[i].begin(); it != _neighbours[i].end(); it++) {
                _neighbours[*it].erase(i);
                _neighbours[*it].insert(biggestNeighbor);
            }

            _neighbours[biggestNeighbor].insert(_neighbours[i].begin(),
                _neighbours[i].end());
            _neighbours[i].clear();
        }
    }

    updateSegmentation(_seg);
}

// merge two or more segments
void svlImageSegmentation::mergeSegments(int segIdA, int segIdB)
{
    if (segIdA > segIdB) {
        mergeSegments(segIdB, segIdA);
        return;
    }

    SVL_ASSERT((segIdA >= 0) && (segIdB < _numSegments) && (segIdA != segIdB));

    // renumber segments
    for (int y = 0; y < _seg->rows; y++) {
        for (int x = 0; x < _seg->cols; x++) {
            if (CV_MAT_ELEM(*_seg, int, y, x) == segIdB) {
                CV_MAT_ELEM(*_seg, int, y, x) = segIdA;
            } else if (CV_MAT_ELEM(*_seg, int, y, x) > segIdB) {
                CV_MAT_ELEM(*_seg, int, y, x) -= 1;
            }
        }
    }

    updateSegmentation(_seg);
}

void svlImageSegmentation::mergeSegments(const set<int>& segIds)
{
    if (segIds.size() < 2) return;

    // renumber segments
    for (set<int>::const_iterator it = segIds.begin(); it != segIds.end(); it++) {
        for (vector<CvPoint>::const_iterator ip = _pixels[*it].begin(); ip != _pixels[*it].end(); ip++) {
            CV_MAT_ELEM(*_seg, int, ip->y, ip->x) = _numSegments;
        }
    }

    updateSegmentation(_seg);
}

// finds the set of points at the intersection between three or more regions
set<CvPoint> svlImageSegmentation::findVertices() const
{
    set<CvPoint> v;
    for (int y = 1; y < _seg->rows; y++) {
        for (int x = 1; x < _seg->cols; x++) {
            int segId = CV_MAT_ELEM(*_seg, int, y, x);

            set<int> nbrs;
            if (CV_MAT_ELEM(*_seg, int, y - 1, x) != segId) {
                nbrs.insert(CV_MAT_ELEM(*_seg, int, y - 1, x));
            }
            if (CV_MAT_ELEM(*_seg, int, y, x - 1) != segId) {
                nbrs.insert(CV_MAT_ELEM(*_seg, int, y, x - 1));
            }
            if (CV_MAT_ELEM(*_seg, int, y - 1, x - 1) != segId) {
                nbrs.insert(CV_MAT_ELEM(*_seg, int, y - 1, x - 1));
            }

            if (nbrs.size() > 1) {
                v.insert(cvPoint(x, y));
            }
        }
    }

    return v;
}

// find segments inside (or partially overlapping) with bounding box
set<int> svlImageSegmentation::findSegments(const CvRect &r, double insideRatio) const
{
    // count pixels inside bounding box
    vector<int> insideCount(_numSegments, 0);
    for (int y = r.y; y < std::min(r.y + r.height, _seg->rows); y++) {
        for (int x = r.x; x < std::min(r.x + r.width, _seg->cols); x++) {
            insideCount[CV_MAT_ELEM(*_seg, int, y, x)] += 1;
        }
    }

    // add to set if greater than insideRatio
    set<int> segIds;
    for (int i = 0; i < _numSegments; i++) {
        if (insideCount[i] >= insideRatio * _pixels[i].size()) {
            segIds.insert(i);
        }
    }

    return segIds;
}

CvRect svlImageSegmentation::boundingBox(int segId, double dilate) const
{
    set<int> s;
    s.insert(segId);
    return boundingBox(s, dilate);
}

CvRect svlImageSegmentation::boundingBox(const set<int>& segIds, double dilate) const
{
    if (segIds.empty()) {
        return cvRect(0, 0, 0, 0);
    }

    CvRect r = cvRect(width(), height(), 0, 0);
    for (set<int>::const_iterator it = segIds.begin(); it != segIds.end(); it++) {
        SVL_ASSERT((*it >= 0) && (*it < _numSegments));
        for (vector<CvPoint>::const_iterator p = _pixels[*it].begin(); p != _pixels[*it].end(); ++p) {
            if (p->x < r.x) r.x = p->x;
            if (p->y < r.y) r.y = p->y;
            if (p->x > r.width) r.width = p->x;
            if (p->y > r.height) r.height = p->y;
        }
    }
    
    r.width -= r.x - 1;
    r.height -= r.y - 1;

    // dilate
    r.x = std::max(0, (int)(r.x - 0.5 * dilate * r.width));
    r.y = std::max(0, (int)(r.y - 0.5 * dilate * r.height));
    r.width = std::min(width() - r.x, (int)(r.width * (1.0 + dilate)));
    r.height = std::min(height() - r.y, (int)(r.height * (1.0 + dilate)));

    return r;
}

// tag each segment with highest occuring label
vector<int> svlImageSegmentation::labelSegments(const CvMat *labels) const
{
    SVL_ASSERT((labels != NULL) || (cvGetElemType(&labels) == CV_32SC1));
    SVL_ASSERT((labels->rows == _seg->rows) && (labels->cols == _seg->cols));

    vector<int> segLabels(_numSegments, -1);
    for (int i = 0; i < _numSegments; i++) {
        vector<int> counts;
        for (vector<CvPoint>::const_iterator p = _pixels[i].begin();
             p != _pixels[i].end(); ++p) {

            int lbl = CV_MAT_ELEM(*labels, int, p->y, p->x);
            if (lbl < 0) continue; // ignore negative labels
            if (counts.size() < (unsigned)lbl + 1) {
                counts.resize(lbl + 1, 0);
            }
            counts[lbl] += 1;
        }

        segLabels[i] = argmax<int>(counts);
    }

    return segLabels;
}

// return all segments and combinations thereof which are in the same region
set<set<int> > svlImageSegmentation::segmentPowerSet(const CvMat *labels, int maxSize) const
{
    // get segment partitions
    vector<int> segLabels = labelSegments(labels);

    // find neighbors within partitions
    vector<set<int> > allowedNeighbors(_numSegments, set<int>());
    for (int i = 0; i < _numSegments; i++) {
        for (set<int>::const_iterator it = _neighbours[i].begin();
             it != _neighbours[i].end(); it++) {
            if (segLabels[i] == segLabels[*it]) {
                allowedNeighbors[i].insert(*it);
            }
        }

        SVL_LOG(SVL_LOG_DEBUG, "allowedNeighbors[" << i << "] = "
            << toString(allowedNeighbors[i]) << "; _neighbours[" << i << "] = "
            << toString(_neighbours[i]));
    }

    // add singleton sets
    set<set<int> > powerSets;
    for (int i = 0; i < _numSegments; i++) {
        set<int> s; s.insert(i);
        powerSets.insert(s);
    }

    // iteratively add neighbors
    unsigned powerSetSize = 0;
    int minSetSize = 1;
    while (powerSetSize != powerSets.size()) {
        powerSetSize = powerSets.size();
        SVL_LOG(SVL_LOG_DEBUG, "powerset has " << powerSetSize << " sets of size "
            << minSetSize << " or less");

        set<set<int> > newSets;
        for (set<set<int> >::const_iterator it = powerSets.begin(); it != powerSets.end(); it++) {
            if ((int)it->size() < minSetSize)
                continue;
            for (set<int>::const_iterator jt = it->begin(); jt != it->end(); jt++) {
                for (set<int>::const_iterator kt = allowedNeighbors[*jt].begin();
                     kt != allowedNeighbors[*jt].end(); kt++) {
                    set<int> s(*it);
                    s.insert(*kt);
                    if (s.size() != it->size()) {
                        newSets.insert(s);

                        // break out if too many sets added
                        if ((int)(newSets.size() + powerSets.size()) > maxSize) {
                            powerSets.insert(newSets.begin(), newSets.end());
                            return powerSets;
                        }
                    }
                }
            }
        }

        powerSets.insert(newSets.begin(), newSets.end());
        minSetSize += 1;
    }

    return powerSets;
}

// return random re-segmentation
svlImageSegmentation svlImageSegmentation::randomResegmentation() const
{
    CvMat *labels = cvCreateMat(_seg->rows, _seg->cols, CV_32SC1);
    cvZero(labels);

    svlImageSegmentation newSeg = randomResegmentation(labels);

    cvReleaseMat(&labels);
    return newSeg;
}

svlImageSegmentation svlImageSegmentation::randomResegmentation(const CvMat *labels) const
{
    svlImageSegmentation newSeg(*this);

    // repeated merge a few segments
    for (int i = 0; i < (int)(0.25 * _numSegments); i++) {
        // get segment partitions
        vector<int> segLabels = newSeg.labelSegments(labels);

        // choose two segments to merge
        int segIdA = rand() % newSeg.numSegments();
        if (newSeg._neighbours[segIdA].empty()) {
            continue;
        }
        vector<int> nbrs;
        nbrs.insert(nbrs.end(), newSeg._neighbours[segIdA].begin(),
            newSeg._neighbours[segIdA].end());

        int segIdB = nbrs[rand() % nbrs.size()];

        if (segLabels[segIdA] != segLabels[segIdB]) {
            continue;
        }

        // merge them
        newSeg.mergeSegments(segIdA, segIdB);
    }

    return newSeg;
}

svlImageSegmentation& svlImageSegmentation::operator=(const svlImageSegmentation& imgSeg)
{
    if (imgSeg._seg == _seg) {
        return *this;
    }

    if (_seg != NULL)
        cvReleaseMat(&_seg);

    if (imgSeg._seg != NULL) {
        _seg = cvCloneMat(imgSeg._seg);
    } else {
        _seg = NULL;
    }

    _numSegments = imgSeg._numSegments;
    _neighbours = imgSeg._neighbours;
    _nbrPixels = imgSeg._nbrPixels;
    _pixels = imgSeg._pixels;

    return *this;
}

// svlMultiSeg ---------------------------------------------------------------

svlMultiSeg::svlMultiSeg() :
    _width(0), _height(0)
{
    // do nothing
}

svlMultiSeg::svlMultiSeg(int w, int h) :
    _width(w), _height(h)
{
    // do nothing
}

svlMultiSeg::svlMultiSeg(const svlMultiSeg& multiSeg) :
    _width(multiSeg._width), _height(multiSeg._height), _segs(multiSeg._segs)
{
    // do nothing
}

svlMultiSeg::~svlMultiSeg()
{
    // do nothing
}

void svlMultiSeg::clear()
{
    _segs.clear();
}

void svlMultiSeg::clear(int w, int h)
{
    _width = w;
    _height = h;
    _segs.clear();
}

void svlMultiSeg::addSegmentation(const CvMat *s)
{
    SVL_ASSERT(s != NULL);
    if ((_width == 0) || (_height == 0)) {
        _width = s->cols;
        _height = s->rows;
    }

    SVL_ASSERT((s->rows == height()) && (s->cols == width()));
    _segs.push_back(svlImageSegmentation(s));
}

void svlMultiSeg::addSegmentation(const svlImageSegmentation& s)
{
    SVL_ASSERT((s.height() == height()) && (s.width() == width()));
    _segs.push_back(s);
}

void svlMultiSeg::addSegmentation(const char *filename)
{
    SVL_ASSERT((_width != 0) && (_height != 0));
    _segs.push_back(svlImageSegmentation(filename, width(), height()));
}

void svlMultiSeg::delSegmentation(int i)
{
    SVL_ASSERT((i >= 0) && (i <= count()));
    _segs.erase(_segs.begin() + i);
}

svlImageSegmentation svlMultiSeg::intersection() const
{
    SVL_ASSERT((_width != 0) && (_height != 0));    
    CvMat *s = cvCreateMat(_height, _width, CV_32SC1);
    SVL_ASSERT(s != NULL);
    cvZero(s);

    vector<int> stride(count(), 1);
    for (int i = 0; i < count() - 1; i++) {
        stride[i + 1] = stride[i] * numSegments(i);
    }

    int *p = (int *)CV_MAT_ELEM_PTR(*s, 0, 0);
    for (int y = 0; y < s->rows; y++) {
        for (int x = 0; x < s->cols; x++, p++) {
            for (int i = 0; i < count(); i++) {
                *p += stride[i] * CV_MAT_ELEM(*segmentation(i), int, y, x);
            }
        }
    }

    svlImageSegmentation superSegmentation(s);
    cvReleaseMat(&s);

    return superSegmentation;
}
