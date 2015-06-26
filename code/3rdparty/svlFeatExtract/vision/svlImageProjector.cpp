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
** FILENAME:    svlImageProjector.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
**
*****************************************************************************/

#include <cstdio>
#include <cassert>
#include <vector>
#include <algorithm>
#include <limits>

#include "cv.h"
#include "cxcore.h"
#include "highgui.h"

#include "svlBase.h"
#include "svlVision.h"

#include "svlImageProjector.h"

using namespace std;

const int svlImageProjector::_MAX_CHANNELS = 3;
const int svlImageProjector::_OCCLUSION_WIDTH = 1;

svlImageProjector::svlImageProjector() :
    _image(NULL), _projectedImage(NULL), _imageData(NULL),
    _bImageDataDirty(false), _width(0), _height(0),
    _smoothing(0.0), _alpha(0.0), _pointWidth(0)
{
    for (int i = 0; i < _MAX_CHANNELS; i++) {
	_projectedData[i] = NULL;
    }
}

svlImageProjector::svlImageProjector(const svlCameraIntrinsics& i, const svlCameraExtrinsics& e) :
    _image(NULL), _projectedImage(NULL), _imageData(NULL), 
    _bImageDataDirty(false), _width(0), _height(0),
    _intrinsics(i), _extrinsics(e),
    _smoothing(0.0), _alpha(0.0), _pointWidth(0)
{
    for (int i = 0; i < _MAX_CHANNELS; i++) {
	_projectedData[i] = NULL;
    }
}

svlImageProjector::~svlImageProjector()
{
    freeMemory();
}

void svlImageProjector::setCameraParameters(svlCameraIntrinsics& i, svlCameraExtrinsics& e)
{
    _intrinsics = i;
    _extrinsics = e;
}

void svlImageProjector::clearImage(int width, int height)
{
    SVL_ASSERT((width > 0) && (height > 0));

    if ((_image == NULL) || (_image->width != width) || (_image->height != height)) {
	freeMemory();
	_image = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
    }

    _width = _image->width;
    _height = _image->height;
    cvZero(_image);

    _bImageDataDirty = true;
}

void svlImageProjector::setImage(const IplImage *image)
{
    SVL_ASSERT((image != NULL) && (image->nChannels == 3) && (image->depth == IPL_DEPTH_8U));

    if ((_image != NULL) && (_image->width == image->width) && (_image->height == image->height)) {
	cvCopy(image, _image);
    } else {
	freeMemory();
	_image = cvCloneImage(image);
    }

    _width = _image->width;
    _height = _image->height;
    _bImageDataDirty = true;
}

void svlImageProjector::setImage(const unsigned char *data, int width, int height)
{
    SVL_ASSERT((data != NULL) && (width > 0) && (height > 0));

    if ((_image == NULL) || (_image->width != width) || (_image->height != height)) {
	freeMemory();
	_image = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
    }

    _width = _image->width;
    _height = _image->height;

    for (int y = 0; y < _height; y++) {
	for (int x = 0; x < _width; x++) {
	     _image->imageData[y * _image->widthStep + 3 * x + 2] = data[3 * (y * _width + x) + 0];
	     _image->imageData[y * _image->widthStep + 3 * x + 1] = data[3 * (y * _width + x) + 1];
	     _image->imageData[y * _image->widthStep + 3 * x + 0] = data[3 * (y * _width + x) + 2];
	}
    }

    _bImageDataDirty = true;
}

const IplImage *svlImageProjector::getAsImage(double scale, double shift)
{
    if (_projectedData[0] == NULL) {
	return NULL;
    }

    if ((_projectedImage == NULL) || (_bImageDataDirty)) {
	if (_projectedImage == NULL) {
	    _projectedImage = cvCreateImage(cvSize(_width, _height), IPL_DEPTH_8U, 3);
	}
	for (int y = 0; y < _height; y++) {
	    for (int x = 0; x < _width; x++) {
		for (int k = 0; k < 3; k++) {
		    _projectedImage->imageData[y * _image->widthStep + 3 * x + k] =
			(unsigned char)(scale * (cvmGet(_projectedData[k], y, x) + shift));
		}
	    }
	}

	if (_alpha > 0.0) {
	    cvAddWeighted(_image, _alpha, _projectedImage, 1.0 - _alpha, 0.0, _projectedImage);
	}
    }

    return _projectedImage;
}

const unsigned char *svlImageProjector::getAsImageData(double scale, double shift)
{
    if (_projectedData[0] == NULL) {
	return NULL;
    }

    getAsImage(scale, shift);  // update _projectedImage

    if ((_imageData == NULL) || (_bImageDataDirty)) {
	if (_imageData == NULL) {
	    _imageData = new unsigned char[3 * _width * _height];
	}
	for (int y = 0; y < _height; y++) {
	    for (int x = 0; x < _width; x++) {
		_imageData[3 * (y * _width + x) + 0] = _projectedImage->imageData[y * _projectedImage->widthStep + 3 * x + 2];
		_imageData[3 * (y * _width + x) + 1] = _projectedImage->imageData[y * _projectedImage->widthStep + 3 * x + 1];
		_imageData[3 * (y * _width + x) + 2] = _projectedImage->imageData[y * _projectedImage->widthStep + 3 * x + 0];
	    }
	}
    }

    return _imageData;
}

void svlImageProjector::project(const vector<svlPoint3d>& points,
    const svlPoint3d& offset, double pan, double tilt, double roll)
{
    SVL_ASSERT(_image != NULL);

    CvMat *unitMatrix;
    if (_smoothing > 0.0) {
        unitMatrix = cvCreateMat(_height, _width, CV_32F);
        cvZero(unitMatrix);
    }

    if (_projectedData[0] == NULL) {
        for (int i = 0; i < _MAX_CHANNELS; i++) {
            _projectedData[i] = cvCreateMat(_height, _width, CV_32FC1);
        }
    }

    for (int i = 0; i < _MAX_CHANNELS; i++) {
        cvZero(_projectedData[i]);
    }

    // TO DO: make pan, tilt and roll more efficient
    SVL_ASSERT((tilt == 0.0) && (roll == 0.0));

#if 1
    // insert projected points into array and sort to allow for
    // fast occlusion detection (by depth buffering)
    vector<pair<double, int> > depthBuffer;
    vector<pair<CvPoint, svlPoint3d> > pointBuffer;
    //int nPointsBehind = 0;
    double cos_pan = cos(pan);
    double sin_pan = sin(pan);
    for (unsigned i = 0; i < points.size(); i++) {
        svlPoint3d q = (points[i] - offset).pan(cos_pan, sin_pan);
        CvPoint p = _intrinsics.point(_extrinsics.world2camera(q.x, q.y, q.z));
        // check if point is behind us?
        if (_extrinsics.lastZ() <= 1.0e-3) {
	    //nPointsBehind += 1;
            continue;
        }
        // check if point is in field of view?
        if ((p.x < 0) || (p.x >= _width) || (p.y < 0) || (p.y >= _height)) {
            continue;
        }
	// add to depth buffer
	pointBuffer.push_back(make_pair(p, svlPoint3d(_extrinsics.lastX(),
		    _extrinsics.lastY(), _extrinsics.lastZ())));
	depthBuffer.push_back(make_pair(_extrinsics.lastZ(), (int)depthBuffer.size()));
    }
    sort(depthBuffer.begin(), depthBuffer.end());

    for (unsigned i = 0; i < depthBuffer.size(); i++) {	
	unsigned index = depthBuffer[i].second;
	CvPoint p = pointBuffer[index].first;

	// TO DO [SG]: speed this up (kd-trees?)
	bool bOccluded = false;
	for (int x = p.x - _OCCLUSION_WIDTH; x <= p.x + _OCCLUSION_WIDTH; x++) {
	    if ((x < 0) || (x >= _width)) continue;
	    for (int y = p.y - _OCCLUSION_WIDTH; y <= p.y + _OCCLUSION_WIDTH; y++) {
		if ((y < 0) || (y >= _height)) continue;
		if ((cvmGet(_projectedData[0], y, x) != 0) ||
		    (cvmGet(_projectedData[1], y, x) != 0) ||
		    (cvmGet(_projectedData[2], y, x) != 0)) {
		    bOccluded = true;
		    break;
		}		
	    }
	    if (bOccluded) break;
	}
	if (bOccluded) continue;
	
        cvmSet(_projectedData[0], p.y, p.x, pointBuffer[index].second.x);
        cvmSet(_projectedData[1], p.y, p.x, pointBuffer[index].second.y);
        cvmSet(_projectedData[2], p.y, p.x, pointBuffer[index].second.z);
        if (_smoothing > 0.0) {
            cvmSet(unitMatrix, p.y, p.x, 1.0);
        }
        // TO DO [SG]: accelerate this
        if (_pointWidth > 0) {
            for (int x = p.x - _pointWidth; x <= p.x + _pointWidth; x++) {
                if ((x < 0) || (x >= _width)) continue;
                for (int y = p.y - _pointWidth; y <= p.y + _pointWidth; y++) {
                    if ((y < 0) || (y >= _height)) continue;
                    cvmSet(_projectedData[0], y, x, pointBuffer[index].second.x);
                    cvmSet(_projectedData[1], y, x, pointBuffer[index].second.y);
                    cvmSet(_projectedData[2], y, x, pointBuffer[index].second.z);
		    if (_smoothing > 0.0) {
			cvmSet(unitMatrix, y, x, 1.0);
		    }
                }
            }
        }	
    }
#else 
    int nPointsBehind = 0;
    int nPointsOutOfView = 0;

    // project points
    for (unsigned i = 0; i < points.size(); i++) {
        svlPoint3d q = (points[i] - offset).pan(pan);
        CvPoint p = _intrinsics.point(_extrinsics.world2camera(q.x, q.y, q.z));
        // check if point is behind us?
        if (_extrinsics.lastZ() < 0.0) {
	    nPointsBehind += 1;
            continue;
        }
        // check if point is in field of view?
        if ((p.x < 0) || (p.x >= _width) || (p.y < 0) || (p.y >= _height)) {
	    nPointsOutOfView += 1;
            continue;
        }

        cvmSet(_projectedData[0], p.y, p.x, colours[i].x);
        cvmSet(_projectedData[1], p.y, p.x, colours[i].y);
        cvmSet(_projectedData[2], p.y, p.x, colours[i].z);
        if (_smoothing > 0.0) {
            cvmSet(unitMatrix, p.y, p.x, 1.0);
        }
        // TO DO [SG]: accelerate this
        if (_pointWidth > 0) {
            for (int x = p.x - _pointWidth; x <= p.x + _pointWidth; x++) {
                if ((x < 0) || (x >= _width)) continue;
                for (int y = p.y - _pointWidth; y <= p.y + _pointWidth; y++) {
                    if ((y < 0) || (y >= _height)) continue;
                    cvmSet(_projectedData[0], y, x, colours[i].x);
                    cvmSet(_projectedData[1], y, x, colours[i].y);
                    cvmSet(_projectedData[2], y, x, colours[i].z);
                }
            }
        }
    }

    cerr << points.size() << " total points; "
	 << nPointsBehind << " behind camera; "
	 << nPointsOutOfView << " out of FOV" << endl;
#endif

    if (_smoothing > 0.0) {
        int w = (int)(_smoothing * _width);
        int h = (int)(_smoothing * _height);
        if (w % 2 == 0) w += 1;
        if (h % 2 == 0) h += 1;
        // divide by unit gaussian
        cvSmooth(unitMatrix, unitMatrix, CV_GAUSSIAN, h, w);
        for (int i = 0; i < _MAX_CHANNELS; i++) {
            cvSmooth(_projectedData[i], _projectedData[i], CV_GAUSSIAN, h, w);
            cvDiv(_projectedData[i], unitMatrix, _projectedData[i]);
        }
        cvReleaseMat(&unitMatrix);
    }

    //cerr << nPointsBehind << " of " << points.size() 
    //	 << " were behind camera in svlImageProjector::project()" << endl;
    _bImageDataDirty = true;
}

void svlImageProjector::project(const vector<svlPoint3d>& points, const vector<svlPoint3d>& colours,
    const svlPoint3d& offset, double pan, double tilt, double roll)
{
    SVL_ASSERT(_image != NULL);
    SVL_ASSERT(points.size() == colours.size());

    CvMat *unitMatrix;
    if (_smoothing > 0.0) {
        unitMatrix = cvCreateMat(_height, _width, CV_32F);
        cvZero(unitMatrix);
    }

    if (_projectedData[0] == NULL) {
        for (int i = 0; i < _MAX_CHANNELS; i++) {
            _projectedData[i] = cvCreateMat(_height, _width, CV_32FC1);
        }
    }

    for (int i = 0; i < _MAX_CHANNELS; i++) {
        cvZero(_projectedData[i]);
    }

    // TO DO: make pan, tilt and roll more efficient
    SVL_ASSERT((tilt == 0.0) && (roll == 0.0));

#if 1
    // insert projected points into array and sort to allow for
    // fast occlusion detection (by depth buffering)
    vector<pair<double, int> > depthBuffer;
    vector<pair<CvPoint, svlPoint3d> > pointBuffer;
    //int nPointsBehind = 0;
    for (unsigned i = 0; i < points.size(); i++) {
        svlPoint3d q = (points[i] - offset).pan(pan);
        CvPoint p = _intrinsics.point(_extrinsics.world2camera(q.x, q.y, q.z));
        // check if point is behind us?
        if (_extrinsics.lastZ() < 0.0) {
	    //nPointsBehind += 1;
            continue;
        }
        // check if point is in field of view?
        if ((p.x < 0) || (p.x >= _width) || (p.y < 0) || (p.y >= _height)) {
            continue;
        }
	// add to depth buffer
	pointBuffer.push_back(make_pair(p, colours[i]));
	depthBuffer.push_back(make_pair(_extrinsics.lastZ(), (int)depthBuffer.size()));
    }
    sort(depthBuffer.begin(), depthBuffer.end());

    for (unsigned i = 0; i < depthBuffer.size(); i++) {	
	unsigned index = depthBuffer[i].second;
	CvPoint p = pointBuffer[index].first;

	// TO DO [SG]: speed this up (kd-trees?)
	bool bOccluded = false;
	for (int x = p.x - _OCCLUSION_WIDTH; x <= p.x + _OCCLUSION_WIDTH; x++) {
	    if ((x < 0) || (x >= _width)) continue;
	    for (int y = p.y - _OCCLUSION_WIDTH; y <= p.y + _OCCLUSION_WIDTH; y++) {
		if ((y < 0) || (y >= _height)) continue;
		if ((cvmGet(_projectedData[0], y, x) != 0) ||
		    (cvmGet(_projectedData[1], y, x) != 0) ||
		    (cvmGet(_projectedData[2], y, x) != 0)) {
		    bOccluded = true;
		    break;
		}		
	    }
	    if (bOccluded) break;
	}
	if (bOccluded) continue;
	
        cvmSet(_projectedData[0], p.y, p.x, pointBuffer[index].second.x);
        cvmSet(_projectedData[1], p.y, p.x, pointBuffer[index].second.y);
        cvmSet(_projectedData[2], p.y, p.x, pointBuffer[index].second.z);
        if (_smoothing > 0.0) {
            cvmSet(unitMatrix, p.y, p.x, 1.0);
        }
        // TO DO [SG]: accelerate this
        if (_pointWidth > 0) {
            for (int x = p.x - _pointWidth; x <= p.x + _pointWidth; x++) {
                if ((x < 0) || (x >= _width)) continue;
                for (int y = p.y - _pointWidth; y <= p.y + _pointWidth; y++) {
                    if ((y < 0) || (y >= _height)) continue;
                    cvmSet(_projectedData[0], y, x, pointBuffer[index].second.x);
                    cvmSet(_projectedData[1], y, x, pointBuffer[index].second.y);
                    cvmSet(_projectedData[2], y, x, pointBuffer[index].second.z);
		    if (_smoothing > 0.0) {
			cvmSet(unitMatrix, y, x, 1.0);
		    }
                }
            }
        }	
    }
#else 
    int nPointsBehind = 0;
    int nPointsOutOfView = 0;

    // project points
    for (unsigned i = 0; i < points.size(); i++) {
        svlPoint3d q = (points[i] - offset).pan(pan);
        CvPoint p = _intrinsics.point(_extrinsics.world2camera(q.x, q.y, q.z));
        // check if point is behind us?
        if (_extrinsics.lastZ() < 0.0) {
	    nPointsBehind += 1;
            continue;
        }
        // check if point is in field of view?
        if ((p.x < 0) || (p.x >= _width) || (p.y < 0) || (p.y >= _height)) {
	    nPointsOutOfView += 1;
            continue;
        }

        cvmSet(_projectedData[0], p.y, p.x, colours[i].x);
        cvmSet(_projectedData[1], p.y, p.x, colours[i].y);
        cvmSet(_projectedData[2], p.y, p.x, colours[i].z);
        if (_smoothing > 0.0) {
            cvmSet(unitMatrix, p.y, p.x, 1.0);
        }
        // TO DO [SG]: accelerate this
        if (_pointWidth > 0) {
            for (int x = p.x - _pointWidth; x <= p.x + _pointWidth; x++) {
                if ((x < 0) || (x >= _width)) continue;
                for (int y = p.y - _pointWidth; y <= p.y + _pointWidth; y++) {
                    if ((y < 0) || (y >= _height)) continue;
                    cvmSet(_projectedData[0], y, x, colours[i].x);
                    cvmSet(_projectedData[1], y, x, colours[i].y);
                    cvmSet(_projectedData[2], y, x, colours[i].z);
                }
            }
        }
    }

    cerr << points.size() << " total points; "
	 << nPointsBehind << " behind camera; "
	 << nPointsOutOfView << " out of FOV" << endl;
#endif

    if (_smoothing > 0.0) {
        int w = (int)(_smoothing * _width);
        int h = (int)(_smoothing * _height);
        if (w % 2 == 0) w += 1;
        if (h % 2 == 0) h += 1;
        // divide by unit gaussian
        cvSmooth(unitMatrix, unitMatrix, CV_GAUSSIAN, h, w);
        for (int i = 0; i < _MAX_CHANNELS; i++) {
            cvSmooth(_projectedData[i], _projectedData[i], CV_GAUSSIAN, h, w);
            cvDiv(_projectedData[i], unitMatrix, _projectedData[i]);
        }
        cvReleaseMat(&unitMatrix);
    }

    //cerr << nPointsBehind << " of " << points.size() 
    //	 << " were behind camera in svlImageProjector::project()" << endl;
    _bImageDataDirty = true;
}

// if channel < 0 projects onto all channels
void svlImageProjector::project(const vector<svlPoint3d>& points, const vector<double>& data,
    const svlPoint3d& offset, double pan, double tilt, double roll,
    int channel)
{
    SVL_ASSERT(_image != NULL);
    SVL_ASSERT(points.size() == data.size());
    SVL_ASSERT(channel < _MAX_CHANNELS);

    CvMat *unitMatrix;
    if (_smoothing > 0.0) {
        unitMatrix = cvCreateMat(_height, _width, CV_32F);
        cvZero(unitMatrix);
    }

    if (_projectedData[0] == NULL) {
        for (int i = 0; i < _MAX_CHANNELS; i++) {
            _projectedData[i] = cvCreateMat(_height, _width, CV_32F);
        }
    }

    for (int i = 0; i < _MAX_CHANNELS; i++) {
        cvZero(_projectedData[i]);
    }

    // TO DO: make pan, tilt and roll more efficient
    SVL_ASSERT((tilt == 0.0) && (roll == 0.0));

    for (unsigned i = 0; i < points.size(); i++) {
        svlPoint3d q = (points[i] - offset).pan(pan);
        CvPoint p = _intrinsics.point(_extrinsics.world2camera(q.x, q.y, q.z));
        // check if point is behind us?
        if (_extrinsics.lastZ() < 0.0) {
            continue;
        }
        // check if point is in field of view?
        if ((p.x < 0) || (p.x >= _width) || (p.y < 0) || (p.y >= _height)) {
            continue;
        }

        if (channel < 0) {
            for (int j = 0; j < _MAX_CHANNELS; j++) {
                cvmSet(_projectedData[j], p.y, p.x, data[i]);
            }
        } else {
            cvmSet(_projectedData[channel], p.y, p.x, data[i]);            
        }
        
        if (_smoothing > 0.0) {
            cvmSet(unitMatrix, p.y, p.x, 1.0);
        }

        // TO DO [SG]: accelerate this
        if (_pointWidth > 0) {
            for (int x = p.x - _pointWidth; x <= p.x + _pointWidth; x++) {
                if ((x < 0) || (x >= _width)) continue;
                for (int y = p.y - _pointWidth; y <= p.y + _pointWidth; y++) {
                    if ((y < 0) || (y >= _height)) continue;
                    if (channel < 0) {
                        for (int j = 0; j < _MAX_CHANNELS; j++) {
                            cvmSet(_projectedData[j], y, x, data[i]);
                        }                        
                    } else {
                        cvmSet(_projectedData[channel], y, x, data[i]);
                    }
                }
            }
        }

    }

    if (_smoothing > 0.0) {
        int w = (int)(_smoothing * _width);
        int h = (int)(_smoothing * _height);
        if (w % 2 == 0) w += 1;
        if (h % 2 == 0) h += 1;
        // divide by unit gaussian
        cvSmooth(unitMatrix, unitMatrix, CV_GAUSSIAN, h, w);
        if (channel < 0) {
            for (int i = 0; i < _MAX_CHANNELS; i++) {
                cvSmooth(_projectedData[i], _projectedData[i], CV_GAUSSIAN, h, w);
                cvDiv(_projectedData[i], unitMatrix, _projectedData[i]);
            }
        } else {
            cvSmooth(_projectedData[channel], _projectedData[channel], CV_GAUSSIAN, h, w);
            cvDiv(_projectedData[channel], unitMatrix, _projectedData[channel]);
        }
        cvReleaseMat(&unitMatrix);
    }

    _bImageDataDirty = true;
}

// returns range over entire image
void svlImageProjector::getRange(int channel, double *minVal, double *maxVal) const
{
    getRange(channel, cvRect(0, 0, _width, _height), minVal, maxVal);
}

// returns range over region
void svlImageProjector::getRange(int channel, const CvRect& region, double *minVal, double *maxVal) const
{
    SVL_ASSERT((channel >= 0) && (channel < _MAX_CHANNELS));
    SVL_ASSERT(_projectedData[channel] != NULL);

    double m = numeric_limits<double>::max();
    double M = -numeric_limits<double>::max();
    
    for (unsigned i = region.x; i < (unsigned)(region.x + region.width); i++) {
	for (unsigned j = region.y; j < (unsigned)(region.y + region.height); j++) {
	    if (m > cvmGet(_projectedData[channel], j, i)) {
		m = cvmGet(_projectedData[channel], j, i);
	    }
	    if (M < cvmGet(_projectedData[channel], j, i)) {
		M = cvmGet(_projectedData[channel], j, i);
	    }
	}
    }

    if (minVal != NULL) *minVal = m;
    if (maxVal != NULL) *maxVal = M;
}

void svlImageProjector::scaleToRange(int channel, double minVal, double maxVal)
{
    SVL_ASSERT((channel < _MAX_CHANNELS) && (minVal < maxVal));
    SVL_ASSERT(_projectedData[channel] != NULL);

    if (channel < 0) {
        for (int c = 0; c < _MAX_CHANNELS; c++) {
            scaleToRange(c, minVal, maxVal);
        }
        return;
    }

    double m, M;
    cvMinMaxLoc(_projectedData[channel], &m, &M);
    if (m != M) {
	    cvScale(_projectedData[channel], _projectedData[channel], (maxVal - minVal) / (M - m), minVal - m / (M - m));
    } else {
        cvZero(_projectedData[channel]);
        cvAddS(_projectedData[channel], cvScalar(minVal), _projectedData[channel]);
    }
}

// returns mean and variance over region
vector<double> svlImageProjector::featureStatistics(int channel, const CvRect& region) const
{
    SVL_ASSERT((channel >= 0) && (channel < _MAX_CHANNELS));
    SVL_ASSERT(_projectedData[channel] != NULL);

    vector<double> v(2, 0.0);
    for (unsigned i = region.x; i < (unsigned)(region.x + region.width); i++) {
	for (unsigned j = region.y; j < (unsigned)(region.y + region.height); j++) {
	    v[0] += cvmGet(_projectedData[channel], j, i);
	    v[1] += cvmGet(_projectedData[channel], j, i) * 
		cvmGet(_projectedData[channel], j, i);
	}
    }
    
    v[0] /= (double)(region.width * region.height);
    v[1] = v[1] / (double)(region.width * region.height) - v[0] * v[0];

    return v;
}


// Protected Member Functions ------------------------------------------------------

void svlImageProjector::freeMemory()
{
    if (_image != NULL) {
	cvReleaseImage(&_image);
	_image = NULL;
    }

    if (_projectedImage != NULL) {
	cvReleaseImage(&_projectedImage);
	_projectedImage = NULL;
    }

    for (int i = 0; i < _MAX_CHANNELS; i++) {
	if (_projectedData[i] != NULL) {
	    cvReleaseMat(&_projectedData[i]);
	    _projectedData[i] = NULL;
	}
    }

    if (_imageData != NULL) {
	delete _imageData;
	_imageData = NULL;
    }

    _bImageDataDirty = false;
}

