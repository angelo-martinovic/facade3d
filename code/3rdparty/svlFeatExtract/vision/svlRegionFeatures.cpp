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
** FILENAME:    svlRegionFeatures.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
**
*****************************************************************************/

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <vector>
#include <cassert>

#include "cv.h"
#include "cxcore.h"

#include "svlBase.h"
#include "svlVisionUtils.h"
#include "svlRegionFeatures.h"

using namespace std;

svlRegionFeatures::svlRegionFeatures() :
    _bIncludeStdev(true), _bIncludeSkewness(true), _bIncludeKurtosis(true)
{
    // set default options
    declareOption("noColor", false);
    declareOption("noIntensity", true);
    declareOption("noTexture", false);
    declareOption("noGeometry", false);
    declareOption("noLocation", false);

    declareOption("includeStdev", _bIncludeStdev);
    declareOption("includeSkewness", _bIncludeSkewness);
    declareOption("includeKurtosis", _bIncludeKurtosis);
}

svlRegionFeatures::~svlRegionFeatures()
{
    // do nothing
}

// compute feature responses
vector<vector<double> > svlRegionFeatures::computeFeatures(IplImage *image,
    const vector<vector<CvPoint> >& regions)
{
    SVL_ASSERT((image != NULL) && (image->depth == IPL_DEPTH_8U));

    // cache options
    _bIncludeStdev = getOptionAsBool("includeStdev");
    _bIncludeSkewness = getOptionAsBool("includeSkewness");
    _bIncludeKurtosis = getOptionAsBool("includeKurtosis");

    vector<vector<double> > featureVectors(regions.size());    
    vector<double> v;

    // colour features (12 * 4)
    if (!getOptionAsBool("noColor")) {
        SVL_ASSERT(image->nChannels == 3);
	IplImage *yccImage = cvCreateImage(cvGetSize(image), IPL_DEPTH_32F, 3);
	IplImage *labImage = cvCreateImage(cvGetSize(image), IPL_DEPTH_32F, 3);
	IplImage *image32 = cvCreateImage(cvGetSize(image), IPL_DEPTH_32F, 3);
	cvCvtScale(image, image32, 1.0 / 255.0);
	cvCvtColor(image32, yccImage, CV_BGR2YCrCb);
	cvCvtColor(image32, labImage, CV_BGR2Lab);
        IplImage *rgbImage = superSaturateImage(image32);
        
	for (unsigned i = 0; i < regions.size(); i++) {
	    v = regionStatistics(image32, regions[i]);
	    featureVectors[i].insert(featureVectors[i].end(), v.begin(), v.end());
	    v = regionStatistics(yccImage, regions[i]);
	    featureVectors[i].insert(featureVectors[i].end(), v.begin(), v.end());
	    v = regionStatistics(labImage, regions[i]);
	    featureVectors[i].insert(featureVectors[i].end(), v.begin(), v.end());
	    v = regionStatistics(rgbImage, regions[i]);
	    featureVectors[i].insert(featureVectors[i].end(), v.begin(), v.end());
	}
        cvReleaseImage(&rgbImage);
	cvReleaseImage(&image32);
	cvReleaseImage(&labImage);
	cvReleaseImage(&yccImage);
    }

    // intensity (greyscale) features
    if (!getOptionAsBool("noIntensity")) {
	IplImage *gray32 = cvCreateImage(cvGetSize(image), IPL_DEPTH_32F, 1);
        if (image->nChannels == 1) {
            cvConvertScale(image, gray32, 1.0 / 255.0);
        } else {
            IplImage *grayImage = cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 1);
            cvCvtColor(image, grayImage, CV_BGR2GRAY);
            cvConvertScale(grayImage, gray32, 1.0 / 255.0);
            cvReleaseImage(&grayImage);
        }
	for (unsigned i = 0; i < regions.size(); i++) {
	    v = regionStatistics(gray32, regions[i]);
	    featureVectors[i].insert(featureVectors[i].end(), v.begin(), v.end());
        }
	cvReleaseImage(&gray32);
    }

    // texture (filter) features
    if (!getOptionAsBool("noTexture")) {
	IplImage *grayImage = cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 1);
        if (image->nChannels == 1) {
            cvCopy(image, grayImage);
        } else {
            cvCvtColor(image, grayImage, CV_BGR2GRAY);
        }
	IplImage *gray32 = cvCreateImage(cvGetSize(image), IPL_DEPTH_32F, 1);
	cvConvertScale(grayImage, gray32, 1.0 / 255.0);
        cvReleaseImage(&grayImage);

	IplImage *response = cvCreateImage(cvGetSize(image), IPL_DEPTH_32F, 1);
        populateFilterBank();
	for (unsigned i = 0; i < _filterBank.size(); i++) {
	    _filterBank[i].filter(gray32, response);
	    for (unsigned j = 0; j < regions.size(); j++) {
		v = regionStatistics(response, regions[j]);
		featureVectors[j].insert(featureVectors[j].end(), v.begin(), v.end());
	    }
	}
	cvReleaseImage(&response);
	cvReleaseImage(&gray32);
    }

    // geometry features (6)
    if (!getOptionAsBool("noGeometry")) {
	double imageArea = (double)image->width * (double)image->height;
        double sqrtImgArea = sqrt(imageArea);
	CvMat *regionMap = cvCreateMat(image->height, image->width, CV_32FC1);
	cvSet(regionMap, cvScalar(-1.0));
	vector<double> muX2(regions.size(), 0.0);
	vector<double> muY2(regions.size(), 0.0);
	vector<double> muXY(regions.size(), 0.0);
	for (int i = 0; i < (int)regions.size(); i++) {
	    double muX = 0.0;
	    double muY = 0.0;
	    for (int j = 0; j < (int)regions[i].size(); j++) {
		CV_MAT_ELEM(*regionMap, int, regions[i][j].y, regions[i][j].x) = i;
		muX += (double)regions[i][j].x;
		muY += (double)regions[i][j].y;
		muX2[i] += (double)(regions[i][j].x * regions[i][j].x);
		muY2[i] += (double)(regions[i][j].y * regions[i][j].y);
		muXY[i] += (double)(regions[i][j].x * regions[i][j].y);
	    }
	    if (!regions[i].empty()) {
		muX /= (double)regions[i].size();
		muY /= (double)regions[i].size();
		muX2[i] /= (double)regions[i].size();
		muX2[i] -= muX * muX;
		muY2[i] /= (double)regions[i].size();
		muY2[i] -= muY * muY;
		muXY[i] /= (double)regions[i].size();
		muXY[i] -= muX * muY;
	    }
	}

	vector<int> perimeter(regions.size(), 0);
	// interior
	for (int y = 1; y < image->height - 1; y++) {
	    for (int x = 1; x < image->width - 1; x++) {
		int regionId = CV_MAT_ELEM(*regionMap, int, y, x);
		if (regionId < 0) continue;
		// 4-connected neighbours
		if ((regionId != CV_MAT_ELEM(*regionMap, int, y - 1, x)) ||
		    (regionId != CV_MAT_ELEM(*regionMap, int, y, x - 1)) ||
		    (regionId != CV_MAT_ELEM(*regionMap, int, y + 1, x)) ||
		    (regionId != CV_MAT_ELEM(*regionMap, int, y, x + 1))) {
		    perimeter[regionId] += 1;
		}
	    }
	}

	// boundary
	for (int y = 0; y < image->height; y++) {
	    int regionId = CV_MAT_ELEM(*regionMap, int, y, 0);
	    if (regionId >= 0) perimeter[regionId] += 1;		
	    regionId = CV_MAT_ELEM(*regionMap, int, y, image->width - 1);
	    if (regionId >= 0) perimeter[regionId] += 1;		
	}
	for (int x = 0; x < image->width; x++) {
	    int regionId = CV_MAT_ELEM(*regionMap, int, 0, x);
	    if (regionId >= 0) perimeter[regionId] += 1;		
	    regionId = CV_MAT_ELEM(*regionMap, int, image->height - 1, x);
	    if (regionId >= 0) perimeter[regionId] += 1;		
	}


	// add features
	for (unsigned i = 0; i < regions.size(); i++) {
	    // size / shape
	    double regionSize = (double)regions[i].size();
            double sqrtRegSize = sqrt(regionSize);
	    featureVectors[i].push_back(regionSize / imageArea); // area
	    featureVectors[i].push_back((double)perimeter[i] / sqrtImgArea); // perimeter
	    if (regions[i].empty()) {
		featureVectors[i].push_back(0.0);
	    } else {
		featureVectors[i].push_back((double)perimeter[i] / sqrtRegSize); // perimeter-to-area
	    }
	    
	    // moments
	    featureVectors[i].push_back(muX2[i]);
	    featureVectors[i].push_back(muY2[i]);
	    featureVectors[i].push_back(muXY[i]);
	}
    }
    
    // location features (3)
    if (!getOptionAsBool("noLocation")) {
	for (unsigned i = 0; i < regions.size(); i++) {
	    int muX = 0;
	    int muY = 0;
	    for (vector<CvPoint>::const_iterator it = regions[i].begin();
		 it != regions[i].end(); ++it) {
		muX += it->x;
		muY += it->y;
	    }
	    if (regions[i].empty()) {
		featureVectors[i].push_back(0.0);
		featureVectors[i].push_back(0.0);
		featureVectors[i].push_back(0.0);
	    } else {
		double count = (double)regions[i].size();
		double x = (double)muX / count / (double)image->width;
		double y = (double)muY / count / (double)image->height;
		x = 2.0 * (x - 0.5);
		y = 2.0 * (y - 0.5);
		featureVectors[i].push_back(x);
		featureVectors[i].push_back(y);
		featureVectors[i].push_back(x * x + y * y);
	    }
	}
    }

    return featureVectors;
}

vector<vector<double> > svlRegionFeatures::computeFeatures(const svlSegImageBase& image)
{
    vector<vector<CvPoint> > regions(image.numSegments());
    for (unsigned i = 0; i < regions.size(); i++) {
	regions[i] = image.getPixels(i);
    }
    
    return computeFeatures(image.image(), regions);
}

void svlRegionFeatures::populateFilterBank()
{
    // check if already populated
    if (!_filterBank.empty())
        return;

    // construct standard gabor filters
    for (float theta = 0; theta < 180.0; theta += 45.0) {
        for (float gamma = 0.25; gamma < 4.1; gamma *= 4.0) {
            for (float sigma = 2.0; sigma < 8.1; sigma *= 2.0) {
#if 0
                for (float lambda = 0.5; lambda < 3.6; lambda += 1.0) {
#else
		{
		    float lambda = 1.0;
#endif
                    svlGaborConvolution gabor(16, 16, theta / 180.0f * (float)M_PI,
                        gamma, sigma, sigma * lambda);
                    gabor.normalize();
                    _filterBank.push_back(gabor);
                }
            }
        }
    }

    // construct standard laplacian-of-gaussian filters
    for (float sigma = sqrt(2.0f); sigma < 8.1f; sigma *= sqrt(2.0f)) {
	svlLoGConvolution logFilter(16, 16, sigma);
	logFilter.normalize();
	_filterBank.push_back(logFilter);
    }
}

// implements the two-pass algorithm for computing region statistics
// this has been shown to be less sensitive to round-off errors than
// computing the moments in a single pass 
vector<double> svlRegionFeatures::regionStatistics(const IplImage *response, 
    const vector<CvPoint>& region)
{
    vector<double> v;
    int numFeatures = 1;

    if (_bIncludeStdev) numFeatures += 1;
    if (_bIncludeSkewness) numFeatures += 1;
    if (_bIncludeKurtosis) numFeatures += 1;
    
    if (region.size() <= 2) {
	v.resize(numFeatures * response->nChannels, 0.0);
	return v;
    }

    double count = (double)region.size();
    v.reserve(numFeatures * response->nChannels);
    for (int n = 0; n < response->nChannels; n++) {
	double s = 0.0;
	double ep = 0.0;
	double p = 0.0;
	double mu = 0.0;
	double var = 0.0;
	double skew = 0.0;
	double kurt = 0.0;
	if (response->depth == IPL_DEPTH_8U) {
	    for (vector<CvPoint>::const_iterator it = region.begin(); 
		 it != region.end(); ++it) {
		s += (double)CV_IMAGE_ELEM(response, unsigned char, it->y,
		    response->nChannels * it->x + n); 
	    }
	    mu = s / count;
	    for (vector<CvPoint>::const_iterator it = region.begin(); 
		 it != region.end(); ++it) {
		double d = (double)CV_IMAGE_ELEM(response, unsigned char, it->y,
		    response->nChannels * it->x + n);
		ep += (s = d - mu);
		var += (p = s * s);
		skew += (p *= s);
		kurt += (p *= s);
	    }
	} else if (response->depth == IPL_DEPTH_32F) {
	    for (vector<CvPoint>::const_iterator it = region.begin(); 
		 it != region.end(); ++it) {
		s += (double)CV_IMAGE_ELEM(response, float, it->y,
		    response->nChannels * it->x + n); 
	    }
	    mu = s / count;
	    for (vector<CvPoint>::const_iterator it = region.begin(); 
		 it != region.end(); ++it) {
		double d = (double)CV_IMAGE_ELEM(response, float, it->y,
		    response->nChannels * it->x + n);
		ep += (s = d - mu);
		var += (p = s * s);
		skew += (p *= s);
		kurt += (p *= s);
	    }
	} else {
	    SVL_LOG(SVL_LOG_FATAL, "unsupported format in svlRegionFeatures::regionStatistics()");
	}
	var = (var - ep * ep / count) / (count - 1.0);

	// mean, standard deviation, skewness, kurtosis
	v.push_back(mu);
        if (_bIncludeStdev) {
            v.push_back(var > 0.0 ? sqrt(var) : 0.0);
        }
        if (_bIncludeSkewness) {
            v.push_back(var > 0.0 ? skew / (count * var * sqrt(var)) : 0.0);
        }
        if (_bIncludeKurtosis) {
            v.push_back(var > 0.0 ? kurt / (count * var * var) : 0.0);
        }
    }

    return v;
}

vector<double> svlRegionFeatures::regionStatistics(const CvMat *response, 
    const vector<CvPoint>& region)
{
    vector<double> v;
    
    if (region.size() <= 2) {
        int numFeatures = 1;
        if (_bIncludeStdev) numFeatures += 1;
        if (_bIncludeSkewness) numFeatures += 1;
        if (_bIncludeKurtosis) numFeatures += 1;
	v.resize(numFeatures, 0.0);
	return v;
    }

    double count = (double)region.size();
    v.reserve(4);

    double s = 0.0;
    double ep = 0.0;
    double p = 0.0;
    double mu = 0.0;
    double var = 0.0;
    double skew = 0.0;
    double kurt = 0.0;

    for (vector<CvPoint>::const_iterator it = region.begin(); 
	 it != region.end(); ++it) {
	s += (double)CV_MAT_ELEM(*response, float, it->y, it->x); 
    }
    mu = s / count;
    for (vector<CvPoint>::const_iterator it = region.begin(); 
	 it != region.end(); ++it) {
	double d = (double)CV_MAT_ELEM(*response, float, it->y, it->x); 
	ep += (s = d - mu);
	var += (p = s * s);
	skew += (p *= s);
	kurt += (p *= s);
    }
    var = (var - ep * ep / count) / (count - 1.0);

    // mean, standard deviation, skewness, kurtosis
    v.push_back(mu);
    if (_bIncludeStdev) {
        v.push_back(var > 0.0 ? sqrt(var) : 0.0);
    }
    if (_bIncludeSkewness) {
        v.push_back(var > 0.0 ? skew / (count * var * sqrt(var)) : 0.0);
    }
    if (_bIncludeKurtosis) {
        v.push_back(var > 0.0 ? kurt / (count * var * var) : 0.0);
    }

    return v;
}



