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
** FILENAME:    svlVisionUtils.h
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
**              Sid Batra <sidbatra@cs.stanford.edu>
**              Ian Goodfellow <ia3n@cs.stanford.edu>
**              Olga Russakovsky <olga@cs.stanford.edu>
** DESCRIPTION:
**  General vision utility functions. Functions that are merely  
** openCV type conversions should go into svlOpenCVUtils.
**
*****************************************************************************/

#pragma once

#include <vector>
#include <iostream>
#include <sstream>

#include "cv.h"
#include "cxcore.h"
#include "highgui.h"

#include "Eigen/Core"
#include "Eigen/Array"

#include "svlBase.h"
#include "svlPoint3d.h"

#ifndef CV_LOAD_IMAGE_GRAYSCALE
#define CV_LOAD_IMAGE_GRAYSCALE 0
#define CV_LOAD_IMAGE_COLOR 1
#endif

using namespace std;


/*******************************
 * Simple arithmetic operators *
 *******************************/

// Basic operations on CvMat's
void svlAddSquared(CvMat *src1, CvMat *src2, CvMat *dst, bool bNormalize = false);

//Normalize based on median (assumes one channel, 32F)
void svlSubtractMedian(IplImage * img);

//Find the max/min elements of a CvMat
float svlMin(const CvMat * m);
float svlMax(const CvMat * m);

//Find a list of elements each of which is maximal within a widthxheight window
//Output will be in terms of x,y not row,col
void svlFindLocalMaxima(const CvMat * input, vector<CvPoint> & output,
			int width, int height);

//for a and b of same size, finds the average squared difference between
//a pixel in a and the corresponding pixel in b
double svlMeanSquaredError(const CvMat * a, const CvMat * b);

//Computes the normalized greyscale correlation of two image patches
double svlNGC(const CvMat * a, const CvMat * b);


//Returns true if two floats differ by <= 1% of the magnitude of the float with larger magnitude
bool svlFloatCompare(double a, double b);

// OpenCV operations
CvRect svlIntersection(const CvRect& a, const CvRect& b);

// finds the smallest bounding box around these rectangles
CvRect svlFitBoundingBox(vector<CvRect> &rects);

// input: binary image
// output: bounding box around the black pixels
CvRect svlFitBoundingBox(const IplImage *img);
CvRect svlFitBoundingBox(const CvMat *m, int v = 0);
CvPoint svlCenterOfMass(const IplImage *img);


bool operator<(const CvPoint& a, const CvPoint& b);
bool operator==(const CvPoint& a, const CvPoint& b);
bool operator==(const CvRect& a, const CvRect& b);
bool operator==(const CvSize& a, const CvSize& b);

bool svlContainsNanOrInf(CvMat * m);

bool svlImageUniform(IplImage *img, float delta = 0.0);
bool svlImageAlmostUniform(IplImage *img);


/*********************
 * Scaling, resizing *
 *********************/

// scale image or matrix
void scaleToRange(CvArr *array, double minValue = 0.0, double maxValue = 1.0);

// Inplace operations: resize, crop
void resizeInPlace(IplImage **image, int height, int width, int interpolation = CV_INTER_LINEAR);
void resizeInPlace(CvMat **matrix, int rows, int cols);
void cropInPlace(IplImage **image, CvRect& roi);

vector<IplImage *> cropAllImages(const vector<IplImage *> &images, CvRect &roi);

// Moves/chops a rectangle region to fit inside a bounding box or image (deprecated...)
void svlClipRect(CvRect& r, int width, int height);
void svlClipRect(CvRect& r, const IplImage *img);

// Moves a rectangle region if possible to fit inside a bounding box or image
// (if not possible, resorts to using the entire image's width/height)
void svlFitRect(CvRect& r, int width, int height);
void svlFitRect(CvRect &r, const IplImage *img);

// Increase or decrease rectangle to match aspect ratio (width / height)
void svlIncreaseToAspectRatio(CvRect &r, double aspect = 1.0);
void svlDecreaseToAspectRatio(CvRect &r, double aspect = 1.0);

/**************************
 * Image type conversions *
 **************************/

void convertInPlace(IplImage **image, int depth, double scale = 1.0, double shift = 0.0);
void convertInPlace(CvMat **matrix, int depth, double scale = 1.0, double shift = 0.0);
IplImage *create32Fimage(const IplImage *image);


/***************
 * Color utils *
 ***************/

// Return greyscale/color version of image
IplImage *greyImage(const IplImage *color);
IplImage *colorImage(const IplImage *grey); 
IplImage *superSaturateImage(const IplImage *color);

// sharpens the contrast in the image (assumed input: 8U with 1 channel)
void svlContrastNormalize(IplImage *image);

// Color conversion for patches as they're being loaded using svlPatchUtils
enum svlColorType {SVL_COLOR_ERROR, SVL_COLOR_UNDEFINED, SVL_COLOR_GRAY,
		   SVL_COLOR_BGR, SVL_COLOR_BG, SVL_COLOR_BR, SVL_COLOR_GR,
		   SVL_COLOR_HSV, SVL_COLOR_HS, SVL_COLOR_HV, SVL_COLOR_H,
		   SVL_COLOR_YCrCb, SVL_COLOR_CrCb, SVL_COLOR_R, SVL_COLOR_G, SVL_COLOR_B};

// for debugging purposes
string svlColorTypeToText(svlColorType type);

// assumes the image has just been loaded and thus its color space is BGR
// (note: this function can't be called multiple times on the same image)
void svlChangeColorModel(IplImage *&image, svlColorType cs);

void svlLeaveSingleChannel(IplImage *&image, int i);
// the following two functions only work for 3-color images
void svlDeleteChannel(IplImage *&image, int i);
// The image will now have two color channels: the first one i,
// the second one an average of the other two channels in the original
// (note: for now, the depth must be 8U)
void svlAverageColorChannels(IplImage *&image, int i);

// Takes in an image with any number of channels (of depth 8U)
// and averages them together (doesn't support ROI or COI).
// Result must be single-channel with 8U depth
void svlAverageChannels(IplImage *image, IplImage *result);


/*************
 * 3-d utils *
 *************/

// Compute estimate of point normals from 3d point cloud projection into image.
// Data must be arranged in N-by-M matrices and allocated externally. Normals
// returned in nX, nY, nZ and will point towards the origin.
void estimatePointNormals(const CvMat *X, const CvMat *Y, const CvMat *Z,
    CvMat *nX, CvMat *nY, CvMat *nZ, int windowSize = 3);
void estimatePointNormalsFast(const CvMat *X, const CvMat *Y, const CvMat *Z,
    CvMat *nX, CvMat *nY, CvMat *nZ);
double estimatePlane(const CvMat *X, const CvMat *Y, const CvMat *Z,
    const vector<CvPoint>& points, svlPoint3d& normal, svlPoint3d& centroid);
double estimatePlane(const Eigen::MatrixXd &points, Eigen::Vector3d& normal, Eigen::Vector3d& centroid);

// Rotate and translate a (dense) point cloud represented as three matrices.
void svlRotatePointCloud(CvMat *X, CvMat *Y, CvMat *Z, const CvMat *R);
void svlTranslatePointCloud(CvMat *X, CvMat *Y, CvMat *Z,
    double dx, double dy, double dz);


/***************************
 * Other helpful functions *
 ***************************/

// Renumber connected components (in place)
int svlConnectedComponents(CvMat *components, bool b8Connected = false);
bool svlIsConnectedComponent(CvMat *components, bool b8Connected = false, int regionId = 1);

// Fill zero points with value from nearest-neighbour
void svlNearestNeighbourFill(CvMat *X, float emptyToken = 0.0);
void svlNearestNeighbourFill(IplImage *X, unsigned char emptyToken = 0);

// Geometry functions
bool svlInsidePolygon(const vector<CvPoint>& poly, const CvPoint& p);
double svlPolynomialFit(const vector<CvPoint>& samples, vector<double>& coefficients);
double svlLineFit(const vector<CvPoint>& samples, double& a, double& b);

// Images with x and y pixel coordinates
IplImage *svlCreateXCoordImage(const CvSize& imgSize);
IplImage *svlCreateYCoordImage(const CvSize& imgSize);

/**********************
 * Input/output utils *
 **********************/

// Saves an IplImage to file.
void svlBinaryWriteIpl(const char *fname, const IplImage *img);
IplImage* svlBinaryReadIpl(const char *fname, int depth = IPL_DEPTH_32F);

// Saves a vector to a file.
template<typename C> void svlWriteLine(const char *fname, const vector<C> &v)
{
	FILE *fo = fopen(fname,"wb");
	for ( typename vector<C>::const_iterator it=v.begin(); it!=v.end(); ++it )
		fwrite(&(*it),sizeof(C),1,fo);
	fclose(fo);
}
// Saves linear data to files.
void svlBinaryWriteLine(const char *fname, double *ptr, int n);
void svlBinaryWriteLine(const char *fname, float *ptr, int n);
void svlBinaryWriteLine(const char *fname, int *ptr, int n);

void svlWriteCvPointLine(const char *fname, const vector<CvPoint> &v);

// Prints an IplImage to cerr (useful for debugging, can later be
// modified to print to file, etc.)
void svlPrintImage(const IplImage *image);
