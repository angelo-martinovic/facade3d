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
** FILENAME:    svlOpenCVUtils.h
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
**              Sid Batra <sidbatra@cs.stanford.edu>
**              Ian Goodfellow <ia3n@cs.stanford.edu>
**              Olga Russakovsky <olga@cs.stanford.edu>
** DESCRIPTION:
**  OpenCV type conversion utility functions. More general and complicated
** functions that wrap around openCV should go into svlVisionUtils.h
**
*****************************************************************************/

#pragma once

#include <vector>
#include <iostream>
#include <sstream>

#include "cv.h"
#include "cxcore.h"
#include "highgui.h"

#include "svlBase.h"
#include "svlPoint3d.h"

#include "Eigen/Core"
#include "Eigen/Array"

#ifndef CV_LOAD_IMAGE_GRAYSCALE
#define CV_LOAD_IMAGE_GRAYSCALE 0
#define CV_LOAD_IMAGE_COLOR 1
#endif

using namespace std;

// trap openCV errors
void captureOpenCVerrors();

// show an image and wait
void svlShowDebuggingImage(const CvMat *m, const char *name = NULL, bool bWait = true);
void svlShowDebuggingImage(const IplImage *img, const char *name = NULL, bool bWait = true);

// string conversion
string toString(const CvPoint &p);
string toString(const CvRect& r);
string toString(const CvSize& s);
string toString(const CvMat& m);
string toString(const IplImage& img);
string toString(const vector<IplImage *> imgs);
string toMatlabString(const CvMat& m);
string toMatlabString(const CvMat *m);
string typeToString(const CvMat *m);

// dump data to stdout
void dump(const IplImage& image, int maxWidth = -1, int maxHeight = -1);
void dump(const CvMat& matrix, int maxColumns = -1, int maxRows = -1);

// read and write (matrix must be pre-allocated)
bool readMatrix(CvMat *matrix, istream& is);
bool readMatrix(CvMat *matrix, const char *filename);
void writeMatrix(const CvMat *matrix, ostream& os);
void writeMatrix(const CvMat *matrix, const char *filename);
IplImage* readMatrixAsIplImage(const char *filename, int width=-1, int height=-1);//unless you are using the old matrix file format, width and height are stored in the file itself
void writeMatrixAsIplImage(const IplImage *matrix, const char *filename);
void writeMatrixAsIplImage(const IplImage *matrix, const char *filename , int x , int y , int width , int height);

// Converts a byte stream to an image and vice versa. Caller is responsable for
// freeing memory.
IplImage *makeIplImage(const unsigned char *data, int width, int height, int channels = 3);
unsigned char *makeByteStream(IplImage *image);

// Matrix creation
CvMat *svlCreateMatrix(const vector<vector<double> >& data);
CvMat *svlCreateMatrix(const vector<vector<float> >& data);

//Eigen conversion
CvMat *eigenToCV(Eigen::VectorXd &v);
CvMat *eigenToCV(Eigen::MatrixXd &m);
Eigen::MatrixXd cvToEigen(CvMat *m);
inline CvRect eigenToCVRect(const Eigen::Vector4i &r)
{
    return cvRect(r[0], r[1], r[2], r[3]);
}

// Memory management over vectors of images
vector<IplImage *> svlCreateImages(int nImages, CvSize size, 
    int depth = IPL_DEPTH_8U, int channels = 1);
void svlReleaseImages(vector<IplImage *>& images);

vector<CvMat *> svlCreateMatrices(int nMats, int rows, int cols,
    int mType = CV_32FC1);
void svlReleaseMatrices(vector<CvMat *>& matrices);

// Assemble images into one big image. All images must be of the same format
// and rows * cols must be smaller than images.size(). If negative then will
// choose a square (rows = cols = ceil(sqrt(images.size()))).
IplImage *combineImages(const vector<IplImage *>& images, int rows = -1, int cols = -1);

// Convert a floating point matrix with entries in range [0, 1] to color
// image heatmap in either rainbow, hot or cool specturms.
typedef enum {
    SVL_COLORMAP_RAINBOW, SVL_COLORMAP_HOT, SVL_COLORMAP_COOL, SVL_COLORMAP_REDGREEN
} svlColorMap;

IplImage *svlCreateHeatMap(const CvMat *m, svlColorMap cm = SVL_COLORMAP_RAINBOW);

//Convenient matrix access
#define MAT_ELEM_F(mat, i, j) CV_MAT_ELEM(*mat, float, i, j)

//Convenience wrapper around cvGEMM. Computes c = a*b
inline void svlMul(const CvMat * a, const CvMat * b, CvMat * c)
{
    cvGEMM(a,b,1.0,NULL,0,c);
}
