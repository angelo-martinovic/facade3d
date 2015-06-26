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
** FILENAME:    svlVisionUtils.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
**              Ian Goodfellow <ia3n@cs.stanford.edu>
**              Olga Russakovsky <olga@cs.stanford.edu>
**
*****************************************************************************/

#include <cstdlib>
#include <iostream>
#include <fstream>
#include <vector>
#include <limits>
#include <list>
#include <map>

#include "cv.h"
#include "cxcore.h"
#include "highgui.h"

#include "Eigen/QR"

#include "svlBase.h"
#include "svlML.h"

#include "svlVisionUtils.h"

using namespace std;
USING_PART_OF_NAMESPACE_EIGEN;

/*******************************
 * Simple arithmetic operators *
 *******************************/

void svlAddSquared(CvMat *src1, CvMat *src2, CvMat *dst, bool bNormalize)
{
    SVL_ASSERT((src1 != NULL) && (src2 != NULL) && (dst != NULL));
    SVL_ASSERT((src1->rows == src2->rows) && (src1->cols == src2->cols));
    SVL_ASSERT((src1->rows == dst->rows) && (src1->cols == dst->cols));

    float maxValue = 0.0;
    for (int y = 0; y < dst->rows; y++) {
        for (int x = 0; x < dst->cols; x++) {
            CV_MAT_ELEM(*dst, float, y, x) =
                CV_MAT_ELEM(*src1, float, y, x) * CV_MAT_ELEM(*src1, float, y, x) +
                CV_MAT_ELEM(*src2, float, y, x) * CV_MAT_ELEM(*src2, float, y, x);
            if (CV_MAT_ELEM(*dst, float, y, x) > maxValue)
                maxValue = CV_MAT_ELEM(*dst, float, y, x);
        }
    }

    if (bNormalize && (maxValue != 0.0)) {
        for (int y = 0; y < dst->rows; y++) {
            for (int x = 0; x < dst->cols; x++) {
                CV_MAT_ELEM(*dst, float, y, x) /= maxValue;
            }
        }
    }
}

void svlSubtractMedian(IplImage * img)
{
  vector<double> depths;

  for( int y = 0 ; y < img->height ; y++)
    {
      for( int x = 0 ; x < img->width ; x++)
	{
	  depths.push_back( CV_IMAGE_ELEM(img,float,y,x) );
	}
    }

  double m = median(depths);

  for( int y=0 ; y< img->height ; y++)
    {
      for( int x=0 ; x< img->width ; x++)
	{
	  CV_IMAGE_ELEM(img,float,y,x) = CV_IMAGE_ELEM(img,float,y,x) - (float)m;
	}
    }
}

float svlMin(const CvMat * m)
{
  float rval = (float)cvmGet(m,0,0);//You are not allowed to allocate a matrix with no elements so this is safe

  for (int i = 0; i < m->rows; i++)
    {
      for (int j = 0; j < m->cols; j++)
	{
	  float val = (float)cvmGet(m,i,j);

	  if (val < rval)
	    rval = val;
	}
    }
  return rval;
}


float svlMax(const CvMat * m)
{
  float rval = (float)cvmGet(m,0,0);//You are not allowed to allocate a matrix with no elements so this is safe

  for (int i = 0; i < m->rows; i++)
    {
      for (int j = 0; j < m->cols; j++)
	{
	  float val = (float)cvmGet(m,i,j);

	  if (val > rval)
	    rval = val;
	}
    }
  return rval;
}

void svlFindLocalMaxima(const CvMat * input, vector<CvPoint> & output, int width, int height)
{
  if (width %2 == 0 || height %2 == 0)
    {
      SVL_LOG(SVL_LOG_FATAL, "svlFindLocalMaxima window dimensions must be odd");
    }

  SVL_ASSERT(width > 0);
  SVL_ASSERT(height > 0);

  width--;
  width /= 2;

  height--;
  height/=2;

  output.clear();

  for (int i = 0; i < input->rows; i++)
    {
      for (int j = 0; j < input->cols; j++)
	{
	  float val = (float)cvmGet(input, i,j);

	  bool betterFound = false;

	  for (int ii = max(i-height, 0); ii < min(i+height, input->rows-1); ii++)
	    {

	      for (int jj = max(j-width, 0); jj < min(j+width, input->cols-1); jj++)
		{
		  if (cvmGet(input,ii,jj) > val)
		    {
		      betterFound = true;
		      break;
		    }
		}
	      if (betterFound)
		{
		  break;
		}
	    }

	  if (!betterFound)
	    {
	      output.push_back(cvPoint(j,i));
	    }
	}
    }



}

double svlMeanSquaredError(const CvMat * a, const CvMat * b)
{

  SVL_ASSERT(a->rows == b->rows);
  SVL_ASSERT(a->cols == b->cols);

  CvMat * c = cvCreateMat(a->rows, a->cols, CV_32FC1);
   SVL_ASSERT(c);

  cvmSub(a,b,c);
  cvMul(c,c,c);

  CvScalar rrval = cvAvg(c);
  double rval = rrval.val[0];

  cvReleaseMat(&c);


  return rval;
}


double svlNGC(const CvMat * a, const CvMat * b)
{

  SVL_ASSERT(a->rows == b->rows);
  SVL_ASSERT(a->cols == b->cols);

  CvScalar mma = cvAvg(a);
  CvScalar mmb = cvAvg(b);

  double ma = mma.val[0];
  double mb = mmb.val[0];



  double numer = 0;
  double denom1 = 0;
  double denom2 = 0;

  for (int i = 0; i < a->rows; i++)
    {
      for (int j = 0; j < a->cols; j++)
	{
	  double adiff = cvmGet(a,i,j) - ma;
	  double bdiff = cvmGet(b,i,j) - mb;


	  numer += adiff * bdiff;
	  denom1 += adiff * adiff;
	  denom2 += bdiff * bdiff;
	}
    }


  return numer / sqrt(denom1 * denom2);
}

bool svlFloatCompare(double a, double b)
{
  double mag = max(abs(a),abs(b));

  return abs(a-b) <= mag / 100.0;
}

// OpenCV operations
CvRect svlIntersection(const CvRect& a, const CvRect& b)
{
    CvRect r;
    r.x = std::max(a.x, b.x);
    r.y = std::max(a.y, b.y);
    r.width = std::min(a.x + a.width, b.x + b.width) - r.x;
    r.height = std::min(a.y + a.height, b.y + b.height) - r.y;

    return r;
}

CvRect svlFitBoundingBox(vector<CvRect> &rects)
{
    int x1 = 0;
    int x2 = 0;
    int y1 = 0;
    int y2 = 0;
    for (unsigned i = 0; i < rects.size(); i++) {
        x1 = min(x1, rects[i].x);
        x2 = max(x2, rects[i].x + rects[i].width);
        y1 = min(y1, rects[i].y);
        y2 = max(y2, rects[i].y + rects[i].height);
    }

    return cvRect(x1, y1, x2-x1+1, y2-y1+1);
}

// input: binary image
// output: bounding box around the black pixels
CvRect svlFitBoundingBox(const IplImage *img)
{
    SVL_ASSERT(img->nChannels == 1);

    bool found = false;
    int x1 = 0;
    for (x1 = 0; x1 < img->width && !found; x1++)
        for (int y = 0; y < img->height && !found; y++)
            if (CV_IMAGE_ELEM(img, uchar, y, x1) == 0)
                found = true;
    
    found = false;
    int x2 = 0;
    for (x2 = img->width-1; x2 >= 0 && !found; x2--)
        for (int y = 0; y < img->height && !found; y++)
            if (CV_IMAGE_ELEM(img, uchar, y, x2) == 0)
                found = true;
    
    found = false;
    int y1 = 0;
    for (y1 = 0; y1 < img->height && !found; y1++)
        for (int x = 0; x < img->width && !found; x++)
            if (CV_IMAGE_ELEM(img, uchar, y1, x) == 0)
                found = true;
    
    found = false;
    int y2 = 0;
    for (y2 = img->height-1; y2 >= 0 && !found; y2--)
        for (int x = 0; x < img->width && !found; x++)
            if (CV_IMAGE_ELEM(img, uchar, y2, x) == 0)
                found = true;
    
    return cvRect(x1, y1, x2-x1+1, y2-y1+1);
}

CvRect svlFitBoundingBox(const CvMat *m, int v)
{
    SVL_ASSERT((m != NULL) && (cvGetElemType(m) == CV_32SC1));
    CvRect r = cvRect(m->width, m->height, 0, 0);

    const int *p = (const int *)CV_MAT_ELEM_PTR(*m, 0, 0);
    for (int y = 0; y < m->rows; y++) {
        for (int x = 0; x < m->cols; x++, p++) {
            if (*p != v) continue;
            r.x = std::min(r.x, x);
            r.y = std::min(r.y, y);
            r.width = std::max(r.width, x);
            r.height = std::max(r.height, y);
        }
    }

    r.width -= r.x - 1;
    r.height -= r.y - 1;

    return r;
}

CvPoint svlCenterOfMass(const IplImage *img)
{
    SVL_ASSERT(img->nChannels == 1);

    int total_x = 0;
    int total_y = 0;
    int total = 0;
    for (int y = 0; y < img->height; y++) {
        const uchar *ptr = &CV_IMAGE_ELEM(img, uchar, y, 0);
        for (int x = 0; x < img->width; x++) {
            if (ptr[x] == 0) {
                total_x += x;
                total_y += y;
                total++;
            }
        }
    }

    return cvPoint(total_x/total, total_y/total);
}

bool operator<(const CvPoint& a, const CvPoint& b)
{
    return ((a.x < b.x) || ((a.x == b.x) && (a.y < b.y)));
}

bool operator==(const CvPoint& a, const CvPoint& b)
{
    return ((a.x == b.x) && (a.y == b.y));
}

bool operator==(const CvRect& a, const CvRect& b)
{
    return ((a.x == b.x) && (a.y == b.y) && (a.width == b.width) && (a.height == b.height));
}

bool operator==(const CvSize& a, const CvSize& b)
{
  return (a.width == b.width) && (a.height == b.height);
}

bool svlContainsNanOrInf(CvMat * m)
{
  for (int i = 0; i < m->rows; i++)
    {
      for (int j=0; j < m->cols; j++)
	{
	  float val = cvmGet(m,i,j);
	  if (isnan(val))
	    return true;
	  if (isinf(val))
	    return true;
	}
    }
  return false;
}

bool svlImageUniform(IplImage *img, float min_delta) {
  SVL_ASSERT_MSG(img, "Checking for whether a NULL patch is uniform");

  bool uniform = false;
  for (int i = 1; i <= img->nChannels; i++) {
    cvSetImageCOI(img, i);
    double minVal, maxVal;
    cvMinMaxLoc(img, &minVal, &maxVal);

    // if at least one channel is uniform, the entire thing is considered uniform
    uniform = (uniform) || (maxVal - minVal <= min_delta);

    if (uniform)
      break;
  }
  cvSetImageCOI(img, 0);
  return uniform;
}

bool svlImageAlmostUniform(IplImage *img)
{
  // image can have ROI set

  SVL_ASSERT_MSG(img, "Checking for whether a NULL patch is uniform");

  double min_delta = 0;
  if (img->depth == IPL_DEPTH_8U) {
    min_delta = 32.0;
  } else if (img->depth == IPL_DEPTH_32F) {
    min_delta = 0.125;
  } else {
    SVL_ASSERT_MSG(false, "An image of depth " << img->depth
		   << " is not currently supported by isPatchUniform");
  }

  return svlImageUniform(img, min_delta);
}







/*********************
 * Scaling, resizing *
 *********************/

void scaleToRange(CvArr *array, double minValue, double maxValue)
{
    SVL_ASSERT((array != NULL) && (minValue <= maxValue));

    double m, M;
    cvMinMaxLoc(array, &m, &M);
    if (m != M) {
      //cerr << "Min " << m << ", max " << M << ", scaling factor " << (maxValue - minValue) / (M - m) << endl;
	cvScale(array, array, (maxValue - minValue) / (M - m),
            minValue - m * (maxValue - minValue) / (M - m));
    }
}

// Resize image inplace
void resizeInPlace(IplImage **image, int height, int width, int interpolation)
{
    if (((*image)->height == height) && ((*image)->width == width))
        return;

    IplImage *tmpImage = cvCreateImage(cvSize(width, height),
	(*image)->depth, (*image)->nChannels);
    cvResize(*image, tmpImage, interpolation);
    cvReleaseImage(image);
    *image = tmpImage;
}

void resizeInPlace(CvMat **matrix, int rows, int cols)
{
    if (((*matrix)->rows == rows) && ((*matrix)->cols == cols))
        return;

    CvMat *tmpMatrix = cvCreateMat(rows, cols, (*matrix)->type);
    cvResize(*matrix, tmpMatrix, CV_INTER_NN);
    cvReleaseMat(matrix);
    *matrix = tmpMatrix;
}

// Crop an image inplace
void cropInPlace(IplImage **image, CvRect& roi)
{
  svlClipRect(roi, *image);
  IplImage *tmpImage = cvCreateImage(cvSize(roi.width, roi.height),
				     (*image)->depth, (*image)->nChannels);
  cvSetImageROI(*image, roi);
  cvCopyImage(*image, tmpImage);
  cvResetImageROI(*image);
  cvReleaseImage(image);
  *image = tmpImage;
}

vector<IplImage *> cropAllImages(const vector<IplImage *> &images, CvRect &region)
{
    vector <IplImage *> subImages(images.size());
    svlClipRect(region, images[0]);

    for (unsigned i = 0; i < images.size(); i++) {
        IplImage *imgCopy = cvCloneImage(images[i]);

        subImages[i] = cvCreateImage(cvSize(region.width, region.height),
            images[i]->depth, images[i]->nChannels);
        SVL_ASSERT(subImages[i] != NULL);

        cvSetImageROI(imgCopy, region);
        cvCopyImage(imgCopy, subImages[i]);
        cvReleaseImage(&imgCopy);
    }

    return subImages;
}


void svlClipRect(CvRect& r, int width, int height)
{
  if (r.x < 0) {
    r.width += r.x; // decrease the width by that amount
    r.x = 0;
  }
  if (r.y < 0) {
    r.height += r.y; // decrease the height by that amount
    r.y = 0;
  }
  if (r.x + r.width > width) r.width = width - r.x;
  if (r.y + r.height > height) r.height = height - r.y;
  if (r.width < 0) r.width = 0;
  if (r.height < 0) r.height = 0;
}

void svlClipRect(CvRect& r, const IplImage *img)
{
  SVL_ASSERT(img != NULL);
  svlClipRect(r, img->width, img->height);
}

void svlFitRect(CvRect& r, int width, int height)
{
  if (r.x < 0) r.x = 0;
  if (r.y < 0) r.y = 0;
  if (r.x + r.width > width) r.x = width - r.width;
  if (r.y + r.height > height) r.y = height - r.height;
  if (r.x < 0) {
    r.x = 0;
    r.width = width;
  }
  if (r.height < 0) {
    r.y = 0;
    r.height = height;
  }
}

void svlFitRect(CvRect& r, const IplImage *img)
{
  SVL_ASSERT(img != NULL);
  svlFitRect(r, img->width, img->height);
}


// Increase or decrease rectangle to match aspect ratio (width/height)
void svlIncreaseToAspectRatio(CvRect &r, double aspect)
{
    SVL_ASSERT(aspect > 0.0);
    if ((double)r.width > aspect * r.height) {
        // increase height
        int expectedHeight = (int)(r.width / aspect);
        r.y -= (expectedHeight - r.height) / 2;
        r.height = expectedHeight;
    } else {
        // increase width
        int expectedWidth = (int)(aspect * r.height);
        r.x -= (expectedWidth - r.width) / 2;
        r.width = expectedWidth;
    }
}

void svlDecreaseToAspectRatio(CvRect &r, double aspect)
{
    SVL_ASSERT(aspect > 0.0);
    if ((double)r.width > aspect * r.height) {
        // decrease width
        int expectedWidth = (int)(aspect * r.height);
        r.x += (r.width - expectedWidth) / 2;
        r.width = expectedWidth;
    } else {
        // decrease height
        int expectedHeight = (int)(r.width / aspect);
        r.y += (r.height - expectedHeight) / 2;
        r.height = expectedHeight;
    }
}





/**************************
 * Image type conversions *
 **************************/

// Convert an image inplace
void convertInPlace(IplImage **image, int depth, double scale, double shift)
{
    SVL_ASSERT((image != NULL) && (*image != NULL));
    IplImage *tmpImage = cvCreateImage(cvGetSize(*image), depth, (*image)->nChannels);
    cvConvertScale(*image, tmpImage, scale, shift);
    cvReleaseImage(image);
    *image = tmpImage;
}

// Convert a matrix in place
void convertInPlace(CvMat **matrix, int depth, double scale, double shift)
{
    SVL_ASSERT((matrix != NULL) && (*matrix != NULL));
    CvMat *tmpMatrix = cvCreateMat((*matrix)->rows, (*matrix)->cols, depth);
    cvConvertScale(*matrix, tmpMatrix, scale, shift);
    cvReleaseMat(matrix);
    *matrix = tmpMatrix;
}

IplImage *create32Fimage(const IplImage *image) {
  IplImage *result = cvCreateImage(cvGetSize(image), IPL_DEPTH_32F, image->nChannels);
  if (image->depth == IPL_DEPTH_32F) {
    cvCopy(image, result);
  } else if (image->depth == IPL_DEPTH_8U) {
    cvConvertScale(image, result, 1.0/255.0);
  } else {
    SVL_LOG(SVL_LOG_FATAL, "depth not currently supported");
  }
  return result;
}





/***************
 * Color utils *
 ***************/

IplImage *greyImage(const IplImage *color)
{
    IplImage *grey = cvCreateImage(cvGetSize(color), IPL_DEPTH_8U, 1);
    SVL_ASSERT(grey != NULL);
    cvCvtColor(color, grey, CV_BGR2GRAY);
    return grey;
}

IplImage *colorImage(const IplImage *grey)
{
    IplImage *color = cvCreateImage(cvGetSize(grey), IPL_DEPTH_8U, 3);
    SVL_ASSERT(color != NULL);
    if (grey->depth != IPL_DEPTH_8U) {
	IplImage *tmpImage = cvCreateImage(cvGetSize(grey), IPL_DEPTH_8U, grey->nChannels);
	cvConvert(grey, tmpImage);
	cvCvtColor(tmpImage, color, CV_GRAY2BGR);
	cvReleaseImage(&tmpImage);
    } else {
	cvCvtColor(grey, color, CV_GRAY2BGR);
    }

    return color;
}

IplImage *superSaturateImage(const IplImage *color)
{
    IplImage *img = cvCreateImage(cvGetSize(color), IPL_DEPTH_32F, 3);
    SVL_ASSERT(img != NULL);

    if (color->depth == IPL_DEPTH_8U) {
        cvConvertScale(color, img, 1.0 / 255.0);
    } else {
        cvCopyImage(color, img);
    }

    for (int y = 0; y < img->height; y++) {
        float * const p = (float *)&img->imageData[y * img->widthStep];
	for (int x = 0; x < 3 * img->width; x += 3) {
            float m;
            m = (p[x] > p[x + 1]) ? p[x] : p[x + 1];
            if (p[x + 2] > m) m = p[x + 2];
            if (m == 0.0) continue;
            p[x] /= m; p[x + 1] /= m; p[x + 2] /= m;
	}
    }

    return img;
}

void svlContrastNormalize(IplImage *origImage)
{
  SVL_ASSERT(origImage->nChannels == 1);
  SVL_ASSERT(origImage->depth == IPL_DEPTH_8U);
  CvSize size = cvGetSize(origImage);

  int N = 21;
  IplImage *imZeroMean = create32Fimage(origImage);
  IplImage *imLocalMean = cvCreateImage(size, IPL_DEPTH_32F, 1);
  cvSmooth(imZeroMean, imLocalMean, CV_GAUSSIAN, N, N, N/4);

  cvSub(imZeroMean, imLocalMean, imZeroMean);
  cvReleaseImage(&imLocalMean);

  N = 11;
  IplImage *imVar = cvCreateImage(size, IPL_DEPTH_32F, 1);
  cvMul(imZeroMean, imZeroMean, imVar);
  cvSmooth(imVar, imVar, CV_GAUSSIAN, N, N, N/4);

  IplImage *result = cvCreateImage(size, IPL_DEPTH_32F, 1);
  for (int y = 0; y < result->height; y++) {
    for (int x = 0; x < result->width; x++) {
      float zeroMean = (CV_IMAGE_ELEM(imZeroMean, float, y, x) * 255.0);
      float var = (CV_IMAGE_ELEM(imVar, float, y, x) * 255.0 * 255.0);
      CV_IMAGE_ELEM(result, float, y, x) = (zeroMean / (sqrt(var) + 10.0));
    }
  }
  cvReleaseImage(&imVar);
  cvReleaseImage(&imZeroMean);

  scaleToRange(result, 0.0, 255.0);

  // convert back to 8U depth
  cvConvert(result, origImage);

  cvReleaseImage(&result);
}



string svlColorTypeToText(svlColorType type)
{
  switch(type) {
  case SVL_COLOR_ERROR: return "ERROR";
  case SVL_COLOR_UNDEFINED: return "UNDEFINED";
  case SVL_COLOR_GRAY: return "GRAY";
  case SVL_COLOR_BGR: return "BGR";
  case SVL_COLOR_BG: return "BG";
  case SVL_COLOR_BR: return "BR";
  case SVL_COLOR_GR: return "GR";
  case SVL_COLOR_HSV: return "HSV";
  case SVL_COLOR_HS: return "HS";
  case SVL_COLOR_HV: return "HV";
  case SVL_COLOR_H: return "H";
  case SVL_COLOR_YCrCb: return "YCrCb";
  case SVL_COLOR_CrCb: return "CrCb";
  case SVL_COLOR_R: return "RED";
  case SVL_COLOR_G: return "GREEN";
  case SVL_COLOR_B: return "BLUE";
  }
  return "";
}

void svlChangeColorModel(IplImage *&image, svlColorType cs)
{
  if (image == NULL) return;
  if (image->nChannels != 3) return;

  IplImage *result;
  switch(cs) {
    // conversions using openCV to grayscale
  case SVL_COLOR_GRAY:
    result = cvCreateImage(cvSize(image->width, image->height), image->depth, 1);
    cvCvtColor(image, result, CV_BGR2GRAY);
    cvReleaseImage(&image);
    image = result;
    return;

    // this is what it is by assumption
  case SVL_COLOR_BGR:
    return;

    // conversion to 2 colors from BGR by averaging
  case SVL_COLOR_BG:
    svlAverageColorChannels(image, 3);
    return;
  case SVL_COLOR_BR:
    svlAverageColorChannels(image, 2);
    return;
  case SVL_COLOR_GR:
    svlAverageColorChannels(image, 1);
    return;

    // conversion to HSV
  case SVL_COLOR_HSV:
    cvCvtColor(image, image, CV_BGR2HSV);
    return;

    // conversion to 2 colors from HSV by deleting a channel
  case SVL_COLOR_HS:
    cvCvtColor(image, image, CV_BGR2HSV);
    svlDeleteChannel(image, 3);
    return;
  case SVL_COLOR_HV:
    cvCvtColor(image, image, CV_BGR2HSV);
    svlDeleteChannel(image, 2);
    return;

    // conversion to intensity-type patch from HSV by leaving only the hue
  case SVL_COLOR_H:
    cvCvtColor(image, image, CV_BGR2HSV);
    svlLeaveSingleChannel(image, 1);
    return;

    // conversion to YCrCb
  case SVL_COLOR_YCrCb:
    cvCvtColor(image, image, CV_BGR2YCrCb);
    return;

    // conversion to 2 colors from YCrCb by deleting a channel
  case SVL_COLOR_CrCb:
    cvCvtColor(image, image, CV_BGR2YCrCb);
    svlDeleteChannel(image, 1);
    return;

    // extracting just one channel.
  case SVL_COLOR_R:
    svlLeaveSingleChannel(image, 3);
    return;

  case SVL_COLOR_G:
    svlLeaveSingleChannel(image, 2);
    return;

  case SVL_COLOR_B:
    svlLeaveSingleChannel(image, 1);
    return;

  default:
    SVL_ASSERT_MSG(false, "Unknown color channel");
  }
  SVL_ASSERT(false);
}


void svlDeleteChannel(IplImage *&img, int i) {
  SVL_ASSERT_MSG(img->nChannels == 3 && i >= 1 && i <= 3,
		 "Only works on 3-channel images, and the channel to delete must be 1, 2 or 3");
  IplImage *result = cvCreateImage(cvSize(img->width, img->height), img->depth, 2);

  int c1 = (i == 1) ? 2 : 1;
  int c2 = (i == 3) ? 2 : 3;

  IplImage *tmp1 = cvCreateImage(cvSize(img->width, img->height), img->depth, 1);
  cvSetImageCOI(img, c1);
  cvCopy(img, tmp1);

  IplImage *tmp2 = cvCreateImage(cvSize(img->width, img->height), img->depth, 1);
  cvSetImageCOI(img, c2);
  cvCopy(img, tmp2);

  cvMerge(tmp1, tmp2, NULL, NULL, result);

  cvReleaseImage(&img);
  cvReleaseImage(&tmp1);
  cvReleaseImage(&tmp2);
  img = result;
}

void svlLeaveSingleChannel(IplImage *&img, int i) {
  SVL_ASSERT(i <= img->nChannels);
  IplImage *result = cvCreateImage(cvSize(img->width, img->height),
				   img->depth, 1);

  cvSetImageCOI(img, i);
  cvCopy(img, result);

  cvReleaseImage(&img);
  img = result;
}

// from openCV
static const float bgr_weights[3] = {0.114f, 0.587f, 0.299f};

void svlAverageColorChannels(IplImage *&img, int i) {
  SVL_ASSERT_MSG(img->nChannels == 3 && i >= 1 && i <= 3 && img->depth == IPL_DEPTH_8U,
		 "Only works on 3-channel 8U images, and the channel to leave must be 1, 2 or 3");

  IplImage *result = cvCreateImage(cvSize(img->width, img->height), IPL_DEPTH_8U, 2);

  i = i-1;
  int c1 = (i+1)%3;
  int c2 = (i+2)%3;
  float w1 = bgr_weights[c1];
  float w2 = bgr_weights[c2];
  float total = w1 + w2;

  // Do floats if depth is not 8U
  for(int y = 0; y < img->height; y++) {
    const char *im = &(img->imageData[y * img->widthStep]);
    char *r = &(result->imageData[y * result->widthStep]);

    for(int x = 0; x < img->width; x++) {
      r[2*x] = im[3*x+i];
      r[2*x + 1] = (uchar)((w1 * im[3*x + c1] + w2 * im[3*x + c2]) / total);
    }
  }

  cvReleaseImage(&img);
  img = result;
}

void svlAverageChannels(IplImage *image, IplImage *result) {
  SVL_ASSERT_MSG(image->depth == IPL_DEPTH_8U, "Only works with 8U images for now");

  SVL_ASSERT(result->depth == IPL_DEPTH_8U && result->nChannels == 1);

  if (image->nChannels == 1) {
    cvCopy(image, result);
    return;
  }

  int val = 0;
  for (int y = 0; y < image->height; y++) {
    const char *im = &(image->imageData[y * image->widthStep]);
    char *r = &(result->imageData[y * result->widthStep]);

    for (int x = 0; x < image->width; x++) {
      val = 0;
      for (int c = 0; c < image->nChannels; c++)
	val += im[image->nChannels * x + c];

      r[x] = (char)(val / image->nChannels);
    }
  }
}








/*************
 * 3-d utils *
 *************/

// estimate normals using SVD around local neighbourhood of point
void estimatePointNormals(const CvMat *X, const CvMat *Y, const CvMat *Z,
    CvMat *nX, CvMat *nY, CvMat *nZ, int windowSize)
{
    int handle = svlCodeProfiler::getHandle("estimatePointNormals");
    svlCodeProfiler::tic(handle);

    SVL_ASSERT((X != NULL) && (Y != NULL) && (Z != NULL));
    SVL_ASSERT((nX != NULL) && (nY != NULL) && (nZ != NULL));

    int width = Z->width;
    int height = Z->height;

    SVL_ASSERT((X->width == width) && (X->height == height));
    SVL_ASSERT((Y->width == width) && (Y->height == height));
    SVL_ASSERT((Z->width == width) && (Z->height == height));
    SVL_ASSERT((nX->width == width) && (nX->height == height));
    SVL_ASSERT((nY->width == width) && (nY->height == height));
    SVL_ASSERT((nZ->width == width) && (nZ->height == height));

    // allocate temporary matrices
    CvMat* C = cvCreateMat(3,3,CV_32FC1); // covariance matrix
    CvMat* U = cvCreateMat(3,3,CV_32FC1); // left orthogonal matrix
    CvMat* S = cvCreateMat(3,3,CV_32FC1); // singular values
    SVL_ASSERT((C != NULL) && (U != NULL) && (S != NULL));

    // estimate point normals by SVD
    float muX, muY, muZ;
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            cvZero(C);
	    muX = 0.0; muY = 0.0; muZ = 0.0;
            int nPoints = 0;
            for (int v = MAX(y - windowSize, 0); v < MIN(y + windowSize, height); v++) {
                for (int u = MAX(x - windowSize, 0); u < MIN(x + windowSize, width); u++) {
		    float ptX = CV_MAT_ELEM(*X, float, v, u);
		    float ptY = CV_MAT_ELEM(*Y, float, v, u);
		    float ptZ = CV_MAT_ELEM(*Z, float, v, u);
                    muX += ptX; muY += ptY; muZ += ptZ;
                    nPoints += 1;
                    CV_MAT_ELEM(*C, float, 0, 0) += ptX * ptX;
                    CV_MAT_ELEM(*C, float, 0, 1) += ptX * ptY;
                    CV_MAT_ELEM(*C, float, 0, 2) += ptX * ptZ;
                    CV_MAT_ELEM(*C, float, 1, 1) += ptY * ptY;
                    CV_MAT_ELEM(*C, float, 1, 2) += ptY * ptZ;
                    CV_MAT_ELEM(*C, float, 2, 2) += ptZ * ptZ;
                }
            }

            CV_MAT_ELEM(*C, float, 0, 0) -= (muX * muX) / (float)nPoints;
            CV_MAT_ELEM(*C, float, 0, 1) -= (muX * muY) / (float)nPoints;
	    CV_MAT_ELEM(*C, float, 0, 2) -= (muX * muZ) / (float)nPoints;
            CV_MAT_ELEM(*C, float, 1, 1) -= (muY * muY) / (float)nPoints;
	    CV_MAT_ELEM(*C, float, 1, 2) -= (muY * muZ) / (float)nPoints;
	    CV_MAT_ELEM(*C, float, 2, 2) -= (muZ * muZ) / (float)nPoints;
            CV_MAT_ELEM(*C, float, 1, 0) = CV_MAT_ELEM(*C, float, 0, 1);
            CV_MAT_ELEM(*C, float, 2, 0) = CV_MAT_ELEM(*C, float, 0, 2);
            CV_MAT_ELEM(*C, float, 2, 1) = CV_MAT_ELEM(*C, float, 1, 2);

            cvSVD(C, S, U, NULL, CV_SVD_MODIFY_A | CV_SVD_U_T);

	    float dp = CV_MAT_ELEM(*U, float, 2, 0) * CV_MAT_ELEM(*X, float, y, x) +
		CV_MAT_ELEM(*U, float, 2, 1) * CV_MAT_ELEM(*Y, float, y, x) +
		CV_MAT_ELEM(*U, float, 2, 2) * CV_MAT_ELEM(*Z, float, y, x);
	    if (dp <= 0.0) {
		CV_MAT_ELEM(*nX, float, y, x) = CV_MAT_ELEM(*U, float, 2, 0);
		CV_MAT_ELEM(*nY, float, y, x) = CV_MAT_ELEM(*U, float, 2, 1);
		CV_MAT_ELEM(*nZ, float, y, x) = CV_MAT_ELEM(*U, float, 2, 2);
	    } else {
		CV_MAT_ELEM(*nX, float, y, x) = -CV_MAT_ELEM(*U, float, 2, 0);
		CV_MAT_ELEM(*nY, float, y, x) = -CV_MAT_ELEM(*U, float, 2, 1);
		CV_MAT_ELEM(*nZ, float, y, x) = -CV_MAT_ELEM(*U, float, 2, 2);
	    }
        }
    }

    // release temporary matrices
    cvReleaseMat(&S);
    cvReleaseMat(&U);
    cvReleaseMat(&C);

    svlCodeProfiler::toc(handle);
}

void estimatePointNormalsFast(const CvMat *X, const CvMat *Y, const CvMat *Z,
    CvMat *nX, CvMat *nY, CvMat *nZ)
{
    int handle = svlCodeProfiler::getHandle("estimatePointNormalsFast");
    svlCodeProfiler::tic(handle);

    SVL_ASSERT((X != NULL) && (Y != NULL) && (Z != NULL));
    SVL_ASSERT((nX != NULL) && (nY != NULL) && (nZ != NULL));

    int width = Z->width;
    int height = Z->height;

    SVL_ASSERT((X->width == width) && (X->height == height));
    SVL_ASSERT((Y->width == width) && (Y->height == height));
    SVL_ASSERT((Z->width == width) && (Z->height == height));
    SVL_ASSERT((nX->width == width) && (nX->height == height));
    SVL_ASSERT((nY->width == width) && (nY->height == height));
    SVL_ASSERT((nZ->width == width) && (nZ->height == height));

    // estimate point normals by orthogonality to neighbours
    float u1, u3, v2, v3;
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
	    v2 = CV_MAT_ELEM(*Y, float, MIN(y + 1, height - 1), x) -
		CV_MAT_ELEM(*Y, float, MAX(y - 1, 0), x);
	    v3 = CV_MAT_ELEM(*Z, float, MIN(y + 1, height - 1), x) -
		CV_MAT_ELEM(*Z, float, MAX(y - 1, 0), x);
	    u1 = CV_MAT_ELEM(*Y, float, y, MIN(x + 1, width - 1)) -
		CV_MAT_ELEM(*Y, float, y, MAX(x - 1, 0));
	    u3 = CV_MAT_ELEM(*Z, float, y, MIN(x + 1, width - 1)) -
		CV_MAT_ELEM(*Z, float, y, MAX(x - 1, 0));

	    CV_MAT_ELEM(*nX, float, y, x) = v2 * u3;
	    CV_MAT_ELEM(*nY, float, y, x) = v3 * u1;
	    CV_MAT_ELEM(*nZ, float, y, x) = -v2 * u1;

	    float s = sqrt(CV_MAT_ELEM(*nX, float, y, x) * CV_MAT_ELEM(*nX, float, y, x) +
		CV_MAT_ELEM(*nY, float, y, x) * CV_MAT_ELEM(*nY, float, y, x) +
		CV_MAT_ELEM(*nZ, float, y, x) * CV_MAT_ELEM(*nZ, float, y, x));
	    if (s < 1.0e-12) {
		CV_MAT_ELEM(*nX, float, y, x) = 0.0;
		CV_MAT_ELEM(*nY, float, y, x) = 0.0;
		CV_MAT_ELEM(*nZ, float, y, x) = 1.0;
	    } else {
		CV_MAT_ELEM(*nX, float, y, x) /= s;
		CV_MAT_ELEM(*nY, float, y, x) /= s;
		CV_MAT_ELEM(*nZ, float, y, x) /= s;
	    }

	    float dp = CV_MAT_ELEM(*nX, float, y, x) * CV_MAT_ELEM(*X, float, y, x) +
		CV_MAT_ELEM(*Y, float, y, x) * CV_MAT_ELEM(*Y, float, y, x) +
		CV_MAT_ELEM(*Z, float, y, x) * CV_MAT_ELEM(*Z, float, y, x);
	    if (dp > 0.0) {
		CV_MAT_ELEM(*nX, float, y, x) = -CV_MAT_ELEM(*nX, float, 2, 0);
		CV_MAT_ELEM(*nY, float, y, x) = -CV_MAT_ELEM(*nY, float, 2, 1);
		CV_MAT_ELEM(*nZ, float, y, x) = -CV_MAT_ELEM(*nZ, float, 2, 2);
	    }
        }
    }

    svlCodeProfiler::toc(handle);
}

double estimatePlane(const CvMat *X, const CvMat *Y, const CvMat *Z,
    const vector<CvPoint>& points, svlPoint3d &n, svlPoint3d &c)
{
    int handle = svlCodeProfiler::getHandle("estimatePlane");
    svlCodeProfiler::tic(handle);

    SVL_ASSERT((X != NULL) && (Y != NULL) && (Z != NULL));

    n = svlPoint3d(0.0, 0.0, -1.0);
    c = svlPoint3d();

    if (points.empty()) {
        svlCodeProfiler::toc(handle);
        return 0.0;
    }

    int width = Z->width;
    int height = Z->height;

    SVL_ASSERT((X->width == width) && (X->height == height));
    SVL_ASSERT((Y->width == width) && (Y->height == height));
    SVL_ASSERT((Z->width == width) && (Z->height == height));

#if 0
    // remove outliers
    vector<CvPoint> trimmedPoints(points);
    vector<double> s(trimmedPoints.size());
#if 1
    // X
    for (unsigned i = 0; i < trimmedPoints.size(); i++) {
        s[i] = CV_MAT_ELEM(*X, float, trimmedPoints[i].y,
            trimmedPoints[i].x);
    }
    trimmedPoints = removeOutliers(trimmedPoints, s,
        (int)(0.9 * trimmedPoints.size()));
    // Y
    s.resize(trimmedPoints.size());
    for (unsigned i = 0; i < trimmedPoints.size(); i++) {
        s[i] = CV_MAT_ELEM(*Y, float, trimmedPoints[i].y,
            trimmedPoints[i].x);
    }
    trimmedPoints = removeOutliers(trimmedPoints, s,
        (int)(0.9 * trimmedPoints.size()));
#endif
    // Z
    s.resize(trimmedPoints.size());
    for (unsigned i = 0; i < trimmedPoints.size(); i++)
    {
        s[i] = CV_MAT_ELEM(*Z, float, trimmedPoints[i].y,
            trimmedPoints[i].x);
    }

    trimmedPoints = removeOutliers(trimmedPoints, s,
        (int)(0.9 * trimmedPoints.size()));
#else
    const vector<CvPoint> &trimmedPoints(points);
#endif

    // allocate temporary matrices
    CvMat* C = cvCreateMat(3,3,CV_32FC1); // covariance matrix
    CvMat* U = cvCreateMat(3,3,CV_32FC1); // left orthogonal matrix
    CvMat* S = cvCreateMat(3,3,CV_32FC1); // singular values
    SVL_ASSERT((C != NULL) && (U != NULL) && (S != NULL));

    float muX, muY, muZ;
    float numPoints = (float)trimmedPoints.size();

    // estimate point normal by SVD
    cvZero(C);
    muX = 0.0; muY = 0.0; muZ = 0.0;
    for (unsigned i = 0; i < trimmedPoints.size(); i++) {
        int u = trimmedPoints[i].x;
        int v = trimmedPoints[i].y;
        float ptX = CV_MAT_ELEM(*X, float, v, u);
        float ptY = CV_MAT_ELEM(*Y, float, v, u);
        float ptZ = CV_MAT_ELEM(*Z, float, v, u);
        muX += ptX; muY += ptY; muZ += ptZ;
        CV_MAT_ELEM(*C, float, 0, 0) += ptX * ptX;
        CV_MAT_ELEM(*C, float, 0, 1) += ptX * ptY;
        CV_MAT_ELEM(*C, float, 0, 2) += ptX * ptZ;
        CV_MAT_ELEM(*C, float, 1, 1) += ptY * ptY;
        CV_MAT_ELEM(*C, float, 1, 2) += ptY * ptZ;
        CV_MAT_ELEM(*C, float, 2, 2) += ptZ * ptZ;
    }

    CV_MAT_ELEM(*C, float, 0, 0) -= (muX * muX) / numPoints;
    CV_MAT_ELEM(*C, float, 0, 1) -= (muX * muY) / numPoints;
    CV_MAT_ELEM(*C, float, 0, 2) -= (muX * muZ) / numPoints;
    CV_MAT_ELEM(*C, float, 1, 1) -= (muY * muY) / numPoints;
    CV_MAT_ELEM(*C, float, 1, 2) -= (muY * muZ) / numPoints;
    CV_MAT_ELEM(*C, float, 2, 2) -= (muZ * muZ) / numPoints;
    CV_MAT_ELEM(*C, float, 1, 0) = CV_MAT_ELEM(*C, float, 0, 1);
    CV_MAT_ELEM(*C, float, 2, 0) = CV_MAT_ELEM(*C, float, 0, 2);
    CV_MAT_ELEM(*C, float, 2, 1) = CV_MAT_ELEM(*C, float, 1, 2);

    cvSVD(C, S, U, NULL, CV_SVD_MODIFY_A | CV_SVD_U_T);
    float residual = CV_MAT_ELEM(*S, float, 2, 2);

    // normal
    float dp = CV_MAT_ELEM(*U, float, 2, 0) * muX +
        CV_MAT_ELEM(*U, float, 2, 1) * muY +
        CV_MAT_ELEM(*U, float, 2, 2) * muZ;
    if (dp <= 0.0) {
        n.x = CV_MAT_ELEM(*U, float, 2, 0);
        n.y = CV_MAT_ELEM(*U, float, 2, 1);
        n.z = CV_MAT_ELEM(*U, float, 2, 2);
    } else {
        n.x = -CV_MAT_ELEM(*U, float, 2, 0);
        n.y = -CV_MAT_ELEM(*U, float, 2, 1);
        n.z = -CV_MAT_ELEM(*U, float, 2, 2);
    }

    // centroid
    c = svlPoint3d(muX, muY, muZ) / numPoints;

    // release temporary matrices
    cvReleaseMat(&S);
    cvReleaseMat(&U);
    cvReleaseMat(&C);

    svlCodeProfiler::toc(handle);

    return (double)residual;
}

double estimatePlane(const MatrixXd& points, Vector3d& normal, Vector3d& centroid)
{
    SVL_ASSERT_MSG(points.cols() == 3, "points must be an n-by-3 matrix");

    int handle = svlCodeProfiler::getHandle("estimatePlane");
    svlCodeProfiler::tic(handle);

    centroid.setZero();
    normal << 0.0, 0.0, -1.0;
    if (points.rows() < 3) {
        svlCodeProfiler::toc(handle);
        return 0.0;
    }

    // compute point cloud centroid
    centroid = points.colwise().sum().transpose() / points.rows();

    // estimate point normal by SVD
    Matrix3d C = (points.transpose() * points) / points.rows() -
        centroid * centroid.transpose();

    Eigen::EigenSolver<Matrix3d> S(C);
    normal = S.eigenvectors().col(0).real();
    if (normal.z() > 0.0) {
        normal = -1.0 * normal;
    }

    svlCodeProfiler::toc(handle);
    return S.eigenvalues().real()[0];
}

void svlRotatePointCloud(CvMat *X, CvMat *Y, CvMat *Z, const CvMat *R)
{
    SVL_ASSERT((X != NULL) && (Y != NULL) && (Z != NULL) && (R != NULL));
    SVL_ASSERT((X->cols == Y->cols) && (X->rows == Y->rows));
    SVL_ASSERT((X->cols == Z->cols) && (X->rows == Z->rows));
    SVL_ASSERT((R->rows == 3) && (R->cols == 3));

    CvMat *b = cvCreateMat(3, 1, CV_32FC1);
    for (int v = 0; v < X->rows; v++) {
        for (int u = 0; u < X->cols; u++) {
            CV_MAT_ELEM(*b, float, 0, 0) = CV_MAT_ELEM(*X, float, v, u);
            CV_MAT_ELEM(*b, float, 1, 0) = CV_MAT_ELEM(*Y, float, v, u);
            CV_MAT_ELEM(*b, float, 2, 0) = CV_MAT_ELEM(*Z, float, v, u);
            cvMatMul(R, b, b);
            CV_MAT_ELEM(*X, float, v, u) = CV_MAT_ELEM(*b, float, 0, 0);
            CV_MAT_ELEM(*Y, float, v, u) = CV_MAT_ELEM(*b, float, 1, 0);
            CV_MAT_ELEM(*Z, float, v, u) = CV_MAT_ELEM(*b, float, 2, 0);
        }
    }
}

void svlTranslatePointCloud(CvMat *X, CvMat *Y, CvMat *Z,
    double dx, double dy, double dz)
{
    cvAddS(X, cvScalar(dx), X);
    cvAddS(Y, cvScalar(dy), Y);
    cvAddS(Z, cvScalar(dz), Z);
}











/***************************
 * Other helpful functions *
 ***************************/

// Renumber connected components
#if 0
int svlConnectedComponents(CvMat *m, bool b8Connected)
{
    SVL_ASSERT((m != NULL) && (cvGetElemType(m) == CV_32SC1));

    // initialize one pixel per component
    const int width = m->cols;
    const int height = m->rows;
    const int total = width * height;
    svlDisjointSets components(total);

    // merge adjacent components with same id
    const int *q = (const int *)CV_MAT_ELEM_PTR(*m, 0, 1);
    int indx = 1;

    if (b8Connected && (height > 1)) {
        // 8-connected
        for (int x = 1; x < width; x++, q++, indx++) {
            int indxSetId = components.find(indx);
            if (*q == *(q - 1)) {
                indxSetId = components.join(indxSetId, components.find(indx - 1));
            }
            if (*q == *(q + width - 1)) {
                components.join(indxSetId, components.find(indx + width - 1));
            }
        }

        for (int y = 1; y < height; y++) {
            // 4-connected
            if (*q == *(q - width)) {
                components.join(components.find(indx), components.find(indx - width));
            }

            q += 1; indx += 1;
            for (int x = 1; x < width; x++, q++, indx++) {
                int indxSetId = components.find(indx);

                // 4-connected
                if (*q == *(q - 1)) {
                    indxSetId = components.join(indxSetId, components.find(indx - 1));
                }
                if (*q == *(q - width)) {
                    indxSetId = components.join(indxSetId, components.find(indx - width));
                }
                // 8-connected
                if (*q == *(q - width - 1)) {
                    indxSetId = components.join(indxSetId, components.find(indx - width - 1));
                }
                if ((y < height - 1) && (*q == *(q + width - 1))) {
                    components.join(indxSetId, components.find(indx + width - 1));
                }
            }
        }

    } else {
        // 4-connected
        for (int x = 1; x < width; x++, q++, indx++) {
            if (*q == *(q - 1)) {
                components.join(components.find(indx), components.find(indx - 1));
            }
        }

        for (int y = 1; y < height; y++) {
            if (*q == *(q - width)) {
                components.join(components.find(indx), components.find(indx - width));
            }

            q += 1; indx += 1;
            for (int x = 1; x < width; x++, q++, indx++) {
                int indxSetId = components.find(indx);
                if (*q == *(q - 1)) {
                    indxSetId = components.join(indxSetId, components.find(indx - 1));
                }
                if (*q == *(q - width)) {
                    components.join(indxSetId, components.find(indx - width));
                }
            }
        }
    }

    // retrieve components
    map<int, int> componentIds;
    int *qq = (int *)CV_MAT_ELEM_PTR(*m, 0, 0);
    for (int i = 0; i < total; i++) {
        int setId = components.find(i);
        map<int, int>::iterator it = componentIds.find(setId);
        if (it == componentIds.end()) {
            it = componentIds.insert(it, make_pair(setId, (int)componentIds.size()));
        }
        qq[i] = it->second;
    }

    /*
    vector<int> componentIds = components.getSetIds();
    int *qq = (int *)CV_MAT_ELEM_PTR(*m, 0, 0);
    for (int i = 0; i < (int)componentIds.size(); i++) {
        qq[componentIds[i]] = i;
    }

    for (int i = 0; i < total; i++) {
        qq[i] = qq[components.find(i)];
    }
    */

    SVL_ASSERT_MSG((int)componentIds.size() == components.sets(),
        componentIds.size() << " != " << components.sets());
    return components.sets();
}
#elif 1
int svlConnectedComponents(CvMat *m, bool b8Connected)
{
    SVL_ASSERT((m != NULL) && (cvGetElemType(m) == CV_32SC1));

    // initialize nodes in disjoint set by 4-connected
    // raster scanning image
    const int width = m->cols;
    const int height = m->rows;

    CvMat *cc = cvCreateMat(height, width, CV_32SC1);
    int *p = (int *)CV_MAT_ELEM_PTR(*cc, 0, 0);
    const int *q = (const int *)CV_MAT_ELEM_PTR(*m, 0, 0);
    int totalNodes = 0;

    // first row
    *p++ = totalNodes++;
    q += 1;
    for (int x = 1; x < width; x++, p++, q++) {
        if (*q == *(q - 1)) {
            *p = *(p - 1);
        } else {
            *p = totalNodes++;
        }
    }

    // special case
    if (height == 1) {
        cvCopy(cc, m);
        cvReleaseMat(&cc);
        return totalNodes;
    }

    // remaining rows
    for (int y = 1; y < height; y++) {
        if (*q == *(q - width)) {
            *p = *(p - width);
        } else {
            *p = totalNodes++;
        }
        p++; q++;
        for (int x = 1; x < width; x++, p++, q++) {
            if (*q == *(q - 1)) {
                *p = *(p - 1);
            } else if (*q == *(q - width)) {
                *p = *(p - width);
            } else {
                *p = totalNodes++;
            }
        }
    }

    // initialize disjoint sets
    svlDisjointSets components(totalNodes);

    // merge adjacent components with same id
    p = (int *)CV_MAT_ELEM_PTR(*cc, 0, 1);
    q = (const int *)CV_MAT_ELEM_PTR(*m, 0, 1);
    if (b8Connected) {
        // 8-connected
        for (int x = 1; x < width; x++, q++, p++) {
            //if ((*q == *(q - 1)) && (*p != *(p - 1))) {
            //    components.join(components.find(*p), components.find(*(p - 1)));
            //}
            if ((*q == *(q + width - 1)) && (*p != *(p + width - 1))) {
                components.join(components.find(*p), components.find(*(p + width - 1)));
            }
        }

        for (int y = 1; y < height; y++) {
            // 4-connected
            if ((*q == *(q - width)) && (*p != *(p - width))) {
                components.join(components.find(*p), components.find(*(p- width)));
            }

            q += 1; p += 1;
            for (int x = 1; x < width; x++, q++, p++) {
                // 4-connected
                //if ((*q == *(q - 1)) && (*p != *(p - 1))) {
                //    components.join(components.find(*p), components.find(*(p - 1)));
                //}
                if ((*q == *(q - width)) && (*p != *(p - width))) {
                    components.join(components.find(*p), components.find(*(p - width)));
                }
                // 8-connected
                if ((*q == *(q - width - 1)) && (*p != *(p - width - 1))) {
                    components.join(components.find(*p), components.find(*(p - width - 1)));
                }
                if ((y < height - 1) && (*q == *(q + width - 1)) && (*p != *(p + width - 1))) {
                    components.join(components.find(*p), components.find(*(p + width - 1)));
                }
            }
        }

    } else {
        // 4-connected
        for (int x = 1; x < width; x++, q++, p++) {
            //if ((*q == *(q - 1)) && (*p != *(p - 1))) {
            //    components.join(components.find(*p), components.find(*(p - 1)));
            //}
        }
        for (int y = 1; y < height; y++) {
            if ((*q == *(q - width)) && (*p != *(p - width))) {
                components.join(components.find(*p), components.find(*(p - width)));
            }

            q += 1; p += 1;
            for (int x = 1; x < width; x++, q++, p++) {
                //if ((*q == *(q - 1)) && (*p != *(p - 1))) {
                //    components.join(components.find(*p), components.find(*(p - 1)));
                //}
                if ((*q == *(q - width)) && (*p != *(p - width))) {
                    components.join(components.find(*p), components.find(*(p - width)));
                }
            }
        }
    }

    // retrieve components
    int regionCount = 0;
    vector<int> componentIds(totalNodes, -1);
    int *qq = (int *)CV_MAT_ELEM_PTR(*m, 0, 0);
    const int *pp = (const int *)CV_MAT_ELEM_PTR(*cc, 0, 0);
    for (int i = 0; i < width * height; i++) {
        if (componentIds[pp[i]] == -1) {
            int setId = components.find(pp[i]);
            if (componentIds[setId] == -1) {
                componentIds[setId] = regionCount++;
            }
            componentIds[pp[i]] = componentIds[setId];
        }
        qq[i] = componentIds[pp[i]];
    }

    cvReleaseMat(&cc);

    return regionCount;
}
#else
int svlConnectedComponents(CvMat *m, bool b8Connected)
{
    SVL_ASSERT((m != NULL) && (cvGetElemType(m) == CV_32SC1));
    //const CvPoint NEIGHBORS4[4] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
    //const CvPoint NEIGHBORS8[4] = {{-1, -1}, {-1, 1}, {1, -1}, {1, 1}};
    //SVL_ASSERT_MSG(!b8Connected, "8-connected not implemented yet");

    CvMat *cc = cvCreateMat(m->rows, m->cols, CV_32SC1);
    const size_t ROWSTEP = (size_t)cc->step;
    const size_t COLSTEP = CV_ELEM_SIZE(cc->type);
    int componentId = 0;

    cvSet(cc, cvScalar(-1));
    int *p = (int *)CV_MAT_ELEM_PTR(*cc, 0, 0);
    int *q = (int *)CV_MAT_ELEM_PTR(*m, 0, 0);
    for (int y = 0; y < m->rows; y++) {
        for (int x = 0; x < m->cols; x++, p++, q++) {
            if (*p != -1) {
                continue;
            }

            // label connected component
            const int id = *q;
            deque<CvPoint> frontier;
            frontier.push_back(cvPoint(x, y));
            *p = componentId;
            while (!frontier.empty()) {
                const CvPoint pt = frontier.front();
                frontier.pop_front();

                //int *pp = (int *)CV_MAT_ELEM_PTR(*cc, pt.y, pt.x);
                //int *qq = (int *)CV_MAT_ELEM_PTR(*m, pt.y, pt.x);
                size_t offset = ROWSTEP * pt.y + COLSTEP * pt.x;
                int *pp = (int *)(cc->data.ptr + offset);
                int *qq = (int *)(m->data.ptr + offset);

                // unroll loop for speed
                if ((pt.x > 0) && (*(qq - 1) == id) && (*(pp - 1) == -1)) {
                    *(pp - 1) = componentId;
                    frontier.push_back(cvPoint(pt.x - 1, pt.y));
                }
                if ((pt.x < m->cols - 1) && (*(qq + 1) == id) && (*(pp + 1) == -1)) {
                    *(pp + 1) = componentId;
                    frontier.push_back(cvPoint(pt.x + 1, pt.y));
                }
                if ((pt.y > 0) && (*(qq - m->cols) == id) && (*(pp - m->cols) == -1)) {
                    *(pp - m->cols) = componentId;
                    frontier.push_back(cvPoint(pt.x, pt.y - 1));
                }
                if ((pt.y < m->rows - 1) && (*(qq + m->cols) == id) && (*(pp + m->cols) == -1)) {
                    *(pp + m->cols) = componentId;
                    frontier.push_back(cvPoint(pt.x, pt.y + 1));
                }

                if (b8Connected) {
                    // (x-1, y+1)
                    if ((pt.x > 0) && (pt.y < m->rows - 1) && (*(qq - 1 + m->cols) == id) && (*(pp - 1 + m->cols) == -1)) {
                        *(pp - 1 + m->cols) = componentId;
                        frontier.push_back(cvPoint(pt.x - 1, pt.y + 1));
                    }
                    // (x+1, y+1)
                    if ((pt.x < m->cols - 1) && (pt.y < m->rows - 1) && (*(qq + 1 + m->cols) == id) && (*(pp + 1 + m->cols) == -1)) {
                        *(pp + 1 + m->cols) = componentId;
                        frontier.push_back(cvPoint(pt.x + 1, pt.y + 1));
                    }
                    // (x-1, y-1)
                    if ((pt.y > 0) && (pt.x > 0) && (*(qq - 1 - m->cols) == id) && (*(pp - 1 - m->cols) == -1)) {
                        *(pp - 1 -  m->cols) = componentId;
                        frontier.push_back(cvPoint(pt.x - 1, pt.y - 1));
                    }
                    // (x+1, y-1)
                    if ((pt.y > 0) && (pt.x < m->cols - 1) && (*(qq + 1 - m->cols) == id) && (*(pp + 1 - m->cols) == -1)) {
                        *(pp + 1 - m->cols) = componentId;
                        frontier.push_back(cvPoint(pt.x + 1, pt.y - 1));
                    }
                }
            }

            componentId += 1;
        }
    }

    cvCopy(cc, m);
    cvReleaseMat(&cc);

    return (int)componentId;
}
#endif

// returns true if regionId is connected
#if 0
bool svlIsConnectedComponent(CvMat *m, bool b8Connected, int regionId)
{
    SVL_ASSERT(m != NULL);

    // 8-bit unsigned version
    if (cvGetElemType(m) == CV_8UC1) {
        CvMat *tmp = cvCreateMat(m->rows, m->cols, CV_32SC1);
        cvConvert(m, tmp);
        bool b = svlIsConnectedComponent(tmp, b8Connected, regionId);
        cvReleaseMat(&tmp);
        return b;
    }

    // 32-bit signed version
    SVL_ASSERT(cvGetElemType(m) == CV_32SC1);
    const int componentId = 1;

    CvMat *cc = cvCreateMat(m->rows, m->cols, CV_32SC1);
    const size_t ROWSTEP = (size_t)cc->step;
    const size_t COLSTEP = CV_ELEM_SIZE(cc->type);

    cvZero(cc);

    // find starting point
    deque<CvPoint> frontier;

    const int *p = (const int *)CV_MAT_ELEM_PTR(*cc, 0, 0);
    const int *q = (const int *)CV_MAT_ELEM_PTR(*m, 0, 0);
    CvPoint seed = cvPoint(0, 0);
    for (seed.y = 0; seed.y < m->rows; seed.y++) {
        for (seed.x = 0; seed.x < m->cols; seed.x++, q++, p++) {
            if (*q == regionId) {
                frontier.push_back(seed);
                break;
            }
        }
        if (!frontier.empty()) break;
    }

    // label connected component
    while (!frontier.empty()) {
        const CvPoint pt = frontier.front();
        frontier.pop_front();

        size_t offset = ROWSTEP * pt.y + COLSTEP * pt.x;
        int *pp = (int *)(cc->data.ptr + offset);
        int *qq = (int *)(m->data.ptr + offset);

        // unroll loop for speed
        if ((pt.x > 0) && (*(qq - 1) == regionId) && (*(pp - 1) == 0)) {
            *(pp - 1) = componentId;
            frontier.push_back(cvPoint(pt.x - 1, pt.y));
        }
        if ((pt.x < m->cols - 1) && (*(qq + 1) == regionId) && (*(pp + 1) == 0)) {
            *(pp + 1) = componentId;
            frontier.push_back(cvPoint(pt.x + 1, pt.y));
        }
        if ((pt.y > 0) && (*(qq - m->cols) == regionId) && (*(pp - m->cols) == 0)) {
            *(pp - m->cols) = componentId;
            frontier.push_back(cvPoint(pt.x, pt.y - 1));
        }
        if ((pt.y < m->rows - 1) && (*(qq + m->cols) == regionId) && (*(pp + m->cols) == 0)) {
            *(pp + m->cols) = componentId;
            frontier.push_back(cvPoint(pt.x, pt.y + 1));
        }

        if (b8Connected) {
            // (x-1, y+1)
            if ((pt.x > 0) && (pt.y < m->rows - 1) && (*(qq - 1 + m->cols) == regionId) && (*(pp - 1 + m->cols) == 0)) {
                *(pp - 1 + m->cols) = componentId;
                frontier.push_back(cvPoint(pt.x - 1, pt.y + 1));
            }
            // (x+1, y+1)
            if ((pt.x < m->cols - 1) && (pt.y < m->rows - 1) && (*(qq + 1 + m->cols) == regionId) && (*(pp + 1 + m->cols) == 0)) {
                *(pp + 1 + m->cols) = componentId;
                frontier.push_back(cvPoint(pt.x + 1, pt.y + 1));
            }
            // (x-1, y-1)
            if ((pt.y > 0) && (pt.x > 0) && (*(qq - 1 - m->cols) == regionId) && (*(pp - 1 - m->cols) == 0)) {
                *(pp - 1 -  m->cols) = componentId;
                frontier.push_back(cvPoint(pt.x - 1, pt.y - 1));
            }
            // (x+1, y-1)
            if ((pt.y > 0) && (pt.x < m->cols - 1) && (*(qq + 1 - m->cols) == regionId) && (*(pp + 1 - m->cols) == 0)) {
                *(pp + 1 - m->cols) = componentId;
                frontier.push_back(cvPoint(pt.x + 1, pt.y - 1));
            }
        }
    }

    // check that all pixels with regionId have been labeled
    for ( ; seed.y < m->rows; seed.y++) {
        for ( ; seed.x < m->cols; seed.x++, p++, q++) {
            if ((*q == regionId) && (*p != componentId)) {
                cvReleaseMat(&cc);
                return false;
            }
        }
        seed.x = 0;
    }

    cvReleaseMat(&cc);
    return true;
}
#else
bool svlIsConnectedComponent(CvMat *m, bool b8Connected, int regionId)
{
    SVL_ASSERT(m != NULL);

    // 8-bit unsigned version
    if (cvGetElemType(m) == CV_8UC1) {
        CvMat *tmp = cvCreateMat(m->rows, m->cols, CV_32SC1);
        cvConvert(m, tmp);
        bool b = svlIsConnectedComponent(tmp, b8Connected, regionId);
        cvReleaseMat(&tmp);
        return b;
    }

    // 32-bit signed version
    SVL_ASSERT(cvGetElemType(m) == CV_32SC1);

    // initialize nodes in disjoint set by 4-connected
    // raster scanning image
    const int width = m->cols;
    const int height = m->rows;

    CvMat *cc = cvCreateMat(height, width, CV_32SC1);
    int *p = (int *)CV_MAT_ELEM_PTR(*cc, 0, 0);
    const int *q = (const int *)CV_MAT_ELEM_PTR(*m, 0, 0);
    int totalNodes = 0;

    // first row
    if (*q == regionId) {
        *p = totalNodes++;
    }
    p += 1; q += 1;
    for (int x = 1; x < width; x++, p++, q++) {
        if (*q == regionId) {
            if (*(q - 1) == regionId) {
                *p = *(p - 1);
            } else {
                *p = totalNodes++;
            }
        }
    }

    // remaining rows
    for (int y = 1; y < height; y++) {
        if (*q == regionId) {
            if (*(q - width) == regionId) {
                *p = *(p - width);
            } else {
                *p = totalNodes++;
            }
        }
        p++; q++;
        for (int x = 1; x < width; x++, p++, q++) {
            if (*q == regionId) {
                if (*(q - 1) == regionId) {
                    *p = *(p - 1);
                } else if (*(q - width) == regionId) {
                    *p = *(p - width);
                } else {
                    *p = totalNodes++;
                }
            }
        }
    }

    // check for simple 4-connectedness
    if (totalNodes <= 1) {
        cvReleaseMat(&cc);
        return true;
    }

    // initialize disjoint sets
    svlDisjointSets components(totalNodes);

    // merge adjacent components with same id
    if (b8Connected) {
        // 8-connected
        p = (int *)CV_MAT_ELEM_PTR(*cc, 0, 1);
        q = (const int *)CV_MAT_ELEM_PTR(*m, 0, 1);

        for (int x = 1; x < width; x++, q++, p++) {
            if ((*q == regionId) && (*(q + width - 1) == regionId) && (*p != *(p + width - 1))) {
                components.join(components.find(*p), components.find(*(p + width - 1)));
            }
        }

        for (int y = 1; y < height; y++) {
            // 4-connected            
            if ((*q == regionId) && (*(q - width) == regionId) && (*p != *(p - width))) {
                components.join(components.find(*p), components.find(*(p - width)));
            }

            q += 1; p += 1;
            for (int x = 1; x < width; x++, q++, p++) {
                if (*q == regionId) {
                    // 4-connected
                    if ((*q == *(q - width)) && (*p != *(p - width))) {
                        components.join(components.find(*p), components.find(*(p - width)));
                    }
                    // 8-connected
                    if ((*q == *(q - width - 1)) && (*p != *(p - width - 1))) {
                        components.join(components.find(*p), components.find(*(p - width - 1)));
                    }
                    if ((y < height - 1) && (*q == *(q + width - 1)) && (*p != *(p + width - 1))) {
                        components.join(components.find(*p), components.find(*(p + width - 1)));
                    }
                }
            }
        }

    } else {
        // 4-connected
        p = (int *)CV_MAT_ELEM_PTR(*cc, 1, 0);
        q = (const int *)CV_MAT_ELEM_PTR(*m, 1, 0);

        for (int y = 1; y < height; y++) {
            if ((*q == regionId) && (*(q - width) == regionId) && (*p != *(p - width))) {
                components.join(components.find(*p), components.find(*(p - width)));
            }

            q += 1; p += 1;
            for (int x = 1; x < width; x++, q++, p++) {
                if (*q == regionId) {
                    if ((*q == *(q - 1)) && (*p != *(p - 1))) {
                        components.join(components.find(*p), components.find(*(p - 1)));
                    }
                    if ((*q == *(q - width)) && (*p != *(p - width))) {
                        components.join(components.find(*p), components.find(*(p - width)));
                    }
                }
            }
        }
    }

    cvReleaseMat(&cc);
    return (components.sets() == 1);
}
#endif

// Fill zero points with value from nearest-neighbour
void svlNearestNeighbourFill(CvMat *X, float emptyToken)
{
    SVL_ASSERT(X != NULL);
    const int width = X->cols;
    const int height = X->rows;

    list<pair<int, int> > toBeFilled;
    for (int v = 0; v < height; v++) {
        for (int u = 0; u < width; u++) {
            if (CV_MAT_ELEM(*X, float, v, u) == emptyToken)
                toBeFilled.push_back(make_pair(v, u));
        }
    }

    CvMat *lastX = cvCreateMat(height, width, CV_32FC1);
    while (!toBeFilled.empty()) {
        cvCopy(X, lastX);
        for (list<pair<int, int> >::iterator it = toBeFilled.begin(); it != toBeFilled.end(); ) {
            int v = it->first;
            int u = it->second;
            if (((v - 1) >= 0) && (CV_MAT_ELEM(*lastX, float, v - 1, u) != emptyToken)) {
                CV_MAT_ELEM(*X, float, v, u) = CV_MAT_ELEM(*lastX, float, v - 1, u);
                it = toBeFilled.erase(it);
                continue;
            }
            if (((u - 1) >= 0) && (CV_MAT_ELEM(*lastX, float, v, u - 1) != emptyToken)) {
                CV_MAT_ELEM(*X, float, v, u) = CV_MAT_ELEM(*lastX, float, v, u - 1);
                it = toBeFilled.erase(it);
                continue;
            }
            if (((v + 1) < height) && (CV_MAT_ELEM(*lastX, float, v + 1, u) != emptyToken)) {
                CV_MAT_ELEM(*X, float, v, u) = CV_MAT_ELEM(*lastX, float, v + 1, u);
                it = toBeFilled.erase(it);
                continue;
            }
            if (((u + 1) < width) && (CV_MAT_ELEM(*lastX, float, v, u + 1) != emptyToken)) {
                CV_MAT_ELEM(*X, float, v, u) = CV_MAT_ELEM(*lastX, float, v, u + 1);
                it = toBeFilled.erase(it);
                continue;
            }
            ++it;
        }
    }
    cvReleaseMat(&lastX);
}

void svlNearestNeighbourFill(IplImage *X, unsigned char emptyToken)
{
    SVL_ASSERT((X != NULL) && (X->depth == IPL_DEPTH_8U));
    const int width = X->width;
    const int height = X->height;

    if (X->nChannels > 1) {
        IplImage *imgChannel = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);
        for (int c = 0; c < X->nChannels; c++) {
            cvSetImageCOI(X, c + 1);
            cvCopyImage(X, imgChannel);
            svlNearestNeighbourFill(imgChannel, emptyToken);
            cvCopyImage(imgChannel, X);
        }
        cvSetImageCOI(X, 0);
        return;
    }


    list<pair<int, int> > toBeFilled;

    for (int v = 0; v < height; v++) {
        for (int u = 0; u < width; u++) {
            if (CV_IMAGE_ELEM(X, unsigned char, v, u) == emptyToken)
                toBeFilled.push_back(make_pair(v, u));
        }
    }

    IplImage *lastX = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);
    while (!toBeFilled.empty()) {
        cvCopy(X, lastX);
        for (list<pair<int, int> >::iterator it = toBeFilled.begin(); it != toBeFilled.end(); ) {
            int v = it->first;
            int u = it->second;
            if (((v - 1) >= 0) && (CV_IMAGE_ELEM(lastX, unsigned char, v - 1, u) != emptyToken)) {
                CV_IMAGE_ELEM(X, unsigned char, v, u) = CV_IMAGE_ELEM(lastX, unsigned char, v - 1, u);
                it = toBeFilled.erase(it);
                continue;
            }
            if (((u - 1) >= 0) && (CV_IMAGE_ELEM(lastX, unsigned char, v, u - 1) != emptyToken)) {
                CV_IMAGE_ELEM(X, unsigned char, v, u) = CV_IMAGE_ELEM(lastX, unsigned char, v, u - 1);
                it = toBeFilled.erase(it);
                continue;
            }
            if (((v + 1) < height) && (CV_IMAGE_ELEM(lastX, unsigned char, v + 1, u) != emptyToken)) {
                CV_IMAGE_ELEM(X, unsigned char, v, u) = CV_IMAGE_ELEM(lastX, unsigned char, v + 1, u);
                it = toBeFilled.erase(it);
                continue;
            }
            if (((u + 1) < width) && (CV_IMAGE_ELEM(lastX, unsigned char, v, u + 1) != emptyToken)) {
                CV_IMAGE_ELEM(X, unsigned char, v, u) = CV_IMAGE_ELEM(lastX, unsigned char, v, u + 1);
                it = toBeFilled.erase(it);
                continue;
            }
            ++it;
        }
    }
    cvReleaseImage(&lastX);
}

// Returns true if p is inside polygon poly. The polygon is assumed to
// be closed,
bool svlInsidePolygon(const vector<CvPoint>& poly, const CvPoint& p)
{
    SVL_ASSERT(poly.size() > 2);

    int i, j, counter = 0;
    for (i = 0, j = (int)poly.size() - 1; i < (int)poly.size(); j = i++) {
        if (((poly[i].y <= p.y) && (poly[j].y <= p.y)) ||
            ((poly[i].y > p.y) && (poly[j].y > p.y)) ||
            ((poly[i].x < p.x) && (poly[j].x < p.x))) {
            if (((p.y == poly[j].y) && (p.x == poly[j].x)) ||
                ((p.y == poly[i].y) && (poly[i].x <= p.x) && (p.x <= poly[j].x)) ||
                ((poly[j].x <= p.x) && (p.x <= poly[i].x)))
                return true;
            continue;
        }

        int dist = (p.y - poly[i].y)*(poly[j].x - poly[i].x) -
            (p.x - poly[i].x)*(poly[j].y - poly[i].y);
        if (dist == 0)
            return true;
        if (poly[j].y < poly[i].y)
            dist = -dist;
        counter += (dist > 0);
    }

    return ((counter % 2) == 1);
}

// Fits a polynomial to a set of points. The degree of the polynomial
// is determined by the length of the coefficient vector: coefficient[0]
// is the offset, coefficient[k] miltiplies x^k. Returns the mean square
// error to the fit.
double svlPolynomialFit(const vector<CvPoint>& samples, vector<double>& coefficients)
{
    SVL_ASSERT(!coefficients.empty());
    SVL_ASSERT(samples.size() > coefficients.size());

    // create matrices
    CvMat *X = cvCreateMat(samples.size(), coefficients.size(), CV_32FC1);
    CvMat *Y = cvCreateMat(samples.size(), 1, CV_32FC1);
    CvMat *b = cvCreateMat(coefficients.size(), 1, CV_32FC1);

    for (unsigned i = 0; i < samples.size(); i++) {
        CV_MAT_ELEM(*X, float, i, 0) = 1.0f;
        for (unsigned j = 1; j < coefficients.size(); j++) {
            CV_MAT_ELEM(*X, float, i, j) = (float)samples[i].x *
                CV_MAT_ELEM(*X, float, i, j - 1);
        }
        CV_MAT_ELEM(*Y, float, i, 0) = (float)samples[i].y;
    }

    // solve b = X^{-1} Y
    cvSolve(X, Y, b, CV_SVD);

    // copy coefficients
    for (unsigned i = 0; i < coefficients.size(); i++) {
        coefficients[i] = (double)CV_MAT_ELEM(*b, float, i, 0);
    }

    // compute error
    cvGEMM(X, b, 1.0, Y, -1.0, Y);
    double mse = cvNorm(Y);

    // free memory and return
    cvReleaseMat(&b);
    cvReleaseMat(&Y);
    cvReleaseMat(&X);

    return mse;
}

// Fits the line ax + by = 1. Returns the error.
double svlLineFit(const vector<CvPoint>& samples, double& a, double& b)
{
    SVL_ASSERT(samples.size() > 1);

    // create matrices
    CvMat *X = cvCreateMat(samples.size(), 2, CV_32FC1);
    CvMat *Y = cvCreateMat(samples.size(), 1, CV_32FC1);
    CvMat *ab = cvCreateMat(2, 1, CV_32FC1);

    for (unsigned i = 0; i < samples.size(); i++) {
        CV_MAT_ELEM(*X, float, i, 0) = (float)samples[i].x;
        CV_MAT_ELEM(*X, float, i, 1) = (float)samples[i].y;
        CV_MAT_ELEM(*Y, float, i, 0) = 1.0f;
    }

    // solve b = X^{-1} Y
    cvSolve(X, Y, ab, CV_SVD);

    // copy coefficients
    a = (double)CV_MAT_ELEM(*ab, float, 0, 0);
    b = (double)CV_MAT_ELEM(*ab, float, 1, 0);

    // compute error
    cvGEMM(X, ab, 1.0, Y, -1.0, Y);
    double mse = cvNorm(Y);

    // free memory and return
    cvReleaseMat(&ab);
    cvReleaseMat(&Y);
    cvReleaseMat(&X);

    return mse;
}

// Images with x and y pixel coordinates
IplImage *svlCreateXCoordImage(const CvSize& imgSize)
{
    IplImage *img = cvCreateImage(imgSize, IPL_DEPTH_32F, 1);
    for (int y = 0; y < img->height; y++) {
        float * const p = (float *)&CV_IMAGE_ELEM(img, float, y, 0);
        for (int x = 0; x < img->width; x++) {
            p[x] = x / (double)img->width;
        }
    }

    return img;
}

IplImage *svlCreateYCoordImage(const CvSize& imgSize)
{
    IplImage *img = cvCreateImage(imgSize, IPL_DEPTH_32F, 1);
    for (int y = 0; y < img->height; y++) {
        float * const p = (float *)&CV_IMAGE_ELEM(img, float, y, 0);
        p[0] = y / (double)img->height;
        for (int x = 1; x < img->width; x++) {
            p[x] = p[0];
        }
    }

    return img;
}

void svlBinaryWriteIpl(const char *fname, const IplImage *img)
{
	FILE *fo = fopen(fname,"wb");
	int gi;
	// Write width and height as int32s.
	fwrite(&(gi=int(img->width)),sizeof(int),1,fo);
	fwrite(&(gi=int(img->height)),sizeof(int),1,fo);
	// Rest rest of file in row-major order.
	for ( int r=0; r<int(img->height); ++r ) {
		for ( int c=0; c<int(img->width); ++c ) {
			if ( img->depth == IPL_DEPTH_32F ) {
				fwrite(&CV_IMAGE_ELEM(img, float, r, c),sizeof(float),1,fo);
			} else if ( img->depth == IPL_DEPTH_64F ) {
				fwrite(&CV_IMAGE_ELEM(img, double, r, c),sizeof(double),1,fo);
			} else if ( img->depth == IPL_DEPTH_8U ) {
				fwrite(&CV_IMAGE_ELEM(img, unsigned char, r, c),sizeof(unsigned char),1,fo);
			}
		}
	}
	fclose(fo);
}

IplImage* svlBinaryReadIpl(const char *fname, int depth)
{
	FILE *fo = fopen(fname,"rb");
	int w, h;
	// Read width and height as int32s.
	fread(&w,sizeof(int),1,fo);
	fread(&h,sizeof(int),1,fo);
	// Read image in row-major order.
	IplImage *img = cvCreateImage(cvSize(w,h), depth, 1);
	int sz = 1;
	switch ( depth ) {
		case IPL_DEPTH_32S: sz = 4; break;
		case IPL_DEPTH_32F: sz = 4; break;
		case IPL_DEPTH_64F: sz = 8; break;
		// else, stay as 1.
	}
	int count = w*h*sz;
	unsigned char *d = (unsigned char*)malloc(count);
	fread(d,1,count,fo);
	unsigned char *ptr = d;
	for ( int r=0; r<h; ++r ) {
		memcpy( img->imageData + r*img->widthStep, ptr, sz*w );
		ptr += sz*w;
	}
	free(d);
	fclose(fo);
	return img;
}

void svlBinaryWriteLine(const char *fname, double *ptr, int n)
{
	FILE *fo = fopen(fname,"wb");
	fwrite(ptr,sizeof(double),n,fo);
	fclose(fo);
}

void svlBinaryWriteLine(const char *fname, float *ptr, int n)
{
	FILE *fo = fopen(fname,"wb");
	fwrite(ptr,sizeof(float),n,fo);
	fclose(fo);
}

void svlBinaryWriteLine(const char *fname, int *ptr, int n)
{
	FILE *fo = fopen(fname,"wb");
	fwrite(ptr,sizeof(int),n,fo);
	fclose(fo);
}

void svlWriteCvPointLine(const char *fname, const vector<CvPoint> &v)
{
	FILE *fo = fopen(fname,"wb");
	for ( vector<CvPoint>::const_iterator it=v.begin(); it!=v.end(); ++it ) {
		fwrite(&it->x,sizeof(int),1,fo);
		fwrite(&it->y,sizeof(int),1,fo);
	}
	fclose(fo);
}

void svlPrintImage(const IplImage *image)
{
    for (int y = 0; y < image->height; y++) {
        for (int x = 0; x < image->width; x++) {
            switch(image->depth) {
            case IPL_DEPTH_8U:
                cerr << (int)(CV_IMAGE_ELEM(image, uchar, y, x)) << " ";
                break;
            case IPL_DEPTH_32F:
                cerr << CV_IMAGE_ELEM(image, float, y, x) << " ";
                break;
            default:
                SVL_LOG(SVL_LOG_WARNING, "Image depth not supported yet, can't print");
                return;
            }
        }
        cerr << endl;
    }
}
