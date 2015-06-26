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
** FILENAME:    svlOpenCVUtils.cpp
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

#include "svlBase.h"
#include "svlOpenCVUtils.h"

using namespace std;

// trap OpenCV errors
int svlHandleCVerror(int status, const char* func_name,
    const char* err_msg, const char* file_name,
    int line, void* userdata )
{
    _svlLogLevel level = SVL_LOG_FATAL; //TODO-- interpret the error level correctly, rather than always assuming the worst
    SVL_LOG(level, "OpenCV error: " << err_msg << " in "
        << file_name << " at line " << line);

    return 0;
}

void captureOpenCVerrors()
{
    cvRedirectError(svlHandleCVerror);
}

// show an image and wait
void svlShowDebuggingImage(const CvMat *m, const char *name, bool bWait)
{
    SVL_ASSERT(m != NULL);

    // determine range
    double lb, ub;
    cvMinMaxLoc(m, &lb, &ub);
    if (ub == lb) { ub += 1.0; lb -= 1.0; }

    IplImage *img = cvCreateImage(cvSize(m->cols, m->rows), IPL_DEPTH_8U, 1);
    cvConvertScale(m, img, 255.0 / (ub - lb), -lb * 255.0 / (ub - lb));

    svlShowDebuggingImage(img, name, bWait);
    cvReleaseImage(&img);
}

void svlShowDebuggingImage(const IplImage *img, const char *name, bool bWait)
{
    SVL_ASSERT(img != NULL);
    if (name == NULL) {
        name = "debugging image";
    }

    cvNamedWindow(name, 1);
    cvShowImage(name, img);

    if (bWait) {
        cvWaitKey(-1);
        cvDestroyWindow(name);
    } else {
        cvWaitKey(100);
    }
}

// toString() routines.
string toString(const CvPoint& p)
{
    std::stringstream s;
    s << "(" << p.x << ", " << p.y << ")";
    return s.str();
}

string toString(const CvRect& r)
{
    std::stringstream s;
    s << "<" << r.x << ", " << r.y << ", " << r.width << ", " << r.height << ">";
    return s.str();
}

string toString(const CvSize& sz)
{
    std::stringstream s;
    s << "<" << sz.width << ", " << sz.height << ">";
    return s.str();
}

string toString(const CvMat& m)
{
    std::stringstream s;
    s << "[";
    for (int i = 0; i < m.rows; i++) {
        for (int j = 0; j < m.cols; j++) {
            if (cvGetElemType(&m) == CV_32SC1) {
                s << "\t" << CV_MAT_ELEM(m, int, i, j);
            } else {
                s << "\t" << cvmGet(&m, i, j);
            }
        }
        if (i != m.rows - 1) {
            s << "\n";
        } else {
            s << "\t]";
        }
    }
    return s.str();
}

string toMatlabString(const CvMat& m)
{
    std::stringstream s;
    s << "[";
    for (int i = 0; i < m.rows; i++) {
        for (int j = 0; j < m.cols; j++) {
            if (cvGetElemType(&m) == CV_32SC1) {
                s << "\t" << CV_MAT_ELEM(m, int, i, j);
            } else {
                s << " " << cvmGet(&m, i, j);
            }
        }
        if (i != m.rows - 1) {
            s << ";";
        } else {
            s << "]";
        }
    }
    return s.str();
}

string toMatlabString(const CvMat * m)
{
  return toMatlabString(* m);
}

string toString(const IplImage& img)
{
    std::stringstream s;
    s << img.width << "x" << img.height
	  << " (" << img.depth << "-bit, "
	  << img.nChannels << "-channel)";
    return s.str();
}

//Function assumes, first entry in the vector is the intensity image and second image is depth map
//images[0] = Intenstiy Image
//images[1] = Depth Map Image
string toString(const vector<IplImage*> imgs)
{
    std::stringstream s;
    for (int i = 0; i < (int)imgs.size(); i++) {
        s << "Image " << i << ": "
          << imgs[i]->width << "x" << imgs[i]->height
	  << " (" << imgs[i]->depth << "-bit, "
	  << imgs[i]->nChannels << "-channel)\n";
    }

    return s.str();
}

string typeToString(const CvMat *m)
{
    std::stringstream s;
    s << "CV_";
    switch (CV_MAT_DEPTH(cvGetElemType(m))) {
    case CV_8U: s << "8U"; break;
    case CV_8S: s << "8S"; break;
    case CV_16U: s << "16U"; break;
    case CV_16S: s << "16S"; break;
    case CV_32S: s << "32S"; break;
    case CV_32F: s << "32F"; break;
    case CV_64F: s << "64F"; break;
    default: s << "UNKNOWN";
    }
    
    s << "C" << CV_MAT_CN(cvGetElemType(m));
    return s.str();
}


void dump(const IplImage& image, int maxWidth, int maxHeight)
{
    if (maxWidth <= 0) {
	maxWidth = numeric_limits<int>::max();
    }

    if (maxHeight <= 0) {
	maxHeight = numeric_limits<int>::max();
    }

    for (int y = 0; y < image.height; y++) {
	if (y > maxHeight) {
	    cout << "..." << endl;
	    break;
	}
	for (int x = 0; x < image.width; x++) {
	    if (x > 0) {
		cout << " ";
	    }
	    if (x > maxWidth) {
		cout << "...";
		break;
	    }
	    CvScalar v = cvGet2D(&image, y, x);
	    if (image.nChannels == 1) {
		cout << v.val[0];
	    } else {
		cout << "(";
		for (int i = 0; i < image.nChannels; i++) {
		    if (i > 0) cout << ", ";
		    cout << v.val[i];
		}
		cout << ")";
	    }
	}
	cout << endl;
    }
}

void dump(const CvMat& matrix, int maxColumns, int maxRows)
{
    if (maxColumns <= 0) {
	maxColumns = numeric_limits<int>::max();
    }

    if (maxRows <= 0) {
	maxRows = numeric_limits<int>::max();
    }

    for (int y = 0; y < matrix.rows; y++) {
	if (y > maxRows) {
	    cout << "..." << endl;
	    break;
	}
	for (int x = 0; x < matrix.cols; x++) {
	    if (x > 0) {
		cout << " ";
	    }
	    if (x > maxColumns) {
		cout << "...";
		break;
	    }

            if (cvGetElemType(&matrix) == CV_32SC1) {
                cout << CV_MAT_ELEM(matrix, int, y, x);
            } else {
                cout << cvmGet(&matrix, y, x);
            }
	}
	cout << endl;
    }
}

bool readMatrix(CvMat *matrix, istream& is)
{
    SVL_ASSERT((matrix != NULL) && (!is.fail()));

    if (CV_MAT_TYPE(matrix->type) == CV_32SC1) {
	int v;
	for (int y = 0; y < matrix->rows; y++) {
	    for (int x = 0; x < matrix->cols; x++) {
		is >> v;
		CV_MAT_ELEM(*matrix, int, y, x) = v;
	    }
	}
    } else if (CV_MAT_TYPE(matrix->type) == CV_32FC1) {
	float v;
	for (int y = 0; y < matrix->rows; y++) {
	    for (int x = 0; x < matrix->cols; x++) {
		is >> v;
		CV_MAT_ELEM(*matrix, float, y, x) = v;
	    }
	}
    } else {
	double v;
	for (int y = 0; y < matrix->rows; y++) {
	    for (int x = 0; x < matrix->cols; x++) {
		is >> v;
		cvmSet(matrix, y, x, v);
	    }
	}
    }

    return true;
}

bool readMatrix(CvMat *matrix, const char *filename)
{
    SVL_ASSERT(filename != NULL);

    ifstream ifs(filename);
    if (ifs.fail()) {
        SVL_LOG(SVL_LOG_WARNING, "could not open " << filename);
	return false;
    }
    SVL_ASSERT(!ifs.fail());
    readMatrix(matrix, ifs);
    ifs.close();

    return true;
}

void writeMatrix(const CvMat *matrix, ostream& os)
{
    SVL_ASSERT((matrix != NULL) && (!os.fail()));

    if (CV_MAT_TYPE(matrix->type) == CV_32FC1) {
	for (int y = 0; y < matrix->rows; y++) {
	    for (int x = 0; x < matrix->cols; x++) {
		os << " " << CV_MAT_ELEM(*matrix, float, y, x);
	    }
	    os << "\n";
	}
    } else if (CV_MAT_TYPE(matrix->type) == CV_32SC1) {
	for (int y = 0; y < matrix->rows; y++) {
	    for (int x = 0; x < matrix->cols; x++) {
		os << " " << CV_MAT_ELEM(*matrix, int, y, x);
	    }
	    os << "\n";
	}
    } else {
	for (int y = 0; y < matrix->rows; y++) {
	    for (int x = 0; x < matrix->cols; x++) {
		os << " " << cvmGet(matrix, y, x);
	    }
	    os << "\n";
	}
    }
}

void writeMatrix(const CvMat *matrix, const char *filename)
{
    ofstream ofs(filename);
    SVL_ASSERT(!ofs.fail());
    writeMatrix(matrix, ofs);
    ofs.close();
}

IplImage* readMatrixAsIplImage(const char *filename , int width , int height)
{
    ifstream ifs(filename);
    SVL_ASSERT_MSG(!ifs.fail(), "Could not open matrix file " << filename);

    string temp;
    ifs >> temp;

    if (temp == "matv2")
      {
	ifs >> height;
	ifs >> width;
	SVL_ASSERT(!ifs.fail());
      }
    else //otherwise it is Sid's format where we rely on the caller to know the matrix size
      {
	ifs.close();
	ifs.open(filename);
	SVL_ASSERT(!ifs.fail());
	SVL_ASSERT(width > 0);
	SVL_ASSERT(height > 0);
      }

    //Used to read values in the file
    float v;

    //Create IplImage to hold matrix
    IplImage* matrix = cvCreateImage(cvSize(width,height),IPL_DEPTH_32F,1);

    for (int i = 0; i < matrix->height; i++)
	{
	for (int j = 0; j < matrix->width; j++)
	{
	    ifs >> v;
	    CV_IMAGE_ELEM(matrix, float, i, j) = v;
	}
    }

    SVL_ASSERT(!ifs.fail());

    return matrix;
}

void writeMatrixAsIplImage(const IplImage *matrix, const char *filename , int x , int y , int width , int height)
{
    ofstream ofs(filename);
    SVL_ASSERT_MSG(!ofs.fail(), "Could not write matrix to " << filename);


    ofs << "matv2 " << height << " " << width << endl;

    for (int i = y; i < y + height; i++) {
	for (int j = x; j < x + width; j++) {
	    float v = CV_IMAGE_ELEM(matrix,float,i,j);

	    SVL_ASSERT(!isnan(v));
	    SVL_ASSERT(!isinf(v));


	    ofs << v;

	    if (j != matrix->width - 1)
		ofs << " ";
	}
	if (i != matrix->height - 1)
	    ofs << "\n";
    }

    ofs.close();
}

void writeMatrixAsIplImage(const IplImage *matrix, const char *filename)
{
    ofstream ofs(filename);
    SVL_ASSERT(!ofs.fail());

    ofs << "matv2 " << matrix->height << " " << matrix->width << endl;

    for (int i = 0; i < (int)(matrix->height); i++) {
	for (int j = 0; j < (int)(matrix->width); j++) {
	    float v = CV_IMAGE_ELEM(matrix,float,i,j);

	    SVL_ASSERT(!isnan(v));
	    SVL_ASSERT(!isinf(v));

	    ofs << v;

	    if (j != matrix->width - 1)
		ofs << " ";
	    }
	if( i != matrix->height - 1 )
	    ofs<<"\n";
    }

    ofs.close();
}

IplImage *makeIplImage(const unsigned char *data, int width, int height, int channels)
{
    SVL_ASSERT(data != NULL);
    IplImage *image = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, channels);

    for (int y = 0; y < image->height; y++) {
#if 1
        int yy = y;
#else
#if defined(_WIN32)||defined(WIN32)||defined(__WIN32__)
        int yy = image->height - y - 1;
#else
        int yy = y;
#endif
#endif
	for (int x = 0; x < image->width; x++) {
	    image->imageData[y * image->widthStep + 3 * x + 2] = data[3 * (yy * image->width + x) + 0];
	    image->imageData[y * image->widthStep + 3 * x + 1] = data[3 * (yy * image->width + x) + 1];
	    image->imageData[y * image->widthStep + 3 * x + 0] = data[3 * (yy * image->width + x) + 2];
	}
    }

    return image;
}

unsigned char *makeByteStream(IplImage *image)
{
    SVL_ASSERT((image != NULL) && (image->nChannels == 3));
    unsigned char *data = new unsigned char[3 * image->width * image->height];

    for (int y = 0; y < image->height; y++) {
#if 1
        int yy = y;
#else
#if defined(_WIN32)||defined(WIN32)||defined(__WIN32__)
        int yy = image->height - y - 1;
#else
        int yy = y;
#endif
#endif
	for (int x = 0; x < image->width; x++) {
	    data[3 * (yy * image->width + x) + 0] = image->imageData[y * image->widthStep + 3 * x + 2];
	    data[3 * (yy * image->width + x) + 1] = image->imageData[y * image->widthStep + 3 * x + 1];
	    data[3 * (yy * image->width + x) + 2] = image->imageData[y * image->widthStep + 3 * x + 0];
	}
    }

    return data;
}

// matrix creation
CvMat *svlCreateMatrix(const vector<vector<double> >& data)
{
    SVL_ASSERT(!data.empty());
    CvMat *m = cvCreateMat(data.size(), data[0].size(), CV_64FC1);
    SVL_ASSERT(m != NULL);
    for (int i = 0; i < m->rows; i++) {
        SVL_ASSERT(data[i].size() == (unsigned)m->cols);
        for (int j = 0; j < m->cols; j++) {
            CV_MAT_ELEM(*m, double, i, j) = data[i][j];
        }
    }

    return m;
}

CvMat *svlCreateMatrix(const vector<vector<float> >& data)
{
    SVL_ASSERT(!data.empty());
    CvMat *m = cvCreateMat(data.size(), data[0].size(), CV_32FC1);
    SVL_ASSERT(m != NULL);
    for (int i = 0; i < m->rows; i++) {
        SVL_ASSERT(data[i].size() == (unsigned)m->cols);
        for (int j = 0; j < m->cols; j++) {
            CV_MAT_ELEM(*m, float, i, j) = data[i][j];
        }
    }

    return m;
}

// memory management
vector<IplImage *> svlCreateImages(int nImages, CvSize size, int depth, int channels)
{
    SVL_ASSERT(nImages > 0);
    vector<IplImage *> images(nImages);

    for (int i = 0; i < nImages; i++) {
        images[i] = cvCreateImage(size, depth, channels);
        SVL_ASSERT(images[i] != NULL);
    }

    return images;
}

void svlReleaseImages(vector<IplImage *>& images)
{
    for (int i = 0; i < (int)images.size(); i++) {
        if (images[i] != NULL) {
            cvReleaseImage(&images[i]);
            images[i] = NULL;
        }
    }
}

vector<CvMat *> svlCreateMatrices(int nMats, int rows, int cols, int mType)
{
    SVL_ASSERT(nMats > 0);
    vector<CvMat *> matrices(nMats);

    for (int i = 0; i < nMats; i++) {
        matrices[i] = cvCreateMat(rows, cols, mType);
        SVL_ASSERT(matrices[i] != NULL);
    }

    return matrices;
}

void svlReleaseMatrices(vector<CvMat *>& matrices)
{
    for (int i = 0; i < (int)matrices.size(); i++) {
        if (matrices[i] != NULL) {
            cvReleaseMat(&matrices[i]);
            matrices[i] = NULL;
        }
    }
}

IplImage *combineImages(const vector<IplImage *>& images, int rows, int cols)
{
    // check sizes
    SVL_ASSERT(!images.empty());
    if ((rows <= 0) && (cols <= 0)) {
	cols = (int)ceil(sqrt((float)images.size()));
    rows = (int)ceil((double)images.size() / cols);
    } else if (rows <= 0) {
	rows = (int)ceil((float)images.size() / cols);
    } else if (cols <= 0) {
	cols = (int)ceil((float)images.size() / rows);
    } else {
	SVL_ASSERT((int)images.size() <= rows * cols);
    }

    int maxWidth = 0;
    int maxHeight = 0;
    for (unsigned i = 0; i < images.size(); i++) {
	if (images[i] == NULL) continue;
	if (images[i]->width > maxWidth) {
	    maxWidth = images[i]->width;
	}
	if (images[i]->height > maxHeight) {
	    maxHeight = images[i]->height;
	}
    }
    SVL_ASSERT((maxWidth > 0) && (maxHeight > 0));

    IplImage *outImg = cvCreateImage(cvSize(maxWidth * cols, maxHeight * rows),
	images.front()->depth, images.front()->nChannels);
    cvZero(outImg);

    for (unsigned i = 0; i < images.size(); i++) {
	if (images[i] == NULL) continue;
	int x = (i % cols) * maxWidth;
	int y = ((int)(i / cols)) * maxHeight;
	cvSetImageROI(outImg, cvRect(x, y, maxWidth, maxHeight));
	cvCopyImage(images[i], outImg);
	cvResetImageROI(outImg);
    }

    return outImg;
}

IplImage *svlCreateHeatMap(const CvMat *m, svlColorMap cm)
{
    SVL_ASSERT((m != NULL) && (cvGetElemType(m) == CV_32FC1));

    IplImage *heatMap = cvCreateImage(cvSize(m->cols, m->rows), IPL_DEPTH_8U, 3);
    const float *p = (const float *)CV_MAT_ELEM_PTR(*m, 0, 0);
    for (int y = 0; y < m->rows; y++) {
        for (int x = 0; x < m->cols; x++, p++) {
            unsigned char red = 0x00;
            unsigned char green = 0x00;
            unsigned char blue = 0x00;

            switch (cm) {
                case SVL_COLORMAP_RAINBOW:
                {
                    int h = (int)(5 * (*p));
                    unsigned char color = (unsigned char)(255 * (5.0 * (*p) - (double)h));
                    switch (h) {
                    case 0:
                        red = 0x00; green = 0x00; blue = color; break;
                    case 1:
                        red = 0x00; green = color; blue = 0xff; break;
                    case 2:
                        red = 0x00; green = 0xff; blue = 0xff - color; break;
                    case 3:
                        red = color; green = 0xff; blue = 0x00; break;
                    case 4:
                        red = 0xff; green = 0xff - color; blue = 0x00; break;
                    default:
                        red = 0xff; green = 0x00; blue = 0x00; break;
                    }
                }
                break;

                case SVL_COLORMAP_HOT:
                {
                    int h = (int)(2 * (*p));
                    unsigned char color = (unsigned char)(255 * (2.0 * (*p) - (double)h));
                    switch (h) {
                    case 0:
                        red = color; green = 0x00; blue = 0x00; break;
                    case 1:
                        red = 0xff; green = color; blue = 0x00; break;
                    default:
                        red = 0xff; green = 0xff; blue = 0x00; break;
                    }
                }
                break;

                case SVL_COLORMAP_COOL:
                {
                    unsigned char color = (unsigned char)(255 * (*p));
                    red = color; green = 0xff - color; blue = 0xff;
                }
                break;

                case SVL_COLORMAP_REDGREEN:
                {
                    int h = (int)(2 * (*p));
                    unsigned char color = (unsigned char)(255 * (2.0 * (*p) - (double)h));
                    switch (h) {
                    case 0:
                        red = 0x00; green = 0xff - color; blue = 0x00; break;
                    case 1:
                        red = color; green = 0x00; blue = 0x00; break;
                    default:
                        red = 0xff; green = 0x00; blue = 0x00; break;
                    }
                }
                break;

                default:
                    SVL_LOG(SVL_LOG_FATAL, "unknown colormap type");
            }

            CV_IMAGE_ELEM(heatMap, unsigned char, y, 3 * x + 0) = blue;
            CV_IMAGE_ELEM(heatMap, unsigned char, y, 3 * x + 1) = green;
            CV_IMAGE_ELEM(heatMap, unsigned char, y, 3 * x + 2) = red;
        }
    }

    return heatMap;
}

CvMat *eigenToCV(Eigen::VectorXd &v)
{
    CvMat *rval = cvCreateMat(v.rows(), 1, CV_32FC1);
    for (int i = 0; i < rval->rows; i++)
        MAT_ELEM_F(rval, i, 0) = v[i];

    return rval;
}

CvMat *eigenToCV(Eigen::MatrixXd &m)
{
    CvMat *rval = cvCreateMat(m.rows(), m.cols(), CV_32FC1);
    float *p = (float *)CV_MAT_ELEM_PTR(*rval, 0, 0);
    for (int i = 0; i < m.rows(); i++) {
        for (int j = 0; j < m.cols(); j++) {
            *p++ = m(i, j);
        }
    }

    return rval;
}

Eigen::MatrixXd cvToEigen(CvMat *m)
{
    SVL_ASSERT(m != NULL);
    Eigen::MatrixXd rval(m->rows, m->cols);

    float *p = (float *)CV_MAT_ELEM_PTR(*m, 0, 0);
    for (int i = 0; i < m->rows; i++) {
        for (int j = 0; j < m->cols; j++) {
            rval(i, j) = (double)(*p++);
        }
    }

    return rval;
}
