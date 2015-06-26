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
** FILENAME:    svlSuperResolution.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
**
*****************************************************************************/

#include "../base/svlCompatibility.h"

#include <cstdlib>
#include <string>
#include <limits>
#include <iomanip>

// OpenCV library
#include "cv.h"
#include "cxcore.h"
#include "highgui.h"

// stair vision library
#include "svlBase.h"
#include "svlML.h"
#include "svlVision.h"

#include "svlSuperResolution.h"

#define SQUARE(X) ((X) * (X))
#define SUPER_RESOLUTION_DEBUG_WND_NAME "svlSuperResolution"

using namespace std;

// svlSuperResolution --------------------------------------------------------

double svlSuperResolution::MIN_DEPTH = 0.25;

void svlSuperResolution::initializeWeights(const IplImage *image)
{
    SVL_ASSERT(image != NULL);

    // check for correct size
    if ((image->width != width) || (image->height != height)) {
        IplImage *rescaledImage = cvCreateImage(cvSize(width, height), image->depth, image->nChannels);
        cvResize(image, rescaledImage);
        initializeWeights(rescaledImage);
        cvReleaseImage(&rescaledImage);
        return;
    }

    // check for greyscale
    if (image->nChannels == 3) {
        IplImage *grey = greyImage(image);
        initializeWeights(grey);
        cvReleaseImage(&grey);
        return;        
    }

    SVL_ASSERT((image->width == width) && (image->height == height));

    if (alpha_weights_m == NULL) {
        alpha_weights_m = cvCreateMat(image->height, image->width, CV_32FC1);
        beta_weights_dx = cvCreateMat(image->height, image->width, CV_32FC1);
        beta_weights_dy = cvCreateMat(image->height, image->width, CV_32FC1);
        gamma_weights_dx = cvCreateMat(image->height, image->width, CV_32FC1);
        gamma_weights_dy = cvCreateMat(image->height, image->width, CV_32FC1);
    }
    SVL_ASSERT((alpha_weights_m != NULL) &&
	    (beta_weights_dx != NULL) && (beta_weights_dy != NULL) &&
	    (gamma_weights_dx != NULL) && (gamma_weights_dy != NULL));

    cvZero(alpha_weights_m);
    cvZero(beta_weights_dx);
    cvZero(beta_weights_dy);
    cvZero(gamma_weights_dx);
    cvZero(gamma_weights_dy);
    for (int x = 1; x < image->width; x++) {
        for (int y = 1; y < image->height; y++) {
            double ddx = CV_IMAGE_ELEM(image, unsigned char, y, x) -
                CV_IMAGE_ELEM(image, unsigned char, y, x - 1);
            double ddy = CV_IMAGE_ELEM(image, unsigned char, y, x) -
                CV_IMAGE_ELEM(image, unsigned char, y - 1, x);
#if 0
            cvmSet(beta_weights_dx, y, x, (double)exp(-weightFactor * ddx * ddx));
            cvmSet(beta_weights_dy, y, x, (double)exp(-weightFactor * ddy * ddy));
#else
            cvmSet(beta_weights_dx, y, x, (double)exp(-weightFactor * fabs(ddx)));
            cvmSet(beta_weights_dy, y, x, (double)exp(-weightFactor * fabs(ddy)));
#endif
        }
    }

#if 1
    cvScale(alpha_weights_m, alpha_weights_m, 0.0, 1.0);
#else
    cvMul(beta_weights_dx, beta_weights_dy, alpha_weights_m);
    cvSmooth(alpha_weights_m, alpha_weights_m, CV_GAUSSIAN, 2 * (int)(0.05 * width) + 1,
        2 * (int)(0.05 * height) + 1);
    double minVal, maxVal;
    cvMinMaxLoc(alpha_weights_m, &minVal, &maxVal);
    cvScale(alpha_weights_m, alpha_weights_m,
	    1.0 / (maxVal - minVal), -minVal / (maxVal - minVal));
#endif

    // scale weights by alpha, beta and gamma
    cvScale(alpha_weights_m, alpha_weights_m, alpha);
    cvScale(beta_weights_dx, gamma_weights_dx, gamma);
    cvScale(beta_weights_dy, gamma_weights_dy, gamma);
    cvScale(beta_weights_dx, beta_weights_dx, beta);
    cvScale(beta_weights_dy, beta_weights_dy, beta);
}

void svlSuperResolution::initializeMeasurements(const CvMat *Z)
{
    SVL_ASSERT(Z != NULL);

    if (measured_depth == NULL) {
        measured_depth = cvCreateMat(height, width, CV_32FC1);
    }

    if ((Z->rows != height) || (Z->cols != width)) {
        CvMat *rescaledZ = cvCreateMat(height, width, CV_32FC1);
        cvResize(Z, rescaledZ, CV_INTER_NN);
        cvCopy(rescaledZ, measured_depth);
        cvReleaseMat(&rescaledZ);
    } else {
        cvCopy(Z, measured_depth);
    }
}

void svlSuperResolution::initializeDepthEstimate(const CvMat *Z_hat)
{
    if (Z_hat == NULL) {
        int k = 0;
        for (int y = 0; y < height; y++) {
	        for (int x = 0; x < width; x++, k++) {
	            setDepth(k, 0.0);
	        }
        }
        return;
    }

    if ((Z_hat->rows != height) || (Z_hat->cols != width)) {
        CvMat *rescaledZ = cvCreateMat(height, width, CV_32FC1);
        cvResize(Z_hat, rescaledZ);
        initializeDepthEstimate(rescaledZ);
        cvReleaseMat(&rescaledZ);
        return;
    }

    int k = 0;
    for (int y = 0; y < height; y++) {
	    for (int x = 0; x < width; x++, k++) {
	        setDepth(k, cvmGet(Z_hat, y, x));
	    }
    }
}

void svlSuperResolution::reconstructPointCloud(svlPointCloudData& reconstructed,
    const IplImage *image, svlCameraIntrinsics& intrinsics) const
{
    SVL_ASSERT(image != NULL);
    double scale = (double)image->width / (double)width;

    CvMat *X = cvCreateMat(height, width, CV_32FC1);
    CvMat *Y = cvCreateMat(height, width, CV_32FC1);
    CvMat *Z = cvCreateMat(height, width, CV_32FC1);
    SVL_ASSERT((X != NULL) && (Y != NULL) && (Z != NULL));

    int k = 0;
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++, k++) {
            CV_MAT_ELEM(*Z, float, y, x) = (float)getDepth(k);
        }
    }

    svlCameraIntrinsics rescaledIntrinsics(intrinsics);
    rescaledIntrinsics.rescale(scale);
    rescaledIntrinsics.calibratedXY(Z, X, Y);

    CvMat *nX = cvCreateMat(height, width, CV_32FC1);
    CvMat *nY = cvCreateMat(height, width, CV_32FC1);
    CvMat *nZ = cvCreateMat(height, width, CV_32FC1);
    SVL_ASSERT((nX != NULL) && (nY != NULL) && (nZ != NULL));

    estimatePointNormals(X, Y, Z, nX, nY, nZ, 3);

    // copy into reconstructed point cloud
    reconstructed.clear();
    for (int y = 1; y < height - 1; y++) {
        for (int x = 1; x < width - 1; x++) {
            if (CV_MAT_ELEM(*Z, float, y, x) < MIN_DEPTH)
                continue;
	    if ((fabs(CV_MAT_ELEM(*Z, float, y, x - 1) - CV_MAT_ELEM(*Z, float, y, x)) > 0.1) ||
		(fabs(CV_MAT_ELEM(*Z, float, y, x + 1) - CV_MAT_ELEM(*Z, float, y, x)) > 0.1) ||
		(fabs(CV_MAT_ELEM(*Z, float, y - 1, x) - CV_MAT_ELEM(*Z, float, y, x)) > 0.1) ||
		(fabs(CV_MAT_ELEM(*Z, float, y + 1, x) - CV_MAT_ELEM(*Z, float, y, x)) > 0.1))
		continue;

            // 3-d point
            reconstructed.pointCloud.push_back(svlPoint3d(
                CV_MAT_ELEM(*X, float, y, x),
                CV_MAT_ELEM(*Y, float, y, x),
                CV_MAT_ELEM(*Z, float, y, x)));

            // surface normal
            reconstructed.pointNormals.push_back(svlPoint3d(
                CV_MAT_ELEM(*nX, float, y, x),
                CV_MAT_ELEM(*nY, float, y, x),
                CV_MAT_ELEM(*nZ, float, y, x)));

            // colour from image
            CvScalar c = cvGet2D(image, (int)(scale * y), (int)(scale * x));
            reconstructed.pointColours.push_back(svlPoint3d(c.val[2], c.val[1], c.val[0]) / 255.0);
            reconstructed.pointWeights.push_back(0.0);
        }
    }

    cvReleaseMat(&nZ);
    cvReleaseMat(&nY);
    cvReleaseMat(&nX);

    cvReleaseMat(&Z);
    cvReleaseMat(&Y);
    cvReleaseMat(&X);
}

void svlSuperResolution::reconstructPointCloud(CvMat *X, CvMat *Y, CvMat *Z,
    svlCameraIntrinsics& intrinsics) const
{
    SVL_ASSERT((Z != NULL) && (X != NULL) && (Y != NULL));
    SVL_ASSERT((X->width == Z->width) && (X->height == Z->height));
    SVL_ASSERT((Y->width == Z->width) && (Y->height == Z->height));

    getDepthMap(Z);
    intrinsics.calibratedXY(Z, X, Y);
}

void svlSuperResolution::getDepthMap(CvMat *Z) const
{
    SVL_ASSERT(Z != NULL);

    if ((Z->width != width) || (Z->height != height)) {
        CvMat *rescaledZ = cvCreateMat(height, width, CV_32FC1);
        SVL_ASSERT(rescaledZ != NULL);
        int k = 0;
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++, k++) {
                CV_MAT_ELEM(*rescaledZ, float, y, x) = (float)getDepth(k);
            }
        }   
        cvResize(rescaledZ, Z, CV_INTER_NN);
        cvReleaseMat(&rescaledZ);
    } else {
        int k = 0;
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++, k++) {
                CV_MAT_ELEM(*Z, float, y, x) = (float)getDepth(k);
            }
        }
    }
}

// svlSecondOrderSuperResolutionMRF -------------------------------------------

const double svlSecondOrderSuperResolutionMRF::MEASUREMENT_HUBER = 0.02;
const double svlSecondOrderSuperResolutionMRF::SMOOTHNESS_HUBER = 0.001;

svlSecondOrderSuperResolutionMRF::~svlSecondOrderSuperResolutionMRF()
{
    if (tmpImage != NULL) {
        cvReleaseMat(&tmpImage);
        cvDestroyWindow(SUPER_RESOLUTION_DEBUG_WND_NAME);
    }
}

inline double svlSecondOrderSuperResolutionMRF::measurementPotential(double x, double z, float w) const
{
#if 0
    return w * (x - z) * (x - z);
#else
    return w * huberFunction(x - z, MEASUREMENT_HUBER);
#endif
}

inline double svlSecondOrderSuperResolutionMRF::dMeasurementPotential(double x, double z, float w) const
{
#if 0
    return 2.0 * w * (x - z);
#else
    return w * huberDerivative(x - z, MEASUREMENT_HUBER);
#endif
}

inline double svlSecondOrderSuperResolutionMRF::fdMeasurementPotential(double x, double z, float w, double *d) const
{
#if 0
    *df = 2.0 * w * (x - z);
    return w * (x - z) * (x - z);
#else
    double dx;
    double f = w * huberFunctionAndDerivative(x - z, &dx, MEASUREMENT_HUBER);
    *d += w * dx;
    return f;
#endif    
}

inline double svlSecondOrderSuperResolutionMRF::firstOrderPotential(double x_i, double x_j, float w_ij) const
{
    return w_ij * huberFunction(x_i - x_j, SMOOTHNESS_HUBER);
}

inline void svlSecondOrderSuperResolutionMRF::dFirstOrderPotential(double x_i, double x_j, float w_ij,
    double *d_i, double *d_j) const
{
    double dx = w_ij * huberDerivative(x_i - x_j, SMOOTHNESS_HUBER);
    *d_i += dx;
    *d_j -= dx;
}

inline double svlSecondOrderSuperResolutionMRF::fdFirstOrderPotential(double x_i, double x_j, float w_ij,
    double *d_i, double *d_j) const
{
    double dx;
    double f = w_ij * huberFunctionAndDerivative(x_i - x_j, &dx, SMOOTHNESS_HUBER);
    dx *= w_ij;
    *d_i += dx;
    *d_j -= dx;
    return f;
}

inline double svlSecondOrderSuperResolutionMRF::secondOrderPotential(double x_i, double x_j, double x_k, float w_i) const
{
    return w_i * huberFunction(2.0 * x_i - x_j - x_k, SMOOTHNESS_HUBER);
}

inline void svlSecondOrderSuperResolutionMRF::dSecondOrderPotential(double x_i, double x_j, double x_k, float w_i,
    double *d_i, double *d_j, double *d_k) const
{
    double dx = w_i * huberDerivative(2.0 * x_i - x_j - x_k, SMOOTHNESS_HUBER);
    *d_i += 2.0 * dx;
    *d_j -= dx;
    *d_k -= dx;
}

inline double svlSecondOrderSuperResolutionMRF::fdSecondOrderPotential(double x_i, double x_j, double x_k, float w_i,
    double *d_i, double *d_j, double *d_k) const
{
    double dx;
    double f = w_i * huberFunctionAndDerivative(2.0 * x_i - x_j - x_k, &dx, SMOOTHNESS_HUBER);
    dx *= w_i;
    *d_i += 2.0 * dx;
    *d_j -= dx;
    *d_k -= dx;
    return f;
}

double svlSecondOrderSuperResolutionMRF::objective(const double *v)
{
    double f = 0.0;

    // interior points
    int k = width + 1;
    for (int y = 1; y < height - 1; ++y) {
        const float *pd = &CV_MAT_ELEM(*measured_depth, float, y, 1);
        const float *pwAm = &CV_MAT_ELEM(*alpha_weights_m, float, y, 1);
        const float *pwBdx = &CV_MAT_ELEM(*beta_weights_dx, float, y, 1);
        const float *pwBdy = &CV_MAT_ELEM(*beta_weights_dy, float, y, 1);
        const float *pwGdx = &CV_MAT_ELEM(*gamma_weights_dx, float, y, 1);
        const float *pwGdy = &CV_MAT_ELEM(*gamma_weights_dy, float, y, 1);
	for (int x = 1; x < width - 1; ++x, ++k, ++pd, ++pwAm, ++pwBdx, ++pwBdy, ++pwGdx, ++pwGdy) {
	    double v_k = v[k];
	    if (*pd > MIN_DEPTH) {
		f += measurementPotential(v_k, *pd, *pwAm);
	    }

	    f += firstOrderPotential(v_k, v[k - 1], *pwBdx);
	    f += firstOrderPotential(v_k, v[k - width], *pwBdy);

	    f += secondOrderPotential(v_k, v[k - 1], v[k + 1], *pwGdx);
	    f += secondOrderPotential(v_k, v[k - width], v[k + width], *pwGdy);
	}
	k += 2;
    }

    return f;
}

void svlSecondOrderSuperResolutionMRF::gradient(const double *v, double *df)
{
    // zero component gradients
    memset((void *)df, 0, _n * sizeof(double));

    // interior points
    int k = width + 1;
    for (int y = 1; y < height - 1; ++y) {
        const float *pd = &CV_MAT_ELEM(*measured_depth, float, y, 1);
        const float *pwAm = &CV_MAT_ELEM(*alpha_weights_m, float, y, 1);
        const float *pwBdx = &CV_MAT_ELEM(*beta_weights_dx, float, y, 1);
        const float *pwBdy = &CV_MAT_ELEM(*beta_weights_dy, float, y, 1);
        const float *pwGdx = &CV_MAT_ELEM(*gamma_weights_dx, float, y, 1);
        const float *pwGdy = &CV_MAT_ELEM(*gamma_weights_dy, float, y, 1);
	for (int x = 1; x < width - 1; ++x, ++k, ++pd, ++pwAm, ++pwBdx, ++pwBdy, ++pwGdx, ++pwGdy) {
	    double v_k = v[k];
	    if (*pd > MIN_DEPTH) {
		df[k] += dMeasurementPotential(v_k, *pd, *pwAm);
	    }

	    dFirstOrderPotential(v_k, v[k - 1], *pwBdx, &df[k], &df[k - 1]);
	    dFirstOrderPotential(v_k, v[k - width], *pwBdy, &df[k], &df[k - width]);

 	    dSecondOrderPotential(v_k, v[k - 1], v[k + 1], *pwGdx,
		&df[k], &df[k - 1], &df[k + 1]);
	    dSecondOrderPotential(v_k, v[k - width], v[k + width], *pwGdy,
		&df[k], &df[k - width], &df[k + width]);
	}
	k += 2;
    }
}

double svlSecondOrderSuperResolutionMRF::objectiveAndGradient(const double *v, double *df)
{
    // zero component gradients
    memset((void *)df, 0, _n * sizeof(double));

    double f = 0.0;

    // interior points
    int k = width + 1;
    const double *vk = &v[k];

    if (beta == 0.0) {
	for (int y = 1; y < height - 1; ++y) {
	    const float *pd = &CV_MAT_ELEM(*measured_depth, float, y, 1);
	    const float *pwAm = &CV_MAT_ELEM(*alpha_weights_m, float, y, 1);
	    const float *pwGdx = &CV_MAT_ELEM(*gamma_weights_dx, float, y, 1);
	    const float *pwGdy = &CV_MAT_ELEM(*gamma_weights_dy, float, y, 1);
	    for (int x = 1; x < width - 1; ++x, ++vk, ++k, ++pd, ++pwAm, ++pwGdx, ++pwGdy) {
#if 0
		if (*pd > MIN_DEPTH) {
		    f += measurementPotential(*vk, *pd, *pwAm);
		    df[k] += dMeasurementPotential(*vk, *pd, *pwAm);
		}
		
		f += secondOrderPotential(*vk, *(vk - 1), *(vk + 1), *pwGdx);
		f += secondOrderPotential(*vk, *(vk - width), *(vk + width), *pwGdy);
 		dSecondOrderPotential(*vk, *(vk - 1), *(vk + 1), *pwGdx,
		    &df[k], &df[k - 1], &df[k + 1]);
		dSecondOrderPotential(*vk, *(vk - width), *(vk + width), *pwGdy,
		    &df[k], &df[k - width], &df[k + width]);
#else
		if (*pd > MIN_DEPTH)
		    f += fdMeasurementPotential(*vk, *pd, *pwAm, &df[k]);

 		f += fdSecondOrderPotential(*vk, *(vk - 1), *(vk + 1), *pwGdx,
		    &df[k], &df[k - 1], &df[k + 1]);
		f += fdSecondOrderPotential(*vk, *(vk - width), *(vk + width), *pwGdy,
		    &df[k], &df[k - width], &df[k + width]);		
#endif
	    }
	    k += 2;
	    vk += 2;
	}
    } else {
	for (int y = 1; y < height - 1; ++y) {
	    const float *pd = &CV_MAT_ELEM(*measured_depth, float, y, 1);
	    const float *pwAm = &CV_MAT_ELEM(*alpha_weights_m, float, y, 1);
	    const float *pwBdx = &CV_MAT_ELEM(*beta_weights_dx, float, y, 1);
	    const float *pwBdy = &CV_MAT_ELEM(*beta_weights_dy, float, y, 1);
	    const float *pwGdx = &CV_MAT_ELEM(*gamma_weights_dx, float, y, 1);
	    const float *pwGdy = &CV_MAT_ELEM(*gamma_weights_dy, float, y, 1);
	    for (int x = 1; x < width - 1; ++x, ++vk, ++k, ++pd, ++pwAm, ++pwBdx, ++pwBdy, ++pwGdx, ++pwGdy) {
#if 0
		if (*pd > MIN_DEPTH) {
		    f += measurementPotential(*vk, *pd, *pwAm);
		    df[k] += dMeasurementPotential(*vk, *pd, *pwAm);
		}
		
		f += firstOrderPotential(*vk, *(vk - 1), *pwBdx);
		f += firstOrderPotential(*vk, *(vk - width), *pwBdy);
		dFirstOrderPotential(*vk, *(vk - 1), *pwBdx, &df[k], &df[k - 1]);
		dFirstOrderPotential(*vk, *(vk - width), *pwBdy, &df[k], &df[k - width]);

		f += secondOrderPotential(*vk, *(vk - 1), *(vk + 1), *pwGdx);
		f += secondOrderPotential(*vk, *(vk - width), *(vk + width), *pwGdy);
		dSecondOrderPotential(*vk, *(vk - 1), *(vk + 1), *pwGdx,
		    &df[k], &df[k - 1], &df[k + 1]);
		dSecondOrderPotential(*vk, *(vk - width), *(vk + width), *pwGdy,
		    &df[k], &df[k - width], &df[k + width]);
#else
		if (*pd > MIN_DEPTH)
		    f += fdMeasurementPotential(*vk, *pd, *pwAm, &df[k]);

		f += fdFirstOrderPotential(*vk, *(vk - 1), *pwBdx, &df[k], &df[k - 1]);
		f += fdFirstOrderPotential(*vk, *(vk - width), *pwBdy, &df[k], &df[k - width]);

 		f += fdSecondOrderPotential(*vk, *(vk - 1), *(vk + 1), *pwGdx,
		    &df[k], &df[k - 1], &df[k + 1]);
		f += fdSecondOrderPotential(*vk, *(vk - width), *(vk + width), *pwGdy,
		    &df[k], &df[k - width], &df[k + width]);		
#endif
	    }
	    k += 2;
	    vk += 2;
	}
    }

    return f;
}

void svlSecondOrderSuperResolutionMRF::monitor(unsigned iter, double objValue)
{
    svlOptimizer::monitor(iter, objValue);

    if (tmpImage == NULL) {
	    tmpImage = cvCreateMat(height, width, CV_32FC1);
    	cvNamedWindow(SUPER_RESOLUTION_DEBUG_WND_NAME, CV_WINDOW_AUTOSIZE);
    }

    int k = 0;
    float *d = (float *)CV_MAT_ELEM_PTR(*tmpImage, 0, 0);
    for (int y = 0; y < height; y++) {
	for (int x = 0; x < width; x++, k++, d++) {
	    *d = (float)(*this)[k];
	}
    }
    scaleToRange(tmpImage, 0, 1.0);

    // visualize
    vector<IplImage *> panels;
    panels.push_back(cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1));
    cvConvertScale(tmpImage, panels.back(), 255.0, 0.0);
    if (gamma > 0) {
	svlAddSquared(gamma_weights_dx, gamma_weights_dy, tmpImage, true);
    } else {
	svlAddSquared(beta_weights_dx, beta_weights_dy, tmpImage, true);
    }
    panels.push_back(cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1));
    cvConvertScale(tmpImage, panels.back(), 255.0, 0.0);
    
    IplImage *canvas = combineImages(panels, 1);
    cvShowImage(SUPER_RESOLUTION_DEBUG_WND_NAME, canvas);
    cvReleaseImage(&canvas);
    svlReleaseImages(panels);

    cvWaitKey(10);
}

