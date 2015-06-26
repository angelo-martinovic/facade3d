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
** FILENAME:    svlSuperResolution.h
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
** DESCRIPTION:
**  Continuous MRF for generating superresolution depth maps with a preference
**  for planar surfaces from laser and image data.
**
*****************************************************************************/

#pragma once

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

// svlSuperResolution Inferface Class ----------------------------------------

class svlSuperResolution {
 public:
    static double MIN_DEPTH;

    double weightFactor;
    double alpha;
    double beta;
    double gamma;

 protected:
    int width, height;
    CvMat *alpha_weights_m;
    CvMat *beta_weights_dx;
    CvMat *beta_weights_dy;
    CvMat *gamma_weights_dx;
    CvMat *gamma_weights_dy;

    CvMat *measured_depth;

 public:
    svlSuperResolution(int w, int h) :  weightFactor(0.05),
        alpha(1.0), beta(0.01), gamma(0.5), width(w), height(h),
        alpha_weights_m(NULL), beta_weights_dx(NULL), beta_weights_dy(NULL),
	    gamma_weights_dx(NULL), gamma_weights_dy(NULL), measured_depth(NULL) {
	    // do nothing
    }

    virtual ~svlSuperResolution() {
	    if (alpha_weights_m != NULL) cvReleaseMat(&alpha_weights_m);
	    if (beta_weights_dx != NULL) cvReleaseMat(&beta_weights_dx);
	    if (beta_weights_dy != NULL) cvReleaseMat(&beta_weights_dy);
	    if (gamma_weights_dx != NULL) cvReleaseMat(&gamma_weights_dx);
	    if (gamma_weights_dy != NULL) cvReleaseMat(&gamma_weights_dy);
	    if (measured_depth != NULL) cvReleaseMat(&measured_depth);
    }

    inline int getWidth() const { return width; }
    inline int getHeight() const { return height; }

    virtual void initializeWeights(const IplImage *image);
    virtual void initializeMeasurements(const CvMat *Z);
    virtual void initializeDepthEstimate(const CvMat *Z_hat = NULL);

    virtual void reconstructPointCloud(svlPointCloudData& reconstructed,
        const IplImage *image, svlCameraIntrinsics& intrinsics) const;
    virtual void reconstructPointCloud(CvMat *X, CvMat *Y, CvMat *Z,
        svlCameraIntrinsics& intrinsics) const;
    virtual void getDepthMap(CvMat *Z) const;

    virtual inline double getDepth(unsigned i) const = 0;
    virtual inline void setDepth(unsigned i, double d) = 0;
};

// svlSecondOrderSuperResolutionMRF Class -----------------------------------------

class svlSecondOrderSuperResolutionMRF : public svlSuperResolution, public svlOptimizer {
 protected:
    static const double MEASUREMENT_HUBER;
    static const double SMOOTHNESS_HUBER;
    CvMat *tmpImage;

 public:
    svlSecondOrderSuperResolutionMRF(int w, int h) : svlSuperResolution(w, h),
        svlOptimizer(w * h), tmpImage(NULL) {
	    // do nothing
    }
    ~svlSecondOrderSuperResolutionMRF();

    // definition of objective function and gradient
    double objective(const double *x);
    void gradient(const double *x, double *df);
    double objectiveAndGradient(const double *x, double *df);

    inline double getDepth(unsigned i) const { return (*this)[i]; }
    inline void setDepth(unsigned i, double d) { (*this)[i] = d; }

protected:
    // callback for each iteration during optimization (if bVerbose is true)
    void monitor(unsigned iter, double objValue);

    inline double measurementPotential(double x, double z, float w) const;
    inline double dMeasurementPotential(double x, double z, float w) const;
    inline double fdMeasurementPotential(double x, double z, float w, double *d) const;

    inline double firstOrderPotential(double x_i, double x_j, float w_ij) const;
    inline void dFirstOrderPotential(double x_i, double x_j, float w_ij,
	double *d_i, double *d_j) const;
    inline double fdFirstOrderPotential(double x_i, double x_j, float w_ij,
	double *d_i, double *d_j) const;

    inline double secondOrderPotential(double x_i, double x_j, double x_k, float w_i) const;
    inline void dSecondOrderPotential(double x_i, double x_j, double x_k, float w_i,
	double *d_i, double *d_j, double *d_k) const;
    inline double fdSecondOrderPotential(double x_i, double x_j, double x_k, float w_i,
	double *d_i, double *d_j, double *d_k) const;
};

