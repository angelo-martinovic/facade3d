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
** FILENAME:    svlConvolution.h
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
**              Olga Russakovsky <olga@cs.stanford.edu>
** DESCRIPTION:
**  Defines interface for convolving an image with a large non-separable
**  kernel function. Also provides a concrete example for typical image
**  filters.
**  
**  Gabor filters:
**      G_r(x,y) = exp(-(X^2 + \gama^2 Y^2) / 2\sigma^2) * cos((2\pi)/(\lambda) * X)
**      G_i(x,y) = exp(-(X^2 + \gama^2 Y^2) / 2\sigma^2) * cos((2\pi)/(\lambda) * Y)
**  where X = x cos(\theta) + y sin(\theta) and Y = -x sin(\theta) + y cos(\theta)
**  with orientation \theta (in radians), aspect ratio \gamma, effective width
**  \sigma and spatial frequency \lambda.
**  
**  Laplacian of Gaussian filters:
**      LoG(x, y) = 1/(\pi * \sigma^4) (r^2 - 1) exp(-r^2)
**  where r^2 = (x^2 + y^2)/(2 \sigma^2).
**
*****************************************************************************/

#pragma once

#include "cv.h"
#include "cxcore.h"

// svlConvolution --------------------------------------------------------------

class svlConvolution {
 public:
    static bool bUseDFTMethod;

 protected:
    CvMat *_kernel;
    CvPoint _anchor;

 public:
    svlConvolution(const CvMat *k = NULL, const CvPoint a = cvPoint(-1, -1));
    svlConvolution(const svlConvolution& c);
    virtual ~svlConvolution();

    // configuration and access functions
    bool readKernel(const char *filename);
    bool writeKernel(const char *filename) const;

    bool initialized() const { return (_kernel != NULL); }
    int width() const { return (_kernel == NULL ? 0 : _kernel->cols); }
    int height() const { return (_kernel == NULL ? 0 : _kernel->rows); }

    // Filtering function. The caller must provide CV32F destination image the
    // same size as the source image. If the source image is not single channel,
    // 32-bit it will be converted (wasting some time).
    // The image is being temporarily downsized to ensure height % 8 == 0 in order
    // to be able to use the opencv cvFilter2D function; if padWithZeros is true,
    // the image is instead temporarily increased in size and padded with zeros.
    // This changes the expected edge effects on the right/bottom parts of the image,
    // but is necessary for very small images (< 8 in height)
    void filter(IplImage *src, IplImage *dst, bool padWithZeros = false);

    // normalize the filter to mean zero, variance one
    void normalize(float mu = 0.0, float sigma = 1.0);

    // produces a visualization of the kernel. Useful for debugging. The calling
    // function must free the returned image.
    IplImage *visualize() const;
};

// svlGaborConvolution ---------------------------------------------------------

class svlGaborConvolution : public svlConvolution {
 public:
    svlGaborConvolution(int w, int h, float theta, float gamma,
        float sigma, float lambda);
    ~svlGaborConvolution();
};

// svlLoGConvolution -----------------------------------------------------------

// very accurate but potentially very slow
#define kernelSize(sigma) (2*(int)(sigma * 3) + 1) 

class svlLoGConvolution : public svlConvolution {
 public:
    svlLoGConvolution(int w, int h, float sigma);
    ~svlLoGConvolution();
};


