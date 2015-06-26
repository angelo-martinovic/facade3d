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
** FILENAME:    svlCameraIntrinsics.h
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
** DESCRIPTION:
**   OpenCV code for undistorting an image.
**
*****************************************************************************/

#pragma once

#include "cv.h"
#include <iostream>
#include <vector>

#include "cxcore.h"

#include "xmlParser/xmlParser.h"

#include "svlPoint3d.h"

// Camera Intrinsics Class

class svlCameraIntrinsics {
public:
    double focalLength[2];      // fc
    double principalPoint[2];   // cc
    double skew;                // alpha_c
    double distortion[4];       // kc

protected:
    CvMat* intrinsicMatrix;
    CvMat* distortionMatrix;

    CvMat* mapX;
    CvMat* mapY;
    CvMat* inverseMapX;
    CvMat* inverseMapY;
    CvMat* inverseMapX2;
    CvMat* inverseMapY2;
    CvMat* convergenceMap2;

    IplImage *imageBuffer;
    CvMat *rayBuffer;

public:
    svlCameraIntrinsics();
    svlCameraIntrinsics(const svlCameraIntrinsics& c);
    ~svlCameraIntrinsics();

    // load parameters from stream and intialize matrices
    bool initialize(const char *filename, const char *node = "intrinsics");
    bool initialize(XMLNode* node);
    bool initialize(std::istream&);
    bool initialize(const std::vector<double>&);
    bool initialize(double fc_x, double fc_y, double cc_x, double cc_y,
		    double alpha_c = 0.0, double kc_0 = 0.0, double kc_1 = 0.0,
		    double kc_2 = 0.0, double kc_3 = 0.0);
    void rescale(double factor = 0.5);

    // undistort image (do not free the returned image)
    const IplImage *undistort(const IplImage *);

    // ray for an image point wrt camera center
    const CvMat *ray(double x, double y, bool bNormalize = true);
    svlPoint3d ray_pt(double x, double y, bool bNormalize = true);

    // image point for a given ray
    CvPoint point(const CvMat *ray) const;
    std::pair<int,int> point(const CvMat *ray, bool) const;
    CvPoint point(const svlPoint3d &pt3) const;
    std::pair<int,int> point(const svlPoint3d &pt3, bool) const;

    // compute calibrated (undistorted) X and Y for given depth map, Z.
    // X and Y must have been previously allocated and the same size as Z.
    // all matrices should be CV_32FC1.
    void calibratedXY(const CvMat *Z, CvMat *X, CvMat *Y);

    // calibrated (undistorted) unit rays for each pixel coordinate
    void unitRays(CvMat *X, CvMat *Y, CvMat *Z);

    // operators
    svlCameraIntrinsics& operator=(const svlCameraIntrinsics& c);

private:
    void createMaps(int width, int height);
    void freeMaps();
};
