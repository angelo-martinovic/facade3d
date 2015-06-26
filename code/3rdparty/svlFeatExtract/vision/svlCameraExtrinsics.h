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
** FILENAME:    svlCameraExtrinsics.h
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
** DESCRIPTION:
**   OpenCV code for camera extrinsics. The extrinsics are represented by
**   a rotation matrix, R, and translation vector, t, such that
**       p_camera = R * p_world + t
**       p_world = R^{-1} * (p_camera - t)
**
*****************************************************************************/

#pragma once

#include <iostream>
#include <vector>

#include "cv.h"
#include "cxcore.h"

#include "xmlParser/xmlParser.h"
#include "svlPoint3d.h"

class svlCameraExtrinsics {
protected:
    CvMat *R;       // rotation wrt world coordinates
    CvMat *t;       // translation wrt world coordinates

    CvMat *invR;    // inverse rotation
    CvMat *buffer;  // internal point buffer

public:
    svlCameraExtrinsics();
    svlCameraExtrinsics(const svlCameraExtrinsics& c);
    ~svlCameraExtrinsics();

    // load parameters from stream or vector
    bool initialize(const char *filename, const char *node = "extrinsics");
    bool initialize(XMLNode *node);
    bool initialize(std::istream&);
    bool initialize(const std::vector<double>&);
    bool initialize(double R11, double R12, double R13,
		    double R21, double R22, double R23,
		    double R31, double R32, double R33,
		    double t1 = 0.0, double t2 = 0.0, double t3 = 0.0);
    void setTranslation(double x, double y, double z);
    void setRotation(double theta, double phi, double psi);

    // modifiers
    void translate(double x, double y, double z);
    void pan(double theta);  // rotate about y-axis
    void tilt(double theta); // rotate about x-axis
    void roll(double theta); // rotate about z-axis

    // access functions
    inline const CvMat *translation() const { return t; }
    inline const CvMat *rotation() const { return R; }

    // point transforms (don't free returned vector)
    const CvMat *world2camera(const CvMat *pt, double s = 1.0);
    const CvMat *world2camera(double x, double y, double z, double s = 1.0);
    const CvMat *world2camera(const svlPoint3d& pt, double s = 1.0);
	const CvMat *world2camera(svlPoint3d& pt, double s = 1.0);
	svlPoint3d world2camera_pt(svlPoint3d &pt, double s = 1.0);

    const CvMat *worldRot2camera(const CvMat *pt, double s = 1.0);
    const CvMat *worldRot2camera(double x, double y, double z, double s = 1.0);
    const CvMat *worldRot2camera(const svlPoint3d& pt, double s = 1.0);
	svlPoint3d worldRot2camera_pt(svlPoint3d &pt, double s = 1.0);

    const CvMat *camera2world(const CvMat *pt, double s = 1.0);
    const CvMat *camera2world(double x, double y, double z, double s = 1.0);
    const CvMat *camera2world(const svlPoint3d& pt, double s = 1.0);
	svlPoint3d camera2world_pt(const CvMat *pt, double s = 1.0);
	svlPoint3d camera2world_pt2pt(svlPoint3d &pt, double s = 1.0);

    // access functions for last transform (if you don't want
    // to mess around with CvMat data structures)
    inline double lastX() const { return cvmGet(buffer, 0, 0); }
    inline double lastY() const { return cvmGet(buffer, 1, 0); }
    inline double lastZ() const { return cvmGet(buffer, 2, 0); }

    svlCameraExtrinsics& operator=(const svlCameraExtrinsics& c);
};


