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
** FILENAME:    svlCameraExtrinsics.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
**
*****************************************************************************/

#include <cstdio>
#include <cstdlib>
#include <cassert>
#include <vector>
#include <iostream>
#include <fstream>

#include <cv.h>
#include <cxcore.h>
#include <highgui.h>

#include "svlBase.h"
#include "svlCameraExtrinsics.h"

using namespace std;

//----------------------------------------------------------------------------
// Camera Extrinsics Class
//----------------------------------------------------------------------------

svlCameraExtrinsics::svlCameraExtrinsics()
{
    R = cvCreateMat(3, 3, CV_32FC1);
    cvZero(R);
    cvmSet(R, 0, 0, 1.0);
    cvmSet(R, 1, 1, 1.0);
    cvmSet(R, 2, 2, 1.0);
    t = cvCreateMat(3, 1, CV_32FC1);
    cvZero(t);
    invR = cvCloneMat(R);
    buffer = cvCloneMat(t);
}

svlCameraExtrinsics::svlCameraExtrinsics(const svlCameraExtrinsics& c)
{
    R = cvCloneMat(c.R);
    t = cvCloneMat(c.t);
    invR = cvCloneMat(c.invR);
    buffer = cvCloneMat(c.buffer);
}

svlCameraExtrinsics::~svlCameraExtrinsics()
{
    cvReleaseMat(&R);
    cvReleaseMat(&t);
    cvReleaseMat(&invR);
    cvReleaseMat(&buffer);
}

// load parameters from stream
bool svlCameraExtrinsics::initialize(const char *filename, const char *node)
{
    XMLNode root = XMLNode::parseFile(filename);
    if (root.isEmpty()) {
	SVL_LOG(SVL_LOG_FATAL, "could not parse XML configuration file");
	return false;
    }

    if (root.nChildNode(node) == 0) {
	SVL_LOG(SVL_LOG_FATAL, "could not find \"" << node << "\" node");
	return false;
    }

    XMLNode child = root.getChildNode(node);
    return initialize(&child);
}

bool svlCameraExtrinsics::initialize(XMLNode *node)
{    
    SVL_ASSERT(node != NULL);

    vector<double> v;
    if (node->nChildNode("rotation") != 1) {
	SVL_LOG(SVL_LOG_FATAL, "could not parse rotation");
	return false;
    }

    parseString(string(node->getChildNode("rotation").getText()), v);
    SVL_ASSERT(v.size() == 9);

    if (node->nChildNode("translation") != 1) {
	SVL_LOG(SVL_LOG_FATAL, "could not parse translation");
	return false;
    }

    parseString(string(node->getChildNode("translation").getText()), v);
    SVL_ASSERT(v.size() == 12);

    return initialize(v);
}

bool svlCameraExtrinsics::initialize(std::istream& is)
{
    vector<double> v;
    v.resize(12);
    for (int i = 0; i < (int)v.size(); i++) {
        is >> v[i];
    }

    return this->initialize(v);
}

bool svlCameraExtrinsics::initialize(const vector<double>& v)
{
    if (v.size() != 12) {
        return false;
    }

    int k = 0;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            cvmSet(R, i, j, v[k++]);
        }
    }

    for (int i = 0; i < 3; i++) {
        cvmSet(t, i, 0, v[k++]);
    }

    cvInvert(R, invR);

    return true;
}

bool svlCameraExtrinsics::initialize(double R11, double R12, double R13,
				  double R21, double R22, double R23,
				  double R31, double R32, double R33,
				  double t1, double t2, double t3)
{
    // set rotation matrix
    cvmSet(R, 0, 0, R11);
    cvmSet(R, 0, 1, R12);
    cvmSet(R, 0, 2, R13);
    cvmSet(R, 1, 0, R21);
    cvmSet(R, 1, 1, R22);
    cvmSet(R, 1, 2, R23);
    cvmSet(R, 2, 0, R31);
    cvmSet(R, 2, 1, R32);
    cvmSet(R, 2, 2, R33);

    // set translation vector
    cvmSet(t, 0, 0, t1);
    cvmSet(t, 1, 0, t2);
    cvmSet(t, 2, 0, t3);

    cvInvert(R, invR);

    return true;
}

void svlCameraExtrinsics::setTranslation(double x, double y, double z)
{
    cvmSet(t, 0, 0, x);
    cvmSet(t, 1, 0, y);
    cvmSet(t, 2, 0, z);    
}

void svlCameraExtrinsics::setRotation(double theta, double phi, double psi)
{
    cvmSet(R, 0, 0, cos(theta) * cos(phi));
    cvmSet(R, 0, 1, -cos(theta) * sin(phi) * sin(psi) - sin(theta) * cos(psi));
    cvmSet(R, 0, 2, -cos(theta) * sin(phi) * cos(psi) + sin(theta) * sin(psi));
    cvmSet(R, 1, 0, sin(theta) * cos(phi));
    cvmSet(R, 1, 1, -sin(theta) * sin(phi) * sin(psi) + cos(theta) * cos(psi));
    cvmSet(R, 1, 2, -sin(theta) * sin(phi) * cos(psi) - cos(theta) * sin(psi));
    cvmSet(R, 2, 0, sin(phi));
    cvmSet(R, 2, 1, cos(phi) * sin(psi));
    cvmSet(R, 2, 2, cos(phi) * cos(psi));

    cvInvert(R, invR);    
}

void svlCameraExtrinsics::translate(double x, double y, double z)
{
    cvmSet(t, 0, 0, cvmGet(t, 0, 0) + x);
    cvmSet(t, 1, 0, cvmGet(t, 1, 0) + y);
    cvmSet(t, 2, 0, cvmGet(t, 2, 0) + z);
}

// rotate about y-axis
void svlCameraExtrinsics::pan(double theta)
{
    // use invR for storing the rotation matrix
    cvZero(invR);
    cvmSet(invR, 0, 0, cos(theta));
    cvmSet(invR, 0, 2, sin(theta));
    cvmSet(invR, 1, 1, 1.0);
    cvmSet(invR, 2, 0, -sin(theta));
    cvmSet(invR, 2, 2, cos(theta));
    cvMatMul(invR, R, R);

    cvInvert(R, invR);
}

// rotate about x-axis
void svlCameraExtrinsics::tilt(double theta)
{
    cvZero(invR);
    cvmSet(invR, 0, 0, 1.0);
    cvmSet(invR, 1, 1, cos(theta));
    cvmSet(invR, 1, 2, sin(theta));
    cvmSet(invR, 2, 1, -sin(theta));
    cvmSet(invR, 2, 2, cos(theta));
    cvMatMul(invR, R, R);

    cvInvert(R, invR);
}

// rotate about z-axis
void svlCameraExtrinsics::roll(double theta)
{
    cvZero(invR);
    cvmSet(invR, 0, 0, cos(theta));
    cvmSet(invR, 0, 1, sin(theta));
    cvmSet(invR, 1, 0, -sin(theta));
    cvmSet(invR, 1, 1, cos(theta));
    cvmSet(invR, 2, 2, 1.0);
    cvMatMul(invR, R, R);

    cvInvert(R, invR);
}

// point transforms
// p_camera = R * s * p_world + t_camera
// p_world = R^{-1} * (s * p_camera - t_camera)
const CvMat *svlCameraExtrinsics::world2camera(const CvMat *pt, double s)
{
    SVL_ASSERT((pt != NULL) && (pt->rows == 3));
    cvScale(pt, buffer, s);
    cvMatMul(R, buffer, buffer);
    cvAdd(buffer, t, buffer);
    return buffer;
}

const CvMat *svlCameraExtrinsics::world2camera(double x, double y, double z, double s)
{
    cvmSet(buffer, 0, 0, s * x);
    cvmSet(buffer, 1, 0, s * y);
    cvmSet(buffer, 2, 0, s * z);

    cvMatMul(R, buffer, buffer);
    cvAdd(buffer, t, buffer);
    return buffer;
}

const CvMat *svlCameraExtrinsics::world2camera(svlPoint3d &pt, double s)
{
    cvmSet(buffer, 0, 0, s * pt.x);
    cvmSet(buffer, 1, 0, s * pt.y);
    cvmSet(buffer, 2, 0, s * pt.z);

    cvMatMul(R, buffer, buffer);
    cvAdd(buffer, t, buffer);
    return buffer;
}

const CvMat *svlCameraExtrinsics::world2camera(const svlPoint3d &pt, double s)
{
    cvmSet(buffer, 0, 0, s * pt.x);
    cvmSet(buffer, 1, 0, s * pt.y);
    cvmSet(buffer, 2, 0, s * pt.z);

    cvMatMul(R, buffer, buffer);
    cvAdd(buffer, t, buffer);
    return buffer;
}

svlPoint3d svlCameraExtrinsics::world2camera_pt(svlPoint3d &pt, double s)
{
    world2camera(pt, s);

    return svlPoint3d(cvmGet(buffer,0,0),
	cvmGet(buffer,1,0),
	cvmGet(buffer,2,0));
}

const CvMat *svlCameraExtrinsics::worldRot2camera(const CvMat *pt, double s)
{
    // apply rotation but not translation
    SVL_ASSERT((pt != NULL) && (pt->rows == 3));
    cvScale(pt, buffer, s);
    cvMatMul(R, buffer, buffer);
    return buffer;
}

const CvMat *svlCameraExtrinsics::worldRot2camera(double x, double y, double z, double s)
{
    // apply rotation but not translation
    cvmSet(buffer, 0, 0, s * x);
    cvmSet(buffer, 1, 0, s * y);
    cvmSet(buffer, 2, 0, s * z);

    cvMatMul(R, buffer, buffer);
    return buffer;
}

const CvMat *svlCameraExtrinsics::worldRot2camera(const svlPoint3d &pt, double s)
{
    // apply rotation but not translation
    return worldRot2camera(pt.x, pt.y, pt.z, s);
}

svlPoint3d svlCameraExtrinsics::worldRot2camera_pt(svlPoint3d &pt, double s)
{
    // apply rotation but not translation
    worldRot2camera(pt, s);
    return svlPoint3d(cvmGet(buffer,0,0),
	cvmGet(buffer,1,0),
	cvmGet(buffer,2,0));
}

const CvMat *svlCameraExtrinsics::camera2world(const CvMat *pt, double s)
{
    SVL_ASSERT((pt != NULL) && (pt->rows == 3));
    cvScale(pt, buffer, s);
    cvSub(buffer, t, buffer);
    cvMatMul(invR, buffer, buffer);
    return buffer;
}

const CvMat *svlCameraExtrinsics::camera2world(double x, double y, double z, double s)
{
    cvmSet(buffer, 0, 0, s * x);
    cvmSet(buffer, 1, 0, s * y);
    cvmSet(buffer, 2, 0, s * z);

    cvSub(buffer, t, buffer);
    cvMatMul(invR, buffer, buffer);
    return buffer;
}

const CvMat *svlCameraExtrinsics::camera2world(const svlPoint3d& pt, double s)
{
    return camera2world(pt.x, pt.y, pt.z, s);
}


svlPoint3d svlCameraExtrinsics::camera2world_pt(const CvMat *pt, double s)
{
    camera2world(pt, s);
    return svlPoint3d(cvmGet(buffer,0,0),
	cvmGet(buffer,1,0),
	cvmGet(buffer,2,0));
}

svlPoint3d svlCameraExtrinsics::camera2world_pt2pt(svlPoint3d &pt, double s)
{
    camera2world(pt.x, pt.y, pt.z, s);
    return svlPoint3d(cvmGet(buffer,0,0),
	cvmGet(buffer,1,0),
	cvmGet(buffer,2,0));
}

svlCameraExtrinsics& svlCameraExtrinsics::operator=(const svlCameraExtrinsics& c)
{
    cvCopy(c.R, R);
    cvCopy(c.t, t);
    cvCopy(c.invR, invR);
    cvCopy(c.buffer, buffer);

    return *this;
}

