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
** FILENAME:    svlPointCloudData.h
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
** DESCRIPTION:
**  Data structure for holding laser point cloud data. The maximum number of
**  points can be set at which point the allocated vectors behave like a ring
**  buffer. This is more efficient than repeatedly allocating and deleting the
**  array. No locks are placed on the buffer so concurrent access should be
**  avoided by external means.
**
**  The point cloud is constructed on a right-handed coordinate system:
**    x = right (width)
**    y = down (-height)
**    z = into plane (depth)
**
*****************************************************************************/

#pragma once

#include <vector>
#include "svlVision.h"

using namespace std;

class svlPointCloudData
{
 public:
    svlPointCloudData(unsigned maxPts = 200000);
    virtual ~svlPointCloudData();

    static void laser2points(const vector<double> &laser, 
			     vector<svlPoint3d> &points, vector<double> &weights, int ofs);

    void appendIntegratedScan(const vector<double> & reading, double timestamp);

    void appendScan(const vector<double> &laser, 
		    const vector<double> &ptu,
		    const vector<double> &odometry,
		    double timestamp, int ofs = 0);

    void setCurrentColours(const vector<svlPoint3d> &colours);

    double getCurrentTime() const {
	return (lastTimestamp - firstTimestamp);
    }

    void clear();
    bool write(const char *filename, bool bPointsOnly = false) const;
    bool read(const char *filename, bool bPointsOnly = false, int dataWidth = 3);
    bool import(const char *filename, bool bPointsOnly = false, int dataWidth = 3);

    unsigned getMaxPoints() const { return maxPoints; }
    unsigned setMaxPoints(unsigned m);

    pair<float, float> getRange(int whichCoord) const;
    pair<svlPoint3d, svlPoint3d> getBoundingBox() const;

    // returns point cloud in robot coordinate system (using robotPath
    // and currentHeading)
    vector<svlPoint3d> points2RobotCoords() const;

 public:
    static bool LASER_VERTICAL;
    static double LASER_OFFSET_X;
    static double LASER_OFFSET_Y;
    static double LASER_OFFSET_Z;
    static double LASER_TILT;
    static double LASER_ROLL;
    static double FIELD_OF_VIEW;

    vector<svlPoint3d> pointCloud;
    vector<svlPoint3d> currentScan;
    vector<svlPoint3d> robotPath;
    vector<double> pointWeights;
    vector<double> amplitudes;
    
    double currentHeading;

    vector<svlPoint3d> pointNormals;
    vector<svlPoint3d> pointColours;

 protected:

    bool readSeparate(ifstream &ifs);
    bool readBundled(ifstream &ifs, bool bPointsOnly, int dataWidth);

    unsigned index;
    unsigned maxPoints;

    double firstTimestamp;
    double lastTimestamp;
};



