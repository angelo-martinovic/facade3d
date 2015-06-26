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
** FILENAME:    svlPointCloudData.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
**
*****************************************************************************/

#include <cstdio>
#include <cmath>
#include <vector>
#include <iostream>
#include <fstream>
#include <assert.h>
#include <limits.h>

#include "svlBase.h"
#include "svlVision.h"

using namespace std;

bool svlPointCloudData::LASER_VERTICAL = false;
double svlPointCloudData::LASER_OFFSET_X = 0.0381;
double svlPointCloudData::LASER_OFFSET_Y = 0.135;
double svlPointCloudData::LASER_OFFSET_Z = 0.05;
double svlPointCloudData::LASER_TILT = 0.03;
double svlPointCloudData::LASER_ROLL = 0.0;
double svlPointCloudData::FIELD_OF_VIEW = 45;

svlPointCloudData::svlPointCloudData(unsigned maxPts) :
    currentHeading(0.0), index(0), maxPoints(maxPts),
    firstTimestamp(-1.0), lastTimestamp(-1.0)
{
    // do nothing
}

svlPointCloudData::~svlPointCloudData()
{
    // do nothing
}

// Converts laser scan line into 3d points in the laser coordinate
// system (i.e. x-component is always 0.0). Also removes outliers.
// Ignores the first ofs points
#undef USE_STATIC_TABLES
void svlPointCloudData::laser2points(const vector<double> &laser, 
				     vector<svlPoint3d> &points, vector<double> &weights, int ofs)
{
#ifdef USE_STATIC_TABLES
    // static sin and cos tables for faster processing
    static vector<double> SIN_TABLE;
    static vector<double> COS_TABLE;
#endif

    // TO DO [SG]: tidy up constants
    double startAngle = -0.5 * M_PI + LASER_TILT;
    double endAngle = 0.5 * M_PI + LASER_TILT;
    double deltaAngle = (endAngle - startAngle) / (double)(laser.size() - 1);
    //const double minRangeUpper = 1.0;
    //const double minRangeLower = 1.5;
    const double minRangeUpper = 0.5;
    const double minRangeLower = 0.5;
    const double maxRange = 7.5;

#ifdef USE_STATIC_TABLES
    // preprocess cos() and sin() for faster appending
    if (SIN_TABLE.size() != laser.size()) {
        SIN_TABLE.resize(laser.size());
        COS_TABLE.resize(laser.size());
        for (unsigned i = 0; i < laser.size(); i++) {
            SIN_TABLE[i] = sin(startAngle + i * deltaAngle);
            COS_TABLE[i] = cos(startAngle + i * deltaAngle);
        }
    }

    // process laser scan
    points.reserve(laser.size()-ofs);
    weights.reserve(laser.size()-ofs);
    unsigned i;
    if (LASER_VERTICAL) {
      for (i = ofs; i < (laser.size()-ofs) / 4; i++) {
	    if ((laser[i] >= minRangeLower) && (laser[i] <= maxRange)) {
		points.push_back(svlPoint3d(0.0, -SIN_TABLE[i] * laser[i],
				     COS_TABLE[i] * laser[i]));
		weights.push_back(points.back().norm2());
	    }
	}
	for (; i < laser.size() - ofs; i++) {
	    if ((laser[i] >= minRangeUpper) && (laser[i] <= maxRange)) {
		points.push_back(svlPoint3d(0.0, -SIN_TABLE[i] * laser[i],
				     COS_TABLE[i] * laser[i]));
		weights.push_back(points.back().norm2());
	    }
	}
    } else {
      for (i = ofs; i < (laser.size()-ofs) / 4; i++) {
	    if ((laser[i] >= minRangeLower) && (laser[i] <= maxRange)) {
		points.push_back(svlPoint3d(-SIN_TABLE[i] * laser[i], 0.0,
				     COS_TABLE[i] * laser[i]));
		weights.push_back(points.back().norm2());
	    }
	}
	for (; i < laser.size() - ofs; i++) {
	    if ((laser[i] >= minRangeUpper) && (laser[i] <= maxRange)) {
		points.push_back(svlPoint3d(-SIN_TABLE[i] * laser[i], 0.0,
				     COS_TABLE[i] * laser[i]));
		weights.push_back(points.back().norm2());
	    }
	}
    }
#else
    // process laser scan
    points.reserve(laser.size()-ofs);
    weights.reserve(laser.size()-ofs);
    unsigned i;
    if (LASER_VERTICAL) {
      for (i = ofs; i < (laser.size()-ofs) / 4; i++) {
	    if (fabs(startAngle + i * deltaAngle) > M_PI * FIELD_OF_VIEW / 180.0)
		continue;
	    if ((laser[i] >= minRangeLower) && (laser[i] <= maxRange)) {
		points.push_back(svlPoint3d(0.0, -sin(startAngle + i * deltaAngle) * laser[i],
				     cos(startAngle + i * deltaAngle) * laser[i]));
		weights.push_back(points.back().norm2());
	    }
	}
      for (; i < laser.size() - ofs; i++) {
	if (fabs(startAngle + i * deltaAngle) > M_PI * FIELD_OF_VIEW / 180.0)
	  continue;
	if ((laser[i] >= minRangeUpper) && (laser[i] <= maxRange)) {
	  points.push_back(svlPoint3d(0.0, -sin(startAngle + i * deltaAngle) * laser[i],
				      cos(startAngle + i * deltaAngle) * laser[i]));
	  weights.push_back(points.back().norm2());
	}
      }
    } else {
      for (i = ofs; i < (laser.size() - ofs) / 4; i++) {
	    if (fabs(startAngle + i * deltaAngle) > M_PI * FIELD_OF_VIEW / 180.0)
		continue;
	    if ((laser[i] >= minRangeLower) && (laser[i] <= maxRange)) {
		points.push_back(svlPoint3d(-sin(startAngle + i * deltaAngle) * laser[i], 0.0,
				     cos(startAngle + i * deltaAngle) * laser[i]));
		weights.push_back(points.back().norm2());
	    }
	}
	for (; i < laser.size() - ofs; i++) {
	    if (fabs(startAngle + i * deltaAngle) > M_PI * FIELD_OF_VIEW / 180.0)
		continue;
	    if ((laser[i] >= minRangeUpper) && (laser[i] <= maxRange)) {
		points.push_back(svlPoint3d(-sin(startAngle + i * deltaAngle) * laser[i], 0.0,
				     cos(startAngle + i * deltaAngle) * laser[i]));
		weights.push_back(points.back().norm2());
	    }
	}
    }
#endif

    // correct for laser roll
    if (LASER_ROLL != 0.0) {
	for (unsigned i = 0; i < points.size(); i++) {
	    points[i].roll(LASER_ROLL);
	}
    }
}


void svlPointCloudData::appendIntegratedScan(const vector<double> & reading, double timestamp)
{
  vector<double> dummyOdometry(3);

  for (int i = 0; i < 3; i++)
    dummyOdometry[i] = 0;

  vector<double> ptu(2);
  
  for (int i = 0; i < 2; i++)
    ptu[i] = reading[i];

  appendScan(reading,ptu,dummyOdometry,timestamp,2);

}

// Expects laser data of any length, pan-tilt-unit data containing
// pan and tilt in radians, and odometry data containing x, y and
// heading in radians.
// Ignores the first ofs members of laser vector
void svlPointCloudData::appendScan(const vector<double> &laser, 
    const vector<double> &ptu,
    const vector<double> &odometry,
    double timestamp, int ofs)
{
    SVL_ASSERT(ptu.size() == 2);
    SVL_ASSERT(odometry.size() == 3);

    // update timestamps
    if (firstTimestamp < 0.0) {
        firstTimestamp = timestamp;
    }
    lastTimestamp = timestamp;

    // update heading
    currentHeading = 180.0 * odometry[2] / M_PI;

    // convert distances to laser coordinates (removing outliers)
    vector<svlPoint3d> v;
    vector<double> weights;
    laser2points(laser, v, weights, ofs);
    if (v.empty())
        return;

    // make sure vectors have enough space to store new data
    if (pointCloud.size() < maxPoints) {
        if (index + v.size() > maxPoints) {
            pointCloud.resize(maxPoints);
            pointNormals.resize(maxPoints);
            pointColours.resize(maxPoints);
            pointWeights.resize(maxPoints);
        } else {
            pointCloud.resize(index + v.size());
            pointNormals.resize(index + v.size());
            pointColours.resize(index + v.size());
            pointWeights.resize(index + v.size());
        }
    }

    // compute normal vector (in laser frame)
    svlPoint3d n;
    for (unsigned i = 1; i < v.size() - 1; i++) {
        n.x = 0.0;
        n.y = v[i + 1].z - v[i - 1].z;
        n.z = v[i - 1].y - v[i + 1].y;
        n /= sqrt(n.norm2());
        // error to line fit is (n' * (v[i] - v[i-1]))
        if (i == 1) pointNormals[(index + i - 1) % maxPoints] = n;
        pointNormals[(index + i - 1) % maxPoints] = n;
        if (i == v.size() - 2) pointNormals[(index + i - 1) % maxPoints] = n;
    }
    if (v.size() == 1) {
        pointNormals[index % maxPoints] = svlPoint3d();
    }
    if (v.size() == 2) {
        pointNormals[index % maxPoints] = svlPoint3d();
        pointNormals[(index + 1) % maxPoints] = svlPoint3d();
    }

    // convert points to world coordinates
    double theta = odometry[2] - ptu[0];
    double phi = -ptu[1];
    for (unsigned i = 0; i < v.size(); i++) {
        v[i].x += LASER_OFFSET_X;
	v[i].y += LASER_OFFSET_Y;
	v[i].z += LASER_OFFSET_Z;
        v[i].pan(theta);
	v[i].tilt(phi);
        v[i] += svlPoint3d(-odometry[1], 0.0, odometry[0]);
    }

    // add points to point cloud
    for (unsigned i = 0; i < v.size(); i++) {
        pointCloud[index] = v[i];
        pointWeights[index] = weights[i];
        pointColours[index] = svlPoint3d(0.25);
        index = (index + 1) % maxPoints;
    }

    currentScan = v;
    robotPath.push_back(svlPoint3d(-odometry[1], 0.0, odometry[0]));

    SVL_ASSERT(pointNormals.size() == pointCloud.size());
    SVL_ASSERT(pointColours.size() == pointCloud.size());
    SVL_ASSERT(pointWeights.size() == pointCloud.size());
}

void svlPointCloudData::setCurrentColours(const vector<svlPoint3d> &colours)
{
    SVL_ASSERT(colours.size() == currentScan.size());

    for (unsigned i = 0; i < colours.size(); i++) {
        pointColours[(index - colours.size() + i +
	    pointColours.size()) % pointColours.size()] = colours[i];
    }
}

void svlPointCloudData::clear()
{
    index = 0;
    pointCloud.clear();
    robotPath.clear();
    currentScan.clear();
    pointWeights.clear();
    
    currentHeading = 0.0;

    pointNormals.clear();
    pointColours.clear();
    
    amplitudes.clear();
}

bool svlPointCloudData::write(const char *filename, bool bPointsOnly) const
{
    SVL_ASSERT(filename != NULL);
    ofstream ofs(filename);
    if (ofs.fail()) {
        return false;
    }

    for (unsigned i = 0; i < pointCloud.size(); i++) {
	    ofs << pointCloud[i];
        if (!bPointsOnly) {
            ofs << " " << pointNormals[i]
		<< " " << pointColours[i]
		<< " " << pointWeights[i];
        }
        ofs << "\n";
    }
    ofs.close();
    return true;
}

bool svlPointCloudData::read(const char * filename, bool bPointsOnly, int dataWidth)
{
   SVL_ASSERT(filename != NULL);
   clear();
   bool result = import(filename, bPointsOnly, dataWidth);
   return result;
}

bool svlPointCloudData::import(const char * filename, bool bPointsOnly, int dataWidth)
{
   SVL_ASSERT(filename != NULL);

   ifstream ifs(filename);
   if (ifs.fail()) {
       return false;
   }
 
   char test = ifs.get();

   if (test == '%') {
       return readSeparate(ifs);
   } else {
       ifs.putback(test);
       return readBundled(ifs, bPointsOnly, dataWidth);
   }

   return true;
}


bool svlPointCloudData::readSeparate(ifstream &ifs)
{
  pointColours.clear();

  vector<double> xCoords;
  vector<double> yCoords;
  vector<double> zCoords;

  //Skip over the distance header
  ifs.ignore(numeric_limits<int>::max(), '\n');
  if (ifs.eof())
      return false;

  //Read the distances
  //[IG] Note: Though the file calls these "distances" they are in fact z-coordinates.
  // I have checked. Treating them as ranges and transforming to z-coordinates introduces
  // curvature to the data.

  double v;
  while (ifs.peek() != '%') {
      ifs >> v;
      if (ifs.fail()) return false;
      zCoords.push_back(v);
  }

  //Skip over the x coordinate header
  ifs.ignore(numeric_limits<int>::max(), '\n');
  if (ifs.eof())
      return false;

  //Read the x coordinates
  while (ifs.peek() != '%') {
      ifs >> v;
      if (ifs.fail()) return false;
      xCoords.push_back(v);
  }

  //Skip over the y coordinate header
  ifs.ignore(numeric_limits<int>::max(), '\n');
  if (ifs.eof())
      return false;

  //Read the y coordinates
  while (ifs.peek() != '%') {
      ifs >> v;
      if (ifs.fail()) return false;
      yCoords.push_back(v);
  }

  //Skip over the amplitude header
  ifs.ignore(numeric_limits<int>::max(), '\n');
  if (ifs.eof())
      return false;
 
  //Read the amplitudes (no stop character to scan for)
  while (!ifs.fail()) {
      ifs >> v;
      amplitudes.push_back(v / 10000.0);
  }

  ifs.close();

  //Synthesize xCoord, yCoord, and zCoord into points;
  //Convert to right handed coordinates

  SVL_ASSERT (xCoords.size() == yCoords.size());
  SVL_ASSERT (yCoords.size() == zCoords.size());
  SVL_ASSERT (zCoords.size() == amplitudes.size());
  
  for (int i = 0; i < (int)xCoords.size(); i++) {
      pointCloud.push_back(svlPoint3d(-xCoords[i], -yCoords[i], zCoords[i]));
  }
  
  return true;
}


bool svlPointCloudData::readBundled(ifstream &ifs, bool bPointsOnly, int dataWidth)
{
    svlPoint3d p;
    double d;
    while (!ifs.eof() && !ifs.fail()) {
        ifs >> p.x >> p.y >> p.z;
        if (ifs.fail())
            break;
        pointCloud.push_back(p);
        if (bPointsOnly) {
	    pointNormals.push_back(svlPoint3d());
            pointColours.push_back(svlPoint3d());
            pointWeights.push_back(0.0);
	    // skip remaining fields
	    for (int i = 3; i < dataWidth; i++) {
		ifs >> d;
	    }
        } else {
            ifs >> p.x >> p.y >> p.z;
            pointNormals.push_back(p);        
            ifs >> p.x >> p.y >> p.z;
            pointColours.push_back(p);
            ifs >> d;
            pointWeights.push_back(d);
        }
        if (pointCloud.size() == maxPoints)
            break;
    }

    ifs.close();

#if 0
    setMaxPoints(maxPoints);
#endif

    SVL_ASSERT(pointNormals.size() == pointCloud.size());
    SVL_ASSERT(pointColours.size() == pointCloud.size());
    SVL_ASSERT(pointWeights.size() == pointCloud.size());
   
   return true;
}

unsigned svlPointCloudData::setMaxPoints(unsigned m) {
    maxPoints = m;

    if (pointCloud.size() > maxPoints) {
        pointCloud.erase(pointCloud.begin(), pointCloud.begin() + pointCloud.size() - maxPoints);
        pointNormals.erase(pointNormals.begin(), pointNormals.begin() + pointNormals.size() - maxPoints);
        pointColours.erase(pointColours.begin(), pointColours.begin() + pointColours.size() - maxPoints);
        pointWeights.erase(pointWeights.begin(), pointWeights.begin() + pointWeights.size() - maxPoints);
        index = 0;
    }

    return (unsigned)pointCloud.size();
}

pair <float, float> svlPointCloudData::getRange(int whichCoord) const
{
  if (pointCloud.size() == 0)
    {
      //TODO-- this should technically throw some sort of error since there is no defined range
      return pair<float,float>(0,0);
    }
  else
    {
      vector<svlPoint3d>::const_iterator i = pointCloud.begin(), last = pointCloud.end();

      float min,max;

      min = max = (float)i->getD(whichCoord);

      for (++i; i!=last; ++i)
	{
	  float coord = (float)i->getD(whichCoord);
	  if (coord < min)
	    min = coord;
	  if (coord > max)
	    max = coord;
	}
      return pair<float,float>(min,max);
    }
  SVL_ASSERT (false); //unreached
  return pair<float,float>(0,0); //Only included to suppress compiler warnings
}

pair<svlPoint3d, svlPoint3d> svlPointCloudData::getBoundingBox() const
{
    pair<svlPoint3d, svlPoint3d> boundingBox;
    
    if (pointCloud.empty())
	return boundingBox;

    boundingBox.first = boundingBox.second = *pointCloud.begin();
    for (vector<svlPoint3d>::const_iterator i = pointCloud.begin(); 
	 i != pointCloud.end(); ++i) {
	boundingBox.first.min_with(*i);
	boundingBox.second.max_with(*i);
    }

    return boundingBox;
}

vector<svlPoint3d> svlPointCloudData::points2RobotCoords() const
{
    vector<svlPoint3d> rotatedCloud;
    rotatedCloud.reserve(pointCloud.size());

    double theta = -currentHeading * M_PI / 180.0;
    double ctheta = cos(theta);
    double stheta = sin(theta);
    svlPoint3d offset;

    if (!robotPath.empty()) {
	offset = robotPath.back();
    }

    for (vector<svlPoint3d>::const_iterator it = pointCloud.begin();
	 it != pointCloud.end(); it++) {
	svlPoint3d p = (*it - offset).pan(ctheta, stheta);
	rotatedCloud.push_back(p);
    }

    return rotatedCloud;
}

