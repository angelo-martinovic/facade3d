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
** FILENAME:    objectList.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
**              Ian Goodfellow <ia3n@cs.stanford.edu>
**
*****************************************************************************/

#include <cstdlib>
#include <iostream>
#include <fstream>
#include <iterator>
#include <vector>
#include <cassert>
#include <string>

#include "xmlParser/xmlParser.h"

#include "svlBase.h"
#include "svlVision.h"

#include "svlObjectList.h"

// svlObject2d Class --------------------------------------------------------

svlObject2d::svlObject2d() :
  name("[unknown]"), x(0.0), y(0.0), w(1.0), h(1.0), pr(1.0), ang(0.0), index(-1), ignore(false)
{
    // do nothing
}

svlObject2d::svlObject2d(const svlObject2d& o) :
    name(o.name), x(o.x), y(o.y), w(o.w), h(o.h), pr(o.pr), ang(o.ang), index(o.index), ignore(false)
{
    // do nothing
}

svlObject2d::svlObject2d(const CvRect & o) :
  name("[unknown]"), x(o.x), y(o.y), w(o.width), h(o.height), pr(1.0), ang(0.0), index(-1), ignore(false)
{
  // do nothing
}

svlObject2d::svlObject2d(double nx, double ny, double s) :
    name("[unknown]"), x(nx), y(ny), w(s), h(s), pr(1.0), ang(0.0), index(-1), ignore(false)
{
    // do nothing
}

svlObject2d::svlObject2d(double nx, double ny, double nw, double nh, double p) :
    name("[unknown]"), x(nx), y(ny), w(nw), h(nh), pr(p), ang(0.0), index(-1), ignore(false)
{
    // do nothing
}

svlObject2d::svlObject2d(const svlPoint3d& u, const svlPoint3d& v, double p) :
    name("[unknown]"), x(u.x), y(u.y), w(v.x - u.x), h(v.y - u.y), pr(p), ang(0.0), index(-1), ignore(false)
{
    if (w < 0.0) { x = v.x; w = -w; }
    if (h < 0.0) { y = v.y; h = -h; }
}

svlObject2d::~svlObject2d()
{
    // do nothing
}

bool svlObject2d::hit(double qx, double qy) const
{
  if ( ang == 0.0 ) {
    return ((qx >= x) && (qx <= x + w) && (qy >= y) && (qy <= y + h));
  } else {
    // Check if point is within the bounds of the parallel edges.
    // Create anchor point (top left) and normal vector to first line segment (from top left to top right).
    double px = x, py = y, nx = sin(ang), ny = cos(ang), gd;
    // If point minus anchor, dotted with normal is out of bounds of rectangle, fail.
    if ( (gd = (qx-px)*nx + (qy-py)*ny) < 0.0 || gd > h )
      return false;
    // Move to upper-right corner (and rotate normal).
    px += w*cos(ang);
    py += -w*sin(ang);
    gd = nx; nx = -ny; ny = gd; // Equivalent to a -90 deg rotation.
    if ( (gd = (qx-px)*nx + (qy-py)*ny) < 0.0 || gd > w )
      return false;
    return true;
  }
}

double svlObject2d::overlap(const svlObject2d& o) const
{
  double ix, iy, iw, ih;  // intersection

  if ( ang == 0.0 && o.ang == 0.0 ) {
    // find region of overlap
    ix = (x < o.x) ? o.x : x;
    iy = (y < o.y) ? o.y : y;
    iw = ((x + w < o.x + o.w) ? x + w : o.x + o.w) - ix;
    ih = ((y + h < o.y + o.h) ? y + h : o.y + o.h) - iy;
    
    if ((iw < 0) || (ih < 0.0)) {
      return 0.0;
    }
    
    // return area of intersection
    return (iw * ih);
  } else {
    // Do separating axis test to see if we can return an easy 0.
    // Calculate vector between the two rectangles' anchor points.
    Vector2d iorg(x,y);
    Vector2d v(o.x - x, o.y - y);
    // Normalize.
    double gd = sqrt(v[0]*v[0] + v[1]*v[1]);
    v /= gd;
    // Construct points of the two rectangles.
    Vector2d is[4] = { Vector2d(x,y),
		       Vector2d(x+w*cos(ang),y-w*sin(ang)),
		       Vector2d(x+w*cos(ang)+h*sin(ang),y+h*cos(ang)-w*sin(ang)),
		       Vector2d(x+h*sin(ang),y+h*cos(ang)) };
    Vector2d js[4] = { Vector2d(o.x,o.y),
		       Vector2d(o.x+o.w*cos(o.ang),o.y-o.w*sin(o.ang)),
		       Vector2d(o.x+o.w*cos(o.ang)+o.h*sin(o.ang),o.y+o.h*cos(o.ang)-o.w*sin(o.ang)),
		       Vector2d(o.x+o.h*sin(o.ang),o.y+o.h*cos(o.ang)) };
    // Calculate max dot product along vector for this object, and min dot of other.
    double maxdot = max( max( 0.0, v.dot(is[1]-iorg) ), max( v.dot(is[2]-iorg), v.dot(is[3]-iorg) ) );
    double mindot = min( min( gd, v.dot(js[1]-iorg) ), min( v.dot(js[2]-iorg), v.dot(js[3]-iorg) ) );
    //printf("min,max: %0.3f %0.3f\n", mindot, maxdot);
    if ( maxdot < mindot )
      return 0.0;
    // Well, now we just have to do lots of math.
    
    // Calculate which of each vertices are within the other rectangle.
    bool hitother[4] = { o.hit(is[0]), o.hit(is[1]), o.hit(is[2]), o.hit(is[3]) }, // Wich of this's hit other.
      hitthis[4] = { hit(js[0]), hit(js[1]), hit(js[2]), hit(js[3]) };	// Which of other's hit this.
      // Some fast exits: check if one rectangle is entirely inside another.
      if ( hitthis[0] && hitthis[1] && hitthis[2] && hitthis[3] )
	return o.w*o.h;
      if ( hitother[0] && hitother[1] && hitother[2] && hitother[3] )
	return w*h;
      
      // Calculate all intersections between edges.
      bool this2other[4][4];
      memset(this2other, 0, sizeof(bool)*16);
      Vector2d inters[4][4], gv;
      // Compute starting normals for both i (this) and j (other) to rotate as we go.
      Vector2d ni(sin(ang),cos(ang)), nj(sin(o.ang),cos(o.ang));
      for ( unsigned i=0; i<4; ++i ) {
	for ( unsigned j=0; j<4; ++j ) {
	  // Intersection between this's (i -> i+1) edge and other's (j -> j+1) edge.
	  // Check if either's vertices both lie on opposite sides of edge using dot product with normal: if product of dot products is negative, signs disagree.
	  if ( ni.dot(js[j]-is[i]) * ni.dot(js[(j+1)%4]-is[i]) < 0.0
	       && nj.dot(is[i]-js[j]) * nj.dot(is[(i+1)%4]-js[j]) < 0.0 ) {
	    this2other[i][j] = true;
	    // Calculate intersection point.
	    gv = js[(j+1)%4] - js[j];
	    gv /= sqrtf( gv[0]*gv[0] + gv[1]*gv[1] );
	    //cout << "gv: " << gv << endl;
	    gd = ni.dot(is[i]-js[j]) / ni.dot(gv);
	    //cout << "gd: " << gd << endl;
	    inters[i][j] = js[j] + gd*gv;
	  }
	  nj = Vector2d(-nj[1],nj[0]);
	}
	// Rotate normals by 90 degrees.
	ni = Vector2d(-ni[1],ni[0]);
      }
      
      // Get starting point: either find first point of other in this ...
      vector<pair<int,int> > pts;
      for ( unsigned i=0; pts.empty() && i<4; ++i )
	if ( hitthis[i] )
	  pts.push_back(pair<int,int>(-1,i));
      // ... or an intersection.
      for ( unsigned i=0; pts.empty() && i<16; ++i )
	if ( this2other[i/4][i%4] )
	  pts.push_back(pair<int,int>(i/4,i%4));
      if ( pts.empty() ) {
	// No intersections or hits.
	return 0.0;
      }
      
#define LLAST_PASS(f,s) ( !llast || llast->first != f || llast->second != s )
      // Run this complicated algorithm that uses the above data structures to construct
      // the convex hull of the intersecting polygon, whose area is then easy to compute.
      int gi;
      while ( pts.size() <= 1 || pts.back() != pts.front() ) {
	pair<int,int> last = pts.back();
	pair<int,int> *llast = pts.size() < 2 ? NULL : &pts[pts.size()-2];
	//printf("@ %d,%d", last.first, last.second);
	//if ( llast ) printf(" (last: %d,%d)", llast->first, llast->second);
	//printf("\n");
	
	if ( last.first == -1 ) { // We're on other, in this.
	  // Jump to next vertex on other that's still in this.
	  if ( hitthis[ gi = (last.second+3)%4 ] && LLAST_PASS(-1,gi) ) {
	    pts.push_back(pair<int,int>(-1,gi));
	  } else if ( hitthis[ gi = (last.second+1)%4 ] && LLAST_PASS(-1,gi) ) {
	    pts.push_back(pair<int,int>(-1,gi));
	  } else {
	    // Check for the intersection to jump to.
	    gi = last.second;
	    for ( int i=0; i<4; ++i )
	      if ( this2other[i][gi] && LLAST_PASS(i,gi) ) {
		pts.push_back(pair<int,int>(i,gi));
		break;
	      }
	    if ( pts.back() == last ) {
	      // Check other leg.
	      gi = (last.second+3)%4;
	      for ( int i=0; i<4; ++i )
		if ( this2other[i][gi] && LLAST_PASS(i,gi) ) {
		  pts.push_back(pair<int,int>(i,gi));
		  break;
		}
	      if ( pts.back() == last ) {
		SVL_LOG(SVL_LOG_FATAL, "Algorithmic error");
	      }
	    }
	  }
	} else if ( last.second == -1 ) { // We're on this, in other.
	  // Jump to next vertex on this that's still in other.
	  if ( hitother[ gi = (last.first+3)%4 ] && LLAST_PASS(gi,-1) ) {
	    pts.push_back(pair<int,int>(gi,-1));
	  } else if ( hitother[ gi = (last.first+1)%4 ] && LLAST_PASS(gi,-1) ) {
	    pts.push_back(pair<int,int>(gi,-1));
	    /*if ( hitother[ gi = (last.first+1)%4 ] ) {
	      pts.push_back(pair<int,int>(gi,-1));*/
	  } else {
	    // Check for the intersection to jump to.
	    gi = last.first;
	    for ( int j=0; j<4; ++j )
	      if ( this2other[gi][j] && LLAST_PASS(gi,j) ) {
		pts.push_back(pair<int,int>(gi,j));
		break;
	      }
	    if ( pts.back() == last ) {
	      // Check other leg.
	      gi = (last.first+3)%4;
	      for ( int j=0; j<4; ++j )
		if ( this2other[gi][j] && LLAST_PASS(gi,j) ) {
		  pts.push_back(pair<int,int>(gi,j));
		  break;
		}
	      if ( pts.back() == last ) {
		SVL_LOG(SVL_LOG_FATAL, "Algorithmic error");
	      }
	    }
	  }
	} else { // At an intersection.
	  // Check for which next point to jump to.
	  if ( hitthis[ gi = last.second ] && LLAST_PASS(-1,gi) ) {
	    pts.push_back(pair<int,int>(-1,gi));
	  } else if ( hitthis[ gi = (last.second+1)%4 ] && LLAST_PASS(-1,gi) ) {
	    pts.push_back(pair<int,int>(-1,gi));
	  } else if ( hitother[ gi = last.first ] && LLAST_PASS(gi,-1) ) {
	    pts.push_back(pair<int,int>(gi,-1)); 
	  } else if ( hitother[ gi = (last.first+1)%4 ] && LLAST_PASS(gi,-1) ) {
	    pts.push_back(pair<int,int>(gi,-1)); 
	  } else {
	    // Check for another intersection.
	    for ( int i=0; i<4; ++i )
	      if ( i != last.first && this2other[i][last.second] && LLAST_PASS(i,last.second) ) {
		pts.push_back(pair<int,int>(i,last.second));
		break;
	      }
	    if ( pts.back() == last ) { // Check for other intersection to jump to.
	      for ( int j=0; j<4; ++j )
		if ( j != last.second && this2other[last.first][j] && LLAST_PASS(last.first,j) ) {
		  pts.push_back(pair<int,int>(last.first,j));
		  break;
		}
	      if ( pts.back() == last ) {
		SVL_LOG(SVL_LOG_FATAL, "Algorithm error");
	      }
	    }
	  }
	}
	// Max intersection is octagonal, with 1 extra point (first and last should be the same).
	if ( pts.size() > 9 ) {
	  SVL_LOG(SVL_LOG_FATAL, "Algorithm error: max valid size exceeded");
	}
      }
      
      // Collect the points of the intersection polygon.
      vector<const Vector2d*> P;
      P.reserve(pts.size());
      for ( unsigned i=0; i<pts.size(); ++i ) {
	if ( pts[i].first == -1 ) { // Other point
	  P.push_back(&js[pts[i].second]);
	} else if ( pts[i].second == -1 ) { // This point
	  P.push_back(&is[pts[i].first]);
	} else { // Intersection
	  P.push_back(&inters[pts[i].first][pts[i].second]);
	}
      }
      
      // Compute area of general polygon.
      Vector2d mean(0.0,0.0);
      for ( unsigned i=0; i<P.size()-1; ++i ) {
	//printf("%d: %0.3f,%0.3f\n", i, P[i][0], P[i][1]);
	mean += *P[i];
      }
      mean /= double(P.size()-1);
      //printf("mean: %0.3f,%0.3f\n", mean[0], mean[1]);
      double area = 0.0;
      // Add in area of sub-triangles.
      for ( unsigned i=0; i<P.size()-1; ++i ) {
	v = *P[i+1] - *P[i];
	v /= gd = sqrtf(v[0]*v[0]+v[1]*v[1]); // gd is now length.
	v = Vector2d(-v[1],v[0]); // Convert to normal.
	area += 0.5 * fabs( gd * v.dot(mean - *P[i]) );
      }
      return area;
  }
}

svlObject2d& svlObject2d::operator=(const svlObject2d& o)
{
  x = o.x; y = o.y; w = o.w; h = o.h; pr = o.pr; ang = o.ang;
  name = o.name;
  index = o.index;
  ignore = o.ignore;

  return *this;
}


// svlObject2dFrame Class --------------------------------------------------------

bool svlObject2dFrame::write(std::ostream & os, const char * id) const
{
    const svlObject2dFrame & v = *this;
    if (os.fail()) return false;
  
    os << "    <Object2dFrame id=\"" << (id == NULL ? "NULL" : id) << "\">" << endl;
    for (unsigned j = 0; j < v.size(); j++) {    
        os << "        <Object name=\"" << v[j].name.c_str() << "\""
           << " x=\"" << v[j].x << "\""
           << " y=\"" << v[j].y << "\""
           << " w=\"" << v[j].w << "\""
           << " h=\"" << v[j].h << "\""
	   << " pr=\"" << v[j].pr << "\"";
        if (v[j].index != -1) os << " index=\"" << v[j].index << "\"";
        if (v[j].ang != 0.0) os << " ang=\"" << v[j].ang << "\"";
        os << " ignore=\"" << v[j].ignore << "\" />" << "\n";
    }
    os << "    </Object2dFrame>" << endl;

    return true;
}

bool writeObject2dFrame(ostream &os, const svlObject2dFrame& v, const char *id)
{
    SVL_LOG(SVL_LOG_WARNING, "writeObject2dFrame is deprecated and may be removed after January 9, 2010. Use svlObject2dFrame::write instead.");
    return v.write(os, id);
}

bool writeCacheObject2dFrame(const char *outputDir,
			     const char *id,
			     const svlObject2dFrame& v) {
  string filename = string(outputDir) + "/" + id + ".txt";
  ofstream os(filename.c_str());
  if (os.fail()) {
    SVL_LOG(SVL_LOG_WARNING, "Couldn't open output directory");
    return false;
  }

  // to save space can actually not output height;
  // reduntant, since the object ratio must remain constant
  for (unsigned j = 0; j < v.size(); j++) {
    os << v[j].index << " "
       << v[j].x << " "
       << v[j].y << " "
       << v[j].w << " "
       << v[j].h << " "
       << v[j].pr << " "
       << v[j].ang << endl;
  }

  os.close();

  return true;
}

bool readCacheObject2dFrame(const char *outputDir,
			    const char *id,
			    svlObject2dFrame& v) {
  v.clear();

  string filename = string(outputDir) + "/" + id + ".txt";
  ifstream ifs(filename.c_str());
  if (ifs.fail()) {
    SVL_LOG(SVL_LOG_WARNING, "Couldn't read the cached file " << filename.c_str());
    return false;
  }

  while (ifs.good()) {
    svlObject2d obj;
    ifs >> obj.index;
    if (!ifs.good()) break;
    ifs >> obj.x >> obj.y >> obj.w >> obj.h >> obj.pr >> obj.ang;
    v.push_back(obj);
  }

  ifs.close();
  return true;
}

bool writeShortCacheObject2dFrame(const char *outputDir,
				  const char *id,
				  const int level,
				  const svlObject2dFrame& v) {
  stringstream filename;
  filename << outputDir << "/" << id << "." << level << ".txt";

  ofstream os(filename.str().c_str());
  if (os.fail()) {
    SVL_LOG(SVL_LOG_WARNING, "Couldn't open output directory");
    return false;
  }

  for (unsigned j = 0; j < v.size(); j++) {
    os << v[j].x << " "
       << v[j].y << " "
       << v[j].pr << endl;
  }

  os.close();

  return true;
}

bool readShortCacheObject2dFrame(const char *outputDir,
				 const char *id,
				 const int level,
				 svlObject2dFrame& v) {
  v.clear();

  stringstream filename;
  filename << outputDir << "/" << id << "." << level << ".txt";

  ifstream ifs(filename.str().c_str());
  if (ifs.fail()) {
    SVL_LOG(SVL_LOG_WARNING, "Couldn't read the short cached file " << filename.str().c_str());
    return false;
  }

  while (ifs.good()) {
    svlObject2d obj;
    ifs >> obj.x;
    if (!ifs.good()) break;
    ifs >> obj.y >> obj.pr;
    v.push_back(obj);
  }

  ifs.close();
  return true;
}


void scaleObject2dFrame(svlObject2dFrame &v, double scale) {
    for (unsigned j = 0; j < v.size(); j++) {
        v[j].scale(scale);
    }
}

int removeOverlappingObjects(svlObject2dFrame& frame, double threshold)
{
	int count = 0;
	for (unsigned i = 0; i < frame.size(); i++) {
		double areaA = frame[i].area();
		for (unsigned j = (unsigned)frame.size() - 1; j > i; j--) {
			if (frame[i].name != frame[j].name) {
				continue;
			}
			double areaOverlap = frame[i].overlap(frame[j]);
			double areaB = frame[j].area();
			if ((areaOverlap > threshold * areaA) &&
				(areaOverlap > threshold * areaB)) {
					if (areaB > areaA) {
						frame[i] = frame[j];		   
					}
					frame.erase(frame.begin() + j);
					count += 1;
			}
		}
	}

	return count;
}

int removeGroundTruthObjects(svlObject2dFrame& frame, const svlObject2dFrame& truth, double threshold)
{
	int count = 0;
	for (int i = (int)frame.size() - 1; i >= 0; i--) {
		double areaA = frame[i].area();
		for (int j = 0; j < (int)truth.size(); j++) {
			if (frame[i].name != truth[j].name) {
				continue;
			}
			double areaOverlap = frame[i].overlap(truth[j]);
			double areaB = truth[j].area();
			if ((areaOverlap > threshold * areaA) &&
				(areaOverlap > threshold * areaB)) {
					frame.erase(frame.begin() + i);
					count += 1;
					break;
			}
		}
	}

	return count;
}

int removeNonGroundTruthObjects(svlObject2dFrame& frame, const svlObject2dFrame& truth, double threshold)
{
	svlObject2dFrame keepList;

	keepList.reserve(frame.size());
	for (int i = (int)frame.size() - 1; i >= 0; i--) {
		double areaA = frame[i].area();
		for (int j = 0; j < (int)truth.size(); j++) {
			if (frame[i].name != truth[j].name) {
				continue;
			}
			double areaOverlap = frame[i].overlap(truth[j]);
			double areaB = truth[j].area();
			if ((areaOverlap > threshold * areaA) &&
				(areaOverlap > threshold * areaB)) {
					keepList.push_back(frame[i]);
					break;
			}
		}
	}

	int count = (int)(frame.size() - keepList.size());
	frame = keepList;

	return count;
}

int removeMatchingObjects(svlObject2dFrame& frame, const char *name)
{
	int count = 0;
	for (int i = (int)frame.size() - 1; i >= 0; i--) {
		if (frame[i].name == string(name)) {
			frame.erase(frame.begin() + i);
			count += 1;
		}
	}

	return count;
}

int removeMatchingObjects(svlObject2dFrame& frame, const std::set<std::string>& names)
{
	int count = 0;
	for (int i = (int)frame.size() - 1; i >= 0; i--) {
		if (names.find(frame[i].name) != names.end()) {
			frame.erase(frame.begin() + i);
			count += 1;
		}
	}

	return count;
}


int removeNonMatchingObjects(svlObject2dFrame& frame, const char *name)
{
    int count = 0;
    for (int i = (int)frame.size() - 1; i >= 0; i--) {
        if (frame[i].name != string(name)) {
	    frame.erase(frame.begin() + i);
	    count += 1;
        }
    }

    return count;
}

int removeNonMatchingObjects(svlObject2dFrame& frame, const std::set<std::string>& names)
{
    int count = 0;
    for (int i = (int)frame.size() - 1; i >= 0; i--) {
        if (names.find(frame[i].name) == names.end()) {
            frame.erase(frame.begin() + i);
            count += 1;
        }
    }
    
    return count;
}

int removeBelowProbability(svlObject2dFrame &frame, float threshold) {
  int count = 0;
  for (int i = (int)frame.size() -1; i >= 0; i--) {
    if (frame[i].pr < threshold) {
      frame.erase(frame.begin() + i);
      count++;
    }
  }
  return count;
}

int removeSmallObjects(svlObject2dFrame &frame, int minWidth, int minHeight) {
  int count = 0;
  for (int i = (int)frame.size() -1; i >= 0; i--) {
    if (frame[i].w < minWidth || frame[i].h < minHeight) {
      frame.erase(frame.begin() + i);
      count++;
    }
  }
  return count;
}

int keepMostProbableObjects(svlObject2dFrame &frame, int nKeep)
{
    if (frame.empty() || (nKeep <= 0)) {
        int nSize = (int)frame.size();
        frame.clear();
        return nSize;
    }

    // sort detections by score (highest first)
    vector<pair<double, int> > sortIndex;
    sortIndex.reserve(frame.size());
    for (unsigned i = 0; i < frame.size(); i++) {
        sortIndex.push_back(make_pair(-frame[i].pr, i));
    }
    stable_sort(sortIndex.begin(), sortIndex.end());
    
    // copy n highest scoring objects
    svlObject2dFrame sortedFrame;
    sortedFrame.reserve(nKeep);
    for (int i = 0; (i < (int)sortIndex.size()) && (i < nKeep); i++) {
        sortedFrame.push_back(frame[sortIndex[i].second]);
    }

    frame = sortedFrame;
    return (int)(sortIndex.size() - frame.size());
}

int removeOutsideObjects(svlObject2dFrame &frame, const CvRect& roi)
{
    int count = 0;
    for (int i = (int)frame.size() - 1; i >= 0; i--) {
        if ((frame[i].x < roi.x) || (frame[i].x + frame[i].w > roi.x + roi.width) ||
            (frame[i].y < roi.y) || (frame[i].y + frame[i].h > roi.y + roi.height)) {
            frame.erase(frame.begin() + i);
            count++;
        }
    }

    return count;    
}

int nonMaximalSuppression(svlObject2dFrame& frame, double dx, double dy, double ds, double dsArea)
{
    SVL_FCN_TIC;
    double threshold = (1.0 - dx) * (1.0 - dy);

    // decide which objects to include in the output
    vector<bool> includeInOutput(frame.size(), true);
    for (unsigned i = 0; i < frame.size(); i++) {
        double areaA = frame[i].area();
        for (unsigned j = (unsigned)frame.size() - 1; j > i; j--) {
            if (!includeInOutput[i] && !includeInOutput[j]) {
                continue;
            }
            if (frame[i].name != frame[j].name) {
	       continue;
            }
            double areaOverlap = frame[i].overlap(frame[j]);
            double areaB = frame[j].area();
            
#if 1
            // first check same scale, otherwise check neighbouring scale
            if ((areaA == areaB) && (areaOverlap > threshold * areaA)) {
                if (frame[i].pr < frame[j].pr) {
                    includeInOutput[i] = false;
                } else {
                    includeInOutput[j] = false;
                }
            } else if ((areaA > areaB * ds) && (areaB > areaA * ds) &&
                ((areaOverlap >= areaA * dsArea) || (areaOverlap >= areaB * dsArea))) {
                if (frame[i].pr < frame[j].pr) {
                    includeInOutput[i] = false;
                } else {
                    includeInOutput[j] = false;
                }		    
            }
#else
            // use area overlap measure = intersection / union
            areaOverlap = areaOverlap / (areaA + areaB - areaOverlap);
            if (areaOverlap > dsArea) {
                if (frame[i].pr < frame[j].pr) {
                    includeInOutput[i] = false;
                } else {
                    includeInOutput[j] = false;
                }	    		
            }
#endif
        }
    }

    // remove suppressed frames
    svlObject2dFrame filteredFrame;
    filteredFrame.reserve(frame.size());
    for (int i = 0; i < (int)includeInOutput.size(); i++) {
        if (includeInOutput[i]) {
            filteredFrame.push_back(frame[i]);
        }
    }
    frame = filteredFrame;

//     for (int i = (int)includeInOutput.size() - 1; i >= 0; i--) {
//         if (!includeInOutput[i]) {
//             frame.erase(frame.begin() + i);
//         }
//     }

    SVL_FCN_TOC;    
    return (int)(includeInOutput.size() - frame.size());
}

svlObject2dFrame sortObjects(const svlObject2dFrame& frame)
{
    // sort detections by score (highest first)
    vector<pair<double, int> > sortIndex;
    sortIndex.reserve(frame.size());
    for (unsigned i = 0; i < frame.size(); i++) {
        sortIndex.push_back(make_pair(-frame[i].pr, i));
    }
    stable_sort(sortIndex.begin(), sortIndex.end());
    
    svlObject2dFrame sortedFrame;
    sortedFrame.reserve(frame.size());
    for (unsigned i = 0; i < sortIndex.size(); i++) {
        sortedFrame.push_back(frame[sortIndex[i].second]);
    }
    
    return sortedFrame;
}

// svlObject2dSequence Class --------------------------------------------------------



bool writeObject2dSequence(const char *filename, const svlObject2dSequence& v)
{
  SVL_LOG(SVL_LOG_WARNING, "writeObject2dSequence is deprecated and may be removed after January 9, 2010. Use svlObject2dSequence::write instead.");
  return v.write(filename);
}

bool svlObject2dSequence::write(const char * filename) const
{
  const svlObject2dSequence & v = * this;
  ofstream ofs(filename);
  if (ofs.fail()) return false;
  
  ofs << "<Object2dSequence version=\"1.0\">" << endl;
  
  for (svlObject2dSequence::const_iterator it = v.begin(); it != v.end(); ++it) {
    it->second.write(ofs, it->first.c_str());
  }
  ofs << "</Object2dSequence>" << endl;
  
  ofs.close();
  return true;
}

bool readObject2dSequence(const char *filename, svlObject2dSequence& v)
{
  SVL_LOG(SVL_LOG_WARNING, "readObject2dSequence is deprecated and may be removed after January 9, 2010. Use svlObject2dSequence::read instead.");
  return v.read(filename);
}

bool svlObject2dSequence::read(const char * filename)
{
    svlObject2dSequence &v = *this;
    v.clear();

    svlCodeProfiler::tic(svlCodeProfiler::getHandle("XMLNode::parseFile"));
    XMLNode root = XMLNode::parseFile(filename, "Object2dSequence");
    svlCodeProfiler::toc(svlCodeProfiler::getHandle("XMLNode::parseFile"));

    if (root.isEmpty()) {
        return false;
    }

    int numObjectFrames = root.nChildNode("Object2dFrame");
    int objectNodeIndx = 0;
    for (int i = 0; i < numObjectFrames; i++) {
        XMLNode node = root.getChildNode("Object2dFrame", &objectNodeIndx);
                
        if (node.isEmpty() || !(node.getAttribute("id"))) continue;
        string id = string(node.getAttribute("id"));
        if (v.find(id) != v.end()) {
            SVL_LOG(SVL_LOG_WARNING, "duplicate id \"" << id << "\" in " << filename);
            continue;
        }

        svlObject2dFrame &frame = v[id];
        frame.resize(node.nChildNode("Object"));
        int nodeIndx = 0;
        for (int j = 0; j < (int)frame.size(); j++) {
            XMLNode objNode = node.getChildNode("Object", &nodeIndx);
            frame[j].name = objNode.getAttribute("name");
            frame[j].x = atof(objNode.getAttribute("x"));
            frame[j].y = atof(objNode.getAttribute("y"));
            frame[j].w = atof(objNode.getAttribute("w"));
            frame[j].h = atof(objNode.getAttribute("h"));
            
            if (frame[j].w < 0.0) { 
                frame[j].x += frame[j].w;
                frame[j].w = -frame[j].w;
            }
            if (frame[j].h < 0.0) { 
                frame[j].y += frame[j].h;
                frame[j].h = -frame[j].h;
            }

            if (objNode.getAttribute("pr")) {
                frame[j].pr = atof(objNode.getAttribute("pr"));
            } else {
                frame[j].pr = 1.0;
            }

            if (objNode.getAttribute("index")) {
                frame[j].index = atoi(objNode.getAttribute("index"));
            }

            if (objNode.getAttribute("ang")) {
                frame[j].ang = atof(objNode.getAttribute("ang"));
            } else {
                frame[j].ang = 0.0;
            }

            if (objNode.getAttribute("ignore")) {
                frame[j].ignore = atoi(objNode.getAttribute("ignore"));
            }            
        }
    }

    return true;
}

// merge
void svlObject2dSequence::merge(const svlObject2dSequence& seq)
{
    for (svlObject2dSequence::const_iterator it = seq.begin(); it != seq.end(); it++) {
        svlObject2dSequence::iterator jt = this->find(it->first);
        if (jt == this->end()) {
            this->insert(*it);
        } else {
            jt->second.insert(jt->second.end(), it->second.begin(), it->second.end());
        }
    }
}

int countObjects(const svlObject2dSequence& v)
{
	int count = 0;
	for (svlObject2dSequence::const_iterator it = v.begin(); it != v.end(); ++it) {
		count += (int)it->second.size();
	}

	return count;
}

int removeOverlappingObjects(svlObject2dSequence& v, double threshold)
{
	int count = 0;
	for (svlObject2dSequence::iterator it = v.begin(); it != v.end(); ++it) {
		count += removeOverlappingObjects(it->second, threshold);
	}

	return count;
}

int nonMaximalSuppression(svlObject2dSequence& v, double dx, double dy, double ds, double dsArea)
{
	int count = 0;   
	for (svlObject2dSequence::iterator it = v.begin(); it != v.end(); ++it) {
		count += nonMaximalSuppression(it->second, dx, dy, ds, dsArea);
	}

	return count;
}

void scaleObject2dSequence(svlObject2dSequence &v, double x_scale, double y_scale) {
	for (svlObject2dSequence::iterator it = v.begin(); it != v.end(); ++it) {
		for (unsigned j = 0; j < it->second.size(); j++) {
			it->second[j].scale(x_scale, y_scale);
		}
	}
}

bool removeFramesFromSequence(svlObject2dSequence &v, const char *filename) {
	// decide which of the current frames to keep
	map<string, bool> keep;
	for (svlObject2dSequence::const_iterator it = v.begin(); it != v.end(); ++it) {
		keep[it->first] = false;
	}

	// parse an xml results file that contains only a subset of the frames
	XMLNode root = XMLNode::parseFile(filename, "Object2dSequence");
	if (root.isEmpty()) {
		return false;
	}

    int numObjectFrames = root.nChildNode("Object2dFrame");
    int objectFrameIndx = 0;
	for (int i = 0; i < numObjectFrames; i++) {
		XMLNode node = root.getChildNode("Object2dFrame", &objectFrameIndx);
		if (node.isEmpty() || !(node.getAttribute("id"))) continue;

		string id = string(node.getAttribute("id"));
		if (v.find(id) != v.end()) {
			keep[id] = true;
		}
	}  

	// delete all frames from video that weren't present in the given file
	for (svlObject2dSequence::iterator it = v.begin(); it != v.end(); ++it) {
		if (keep[it->first]) continue;
		v.erase(it);
	}

	return true;
}


double averageAspectRatio(const svlObject2dSequence & s)
{
	double sum = 0.0;
	double count = 0.0;

	//Read each frame
	for (svlObject2dSequence::const_iterator i = s.begin(); i != s.end(); ++i)
	{
		const svlObject2dFrame & f = i->second;

		//Add this frame's contents to the count
		count += f.size();

		//Read each object in the frame
		for (unsigned int j = 0; j < f.size(); j++)
		{
			const svlObject2d & o = f[j];

			assert(o.h != 0);//can't find the aspect ratio of an object with no height!

			//Calculate the object's aspect ratio
			double ar = o.w / o.h;


			//Add this value to the sum
			sum += ar;
		}
	}

	//Convert the sum to a mean
	return sum / count;
}

bool svlObject2d::operator!=(const svlObject2d & o) const
{
	if (name != o.name)
		return true;

	if (!svlFloatCompare(x,o.x))
		return true;

	if (!svlFloatCompare(y,o.y))
		return true;

	if (!svlFloatCompare(w,o.w))
		return true;

	if (!svlFloatCompare(h,o.h))
		return true;

	if (!svlFloatCompare(pr,o.pr))
		return true;

	if (!svlFloatCompare(ang,o.ang))
		return true;

	if (index != o.index)
		return true;

	return false;
}
