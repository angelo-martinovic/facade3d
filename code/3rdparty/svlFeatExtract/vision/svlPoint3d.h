/******************************************************************************
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
** FILENAME:    svlPoint3d.h
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
**              Ian Goodfellow <ia3n@stanford.edu>
**		Paul Baumstarck <pbaumstarck@stanford.edu>
** DESCRIPTION:
**  Basic 3-d point class. Supports simple mathematical operations. The y-axis
**  is defined as up, so panning is about the y-axis, tilting is about the
**  x-axis and rolling is about the z-axis.
** 
*****************************************************************************/

#pragma once

#if defined(_WIN32)||defined(WIN32)||defined(__WIN32__)
#define _CRT_SECURE_NO_DEPRECATE
#undef max
#undef min
#endif

#include <limits>
#include <cassert>
#include <cmath>
#include <iostream>

using namespace std;

class svlPoint3d {
 public:
    double x;
    double y;
    double z;

 public:
    inline svlPoint3d() {
        x = y = z = 0.0;
    }
    inline svlPoint3d(double v) {
        x = v; y = v; z = v;
    }
    inline svlPoint3d(double nx, double ny, double nz = 0.0) {
        x = nx; y = ny; z = nz;
    }
    inline svlPoint3d(const svlPoint3d& p) {
        x = p.x; y = p.y; z = p.z;
    }
    virtual ~svlPoint3d() {
        // do nothing
    }

    inline int size();
    inline double getD(int d) const;
    inline void setD(int d, double val);
    inline void set(double val);
    inline void set(double _x, double _y, double _z);
    inline double& operator[](int i);
    inline double operator[](int i) const;
    
    inline double norm() const;
    inline double norm2() const;
    void set_min_pt();
    void set_max_pt();
    inline double dot(const svlPoint3d& p) const;
    inline svlPoint3d cross(const svlPoint3d & p) const;
    inline svlPoint3d& normalize();
    inline double do_normalize();
    inline svlPoint3d& ptwise_sqrt();
    inline svlPoint3d& abs();
    inline svlPoint3d get_abs();
    inline double pt_min() const;
    inline double pt_max() const;
    friend inline svlPoint3d min2(const svlPoint3d& p, const svlPoint3d& q);
    friend inline svlPoint3d max2(const svlPoint3d& p, const svlPoint3d& q);
    
    inline svlPoint3d& min_with(const svlPoint3d& p);
    inline svlPoint3d& min_with(double x_in, double y_in, double z_in);
    inline svlPoint3d& max_with(const svlPoint3d& p);
    inline svlPoint3d& max_with(double x_in, double y_in, double z_in);

    inline svlPoint3d& pan(double theta);
    inline svlPoint3d& pan(double cos_theta, double sin_theta);
    inline svlPoint3d& pan(double theta, svlPoint3d& c);
    inline svlPoint3d& tilt(double theta);
    inline svlPoint3d& tilt(double cos_theta, double sin_theta);
    inline svlPoint3d& tilt(double theta, svlPoint3d& c);
    inline svlPoint3d& roll(double theta);
    inline svlPoint3d& roll(double theta, svlPoint3d& c);

    inline svlPoint3d& operator=(const svlPoint3d& p);
    inline svlPoint3d& operator=(const double d);
    inline bool operator>(const double d);
    inline bool operator>=(const double d);
    inline bool operator<(const double d);
    inline bool operator<=(const double d);
    inline svlPoint3d& operator+=(const svlPoint3d& p);
    inline svlPoint3d& operator+=(const double d);
    inline svlPoint3d& operator-=(const svlPoint3d& p);
    inline svlPoint3d& operator-=(const double d);
    inline svlPoint3d& operator*=(const svlPoint3d& p);
    inline svlPoint3d& operator*=(const double d);
    inline svlPoint3d& operator/=(const svlPoint3d& p);
    inline svlPoint3d& operator/=(const double d);
    inline bool operator==(const svlPoint3d& p) const;
    inline bool operator==(const double d) const;
    inline bool operator!=(const svlPoint3d& p) const;
    inline bool operator!=(const double d) const;

    friend inline svlPoint3d operator-(const svlPoint3d& p);
    friend inline svlPoint3d operator+(const svlPoint3d& p, const svlPoint3d& q);
    friend inline svlPoint3d operator+(const svlPoint3d& p, double d);
    friend inline svlPoint3d operator-(const svlPoint3d& p, const svlPoint3d& q);
    friend inline svlPoint3d operator-(const svlPoint3d& p, double d);
    friend inline svlPoint3d operator*(double d, const svlPoint3d& p);
    friend inline svlPoint3d operator*(const svlPoint3d& p, double d);
    friend inline svlPoint3d operator*(const svlPoint3d& p, const svlPoint3d& q);
    friend inline svlPoint3d operator/(const svlPoint3d& p, double d);

    friend std::ostream& operator<<(std::ostream& os, const svlPoint3d& p) {
	os << p.x << " " << p.y << " " << p.z;
	return os;
    }
    friend std::istream& operator>>(std::istream& is, svlPoint3d& p) {
	is >> p.x >> p.y >> p.z;
	return is;
    }
};

/* Implementation ***********************************************************/

inline int svlPoint3d::size() {
    return 3;
}

inline double svlPoint3d::getD(int d)  const
{
    switch(d) {
    case 0: return x; break;
    case 1: return y; break;
    case 2: return z; break;
    default: 
	assert(false);
	return 0.0;
    }
}

inline void svlPoint3d::setD(int d, double val) 
{
    switch(d) {
    case 0: x = val; break;
    case 1: y = val; break;
    case 2: z = val; break;
    default: 
	assert(false);
    }
}

inline void svlPoint3d::set(double val)
{
    x = y = z = val;
}

inline void svlPoint3d::set(double _x, double _y, double _z)
{
    x = _x;
    y = _y;
    z = _z;
}

inline double& svlPoint3d::operator[](int i)
{
    switch(i) {
    case 0: return this->x;
    case 1: return this->y;
    case 2: return this->z;
    default: assert(false);
    }
    return this->x;
}

inline double svlPoint3d::operator[](int i) const
{
    switch(i) {
    case 0: return this->x;
    case 1: return this->y;
    case 2: return this->z;
    default: assert(false);
    }
    return this->x;
}

inline double svlPoint3d::norm() const
{
    return sqrt(norm2());
}

inline double svlPoint3d::norm2() const
{
    return (x * x + y * y + z * z);
}

inline void svlPoint3d::set_min_pt()
{
	x = y = z = numeric_limits<double>::min();
}

inline void svlPoint3d::set_max_pt()
{
	x = y = z = numeric_limits<double>::max();
}

inline double svlPoint3d::dot(const svlPoint3d& p) const
{
    return (x * p.x + y * p.y + z * p.z);
}

inline svlPoint3d svlPoint3d::cross(const svlPoint3d & p) const
{
  return svlPoint3d (y*p.z-z*p.y, z*p.x-x*p.z, x*p.y - y*p.x);
}

inline svlPoint3d& svlPoint3d::normalize()
{
    double len = sqrt(norm2());
    if (len != 0)
      {
	x /= len; y /= len; z /= len;
      }
    return *this;
}

inline double svlPoint3d::do_normalize()
{
    double len = sqrt(norm2());
    x /= len; y /= len; z /= len;
    return len;
}

inline svlPoint3d& svlPoint3d::ptwise_sqrt() {
    x = sqrt(x); y = sqrt(y); z = sqrt(z);
    return *this;
}

inline svlPoint3d& svlPoint3d::abs()
{
    if ( x < 0 ) x *= -1.0;
    if ( y < 0 ) y *= -1.0;
    if ( z < 0 ) z *= -1.0;
    return *this;
}

inline svlPoint3d svlPoint3d::get_abs() {
    return svlPoint3d(x < 0 ? -x:x, y < 0? -y:y, z < 0 ?-z:z);
}

inline double svlPoint3d::pt_min() const
{
    if ( x < y )
        return z < x ? z : x;
    else
        return z < y ? z : y;
    //return std::min(std::min(x,y,),z);
}

inline double svlPoint3d::pt_max() const
{
    if ( x > y )
        return z > x ? z : x;
    else
        return z > y ? z : y;
}

inline svlPoint3d min2(const svlPoint3d& p, const svlPoint3d& q)
{
    return svlPoint3d( p.x < q.x ? p.x : q.x, p.y < q.y ? p.y : q.y, p.z < q.z ? p.z : q.z);
}

inline svlPoint3d max2(const svlPoint3d& p, const svlPoint3d& q)
{
    return svlPoint3d( p.x > q.x ? p.x : q.x, p.y > q.y ? p.y : q.y, p.z > q.z ? p.z : q.z);
}

inline svlPoint3d& svlPoint3d::min_with(const svlPoint3d& p) {
    x = x < p.x ? x : p.x;
    y = y < p.y ? y : p.y;
    z = z < p.z ? z : p.z;
    return *this;
}

inline svlPoint3d& svlPoint3d::min_with(double x_in, double y_in, double z_in) {
    x = x < x_in ? x : x_in;
    y = y < y_in ? y : y_in;
    z = z < z_in ? z : z_in;
    return *this;
}

inline svlPoint3d& svlPoint3d::max_with(const svlPoint3d& p) {
    x = x > p.x ? x : p.x;
    y = y > p.y ? y : p.y;
    z = z > p.z ? z : p.z;
    return *this;
}

inline svlPoint3d& svlPoint3d::max_with(double x_in, double y_in, double z_in) {
    x = x > x_in ? x : x_in;
    y = y > y_in ? y : y_in;
    z = z > z_in ? z : z_in;
    return *this;
}

inline svlPoint3d& svlPoint3d::pan(double theta)
{
    double nx = x * cos(theta) - z * sin(theta);
    double nz = x * sin(theta) + z * cos(theta);
    x = nx; z = nz;

    return *this;
}

inline svlPoint3d& svlPoint3d::pan(double cos_theta, double sin_theta)
{
    double nx = x * cos_theta - z * sin_theta;
    double nz = x * sin_theta + z * cos_theta;
    x = nx; z = nz;

    return *this;
}

inline svlPoint3d& svlPoint3d::pan(double theta, svlPoint3d& c)
{
    double nx = (x - c.x) * cos(theta) - (z - c.z) * sin(theta);
    double nz = (x - c.x) * sin(theta) + (z - c.z) * cos(theta);
    x = c.x + nx; z = c.z + nz;    

    return *this;
}

inline svlPoint3d& svlPoint3d::tilt(double theta)
{
    double ny = y * cos(theta) - z * sin(theta);
    double nz = y * sin(theta) + z * cos(theta);
    y = ny; z = nz;

    return *this;
}

inline svlPoint3d& svlPoint3d::tilt(double cos_theta, double sin_theta)
{
    double ny = y * cos_theta - z * sin_theta;
    double nz = y * sin_theta + z * cos_theta;
    y = ny; z = nz;

    return *this;
}

inline svlPoint3d& svlPoint3d::tilt(double theta, svlPoint3d& c)
{
    double ny = (y - c.y) * cos(theta) - (z - c.z) * sin(theta);
    double nz = (y - c.y) * sin(theta) + (z - c.z) * cos(theta);
    y = c.y + ny; z = c.z + nz;    

    return *this;
}

inline svlPoint3d& svlPoint3d::roll(double theta)
{
    double nx = x * cos(theta) - y * sin(theta);
    double ny = x * sin(theta) + y * cos(theta);
    x = nx; y = ny;

    return *this;
}

inline svlPoint3d& svlPoint3d::roll(double theta, svlPoint3d& c)
{
    double nx = (x - c.x) * cos(theta) - (y - c.y) * sin(theta);
    double ny = (x - c.x) * sin(theta) + (y - c.y) * cos(theta);
    x = c.x + nx; y = c.y + ny;    

    return *this;
}

/* Operators ****************************************************************/

inline svlPoint3d& svlPoint3d::operator=(const svlPoint3d& p)
{
    x = p.x; y = p.y; z = p.z;
    return *this;
}

inline svlPoint3d& svlPoint3d::operator=(const double d)
{
    x = y = z = d;
    return *this;
}

inline bool svlPoint3d::operator>(const double d)
{
    return ((x > d) && (y > d) && (z > d));
}

inline bool svlPoint3d::operator>=(const double d)
{
    return ((x >= d) && (y >= d) && (z >= d));
}

inline bool svlPoint3d::operator<(const double d)
{
    return ((x < d) && (y < d) && (z < d));
}

inline bool svlPoint3d::operator<=(const double d)
{
    return ((x <= d) && (y <= d) && (z <= d));
}

inline svlPoint3d& svlPoint3d::operator+=(const svlPoint3d& p)
{
    x += p.x; y += p.y; z += p.z;
    return *this;
}

inline svlPoint3d& svlPoint3d::operator+=(const double d)
{
    x += d; y += d; z += d;
    return *this;
}

inline svlPoint3d& svlPoint3d::operator-=(const svlPoint3d& p)
{
    x -= p.x; y -= p.y; z -= p.z;
    return *this;
}

inline svlPoint3d& svlPoint3d::operator-=(const double d)
{
    x -= d; y -= d; z -= d;
    return *this;
}

inline svlPoint3d& svlPoint3d::operator*=(const svlPoint3d& p)
{
    x *= p.x; y *= p.y; z *= p.z;
    return *this;
}

inline svlPoint3d& svlPoint3d::operator*=(const double d)
{
    x *= d; y *= d; z *= d;
    return *this;
}

inline svlPoint3d& svlPoint3d::operator/=(const svlPoint3d& p)
{
    x /= p.x; y /= p.y; z /= p.z;
    return *this;
}

inline svlPoint3d& svlPoint3d::operator/=(const double d)
{
    x /= d; y /= d; z /= d;
    return *this;
}

inline bool svlPoint3d::operator==(const svlPoint3d& p) const
{
    return ((x == p.x) && (y == p.y) && (z == p.z));	
}

inline bool svlPoint3d::operator==(const double d) const
{
    return ((x == d) && (y == d) && (z == d));
}

inline bool svlPoint3d::operator!=(const svlPoint3d& p) const
{
    return !(*this == p);
}

inline bool svlPoint3d::operator!=(const double d) const
{
    return !(*this == d);
}

inline svlPoint3d operator-(const svlPoint3d& p)
{
    return svlPoint3d(-p.x, -p.y, -p.z);
}

inline svlPoint3d operator+(const svlPoint3d& p, const svlPoint3d& q)
{
    return svlPoint3d(p.x + q.x, p.y + q.y, p.z + q.z);
}

inline svlPoint3d operator+(const svlPoint3d& p, double d)
{
    return svlPoint3d(p.x + d, p.y + d, p.z + d);
}

inline svlPoint3d operator-(const svlPoint3d& p, const svlPoint3d& q)
{
    return svlPoint3d(p.x - q.x, p.y - q.y, p.z - q.z);
}

inline svlPoint3d operator-(const svlPoint3d& p, double d)
{
    return svlPoint3d(p.x - d, p.y - d, p.z - d);
}

inline svlPoint3d operator*(double d, const svlPoint3d& p)
{
    return svlPoint3d(d * p.x, d * p.y, d * p.z);
}

inline svlPoint3d operator*(const svlPoint3d& p, double d)
{
    return svlPoint3d(d * p.x, d * p.y, d * p.z);
}

inline svlPoint3d operator*(const svlPoint3d& p, const svlPoint3d& q)
{
	return svlPoint3d(p.x * q.x, p.y * q.y, p.z * q.z);
}

inline svlPoint3d operator/(const svlPoint3d& p, double d)
{
    return svlPoint3d(p.x / d, p.y / d, p.z / d);
}

