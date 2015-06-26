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
** FILENAME:    svlFeatureVectors.h
** AUTHOR(S):   Paul Baumstarck <pbaumstarck@stanford.edu>
** DESCRIPTION:
**  Defines a class for storing feature vectors, locations, and scales in
**  double or float format.
**
*****************************************************************************/

#pragma once

#include <cassert>
#include <iostream>
#include <vector>

#include "cv.h"
#include "cxcore.h"

using namespace std;

// svlFeatureVectorsT ---------------------------------------------------

template<typename T = double, typename D = float>
class svlFeatureVectorsT {
public:
	// Controls which members to collect.
	bool _want_scales, _want_locations, _want_features;
	vector<CvSize> _scales;
	vector<vector<CvPoint> > _locations;
	vector<vector<vector<T> > > _features;
public:
	svlFeatureVectorsT() : _want_scales(true), _want_locations(true), _want_features(true) {}
	svlFeatureVectorsT(bool want_scales, bool want_locations, bool want_features)
		: _want_scales(want_scales), _want_locations(want_locations), _want_features(want_features) {}
	virtual ~svlFeatureVectorsT() {};

	inline void clear() {
		_scales.clear();
		_locations.clear();
		_features.clear();
	}
	// Add on another layer of data.
	void push(const CvSize &sz, const vector<CvPoint> &locations, const vector<vector<T> > &features) {
		if ( _want_scales ) _scales.push_back(sz);
		if ( _want_locations ) _locations.push_back(locations);
		if ( _want_features ) _features.push_back(features);
	}
	// With an Ipl for the size.
	void push(const IplImage *img, const vector<CvPoint> &locations, const vector<vector<T> > &features) {
		if ( _want_scales ) _scales.push_back(cvSize(img->width,img->height));
		if ( _want_locations ) _locations.push_back(locations);
		if ( _want_features ) _features.push_back(features);
	}

	// Convert this feature vector to another.
	void convertTo(svlFeatureVectorsT<D,T> &F) {
		F.clear();
		// Copy over all members, starting with scales.
		F._scales = _scales;
		// Locations.
		F._locations.reserve(_locations.size());
		for ( vector<vector<CvPoint> >::const_iterator it=_locations.begin(); it!=_locations.end(); ++it )
			F._locations.push_back(*it);
		// Features.
		F._features.reserve(_features.size());
		for ( unsigned i=0; i<_features.size(); ++i ) {
			F._features.push_back(vector<vector<D> >()); // D
			F._features.back().reserve(_features[i].size());
			for ( unsigned j=0; j<_features[i].size(); ++j ) {
				F._features.back().push_back(vector<D>()); // D
				F._features.back().back().reserve(_features[i][j].size());
				for ( typename vector<T>::const_iterator it=_features[i][j].begin(); it!=_features[i][j].end(); ++it ) // T
					F._features.back().back().push_back( D(*it) ); // D
			}
		}
	}
};

typedef svlFeatureVectorsT<double,float> svlFeatureVectors;
typedef svlFeatureVectorsT<float,double> svlFeatureVectorsF;



