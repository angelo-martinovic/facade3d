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
** FILENAME:    svlMultiAspectAngleSWD.cpp
** AUTHOR(S):   Paul Baumstarck <pbaumstarck@stanford.edu>
**
*****************************************************************************/

#include <stdio.h>
#include <cassert>
#include <iostream>
#include <fstream>
#include <vector>
#include <sys/types.h>
#include <limits>

#include "cxcore.h"
#include "ml.h"

#include "svlBase.h"
#include "svlMultiAspectAngleSWD.h"

using namespace std;

// Classify at multiple aspects ratios and scales.
void svlMultiAspectAngleSWD::classifyImage(const vector<IplImage*>& imagesin,
    vector<vector<svlObject2dFrame> > *objects,
    vector<vector<svlFeatureVectors> > *vecs,
    vector<vector<svlFeatureVectorsF> > *vecsF,
    vector<float> *aspectsin,
    vector<float> *anglesin)
{
	SVL_ASSERT_MSG(_detector, "Need to provide a detector first");
	SVL_ASSERT(imagesin.size() > 0);
	for ( unsigned i=0; i<imagesin.size(); ++i ) {
            SVL_ASSERT(imagesin[i] != NULL);
            SVL_ASSERT(imagesin[i]->width == imagesin[0]->width);
            SVL_ASSERT(imagesin[i]->height == imagesin[0]->height);
	}

	// Generate aspects and angles to use.
	vector<float> aspects = aspectsin ? *aspectsin : vector<float>(1,1.0f),
		angles = anglesin ? *anglesin : vector<float>(1,0.0f);
	// Created rotated window discarder and add to detector.
	svlRotatedWindowDiscarder sword;
	_detector->pushDiscarder(&sword);

	// Loop over all aspects.
	if ( objects ) objects->reserve(aspects.size());
	if ( vecs ) vecs->reserve(aspects.size());
	if ( vecsF ) vecsF->reserve(aspects.size());
	for ( unsigned i=0; i<aspects.size(); ++i ) {
		// Allocate return space where required.
		if ( objects ) objects->push_back(vector<svlObject2dFrame>());
		if ( vecs ) vecs->push_back(vector<svlFeatureVectors>());
		if ( vecsF ) vecsF->push_back(vector<svlFeatureVectorsF>());

		int w = imagesin[i]->width, h = imagesin[i]->height;
		if ( aspects[i] >= 1.0f ) {
			// Contract height to increase aspect ratio.
			h = int(float(h)/aspects[i]);
		} else {
			// Contract width to decrease aspect ratio.
			w = int(float(w)*aspects[i]);
		}
		sword.setSize(cvSize(w,h));
		SVL_LOG(SVL_LOG_VERBOSE, "Scaling from " << imagesin[i]->width << "," << imagesin[i]->height << " to " << w << "," << h);

		// Create rotatable images as this aspect ratio.
		vector<svlRotatableIpl*> rimgs(imagesin.size(),NULL);
		for ( unsigned k=0; k<imagesin.size(); ++k ) {
			if ( aspects[i] == 1.0f ) {
				rimgs[k] = new svlRotatableIpl(imagesin[k]); // That was easy.
			} else {
				// Rescale.
				IplImage *timg = cvCreateImage(cvSize(w,h), imagesin[k]->depth, imagesin[k]->nChannels);
				cvResize(imagesin[k], timg);
				rimgs[k] = new svlRotatableIpl(timg);
				cvReleaseImage(&timg);
			}
		}

		// Loop over all angles.
		if ( objects ) objects->back().reserve(angles.size());
		if ( vecs ) vecs->back().reserve(angles.size());
		if ( vecsF ) vecsF->back().reserve(angles.size());
		for ( unsigned j=0; j<angles.size(); ++j ) {
			// Allocate return space where required.
			if ( objects ) objects->back().push_back(svlObject2dFrame());
			if ( vecs ) vecs->back().push_back(svlFeatureVectors());
			if ( vecsF ) vecsF->back().push_back(svlFeatureVectorsF());
			sword.setAngle(-1.0*double(angles[j]));

			vector<IplImage*> images_send;
			svlRotatableIpl::tandemRotateGetTrimmed(rimgs, -1.0f*angles[j], images_send);

			//// Output images.
			//for ( unsigned k=0; k<imagesin.size(); ++k ) {
			//	char c[32]; sprintf(c,"aa_%d_%d_%d.out",i,j,k);
			//	svlBinaryWriteIpl(c, images_send[k]);
			//}

			if ( vecs ) { // Call double version.
				_detector->classifyImage(images_send,
					objects ? &objects->back().back() : NULL,
					vecs ? &vecs->back().back() : NULL);
			} else { // Call float version.
				_detector->classifyImageF(images_send,
					objects ? &objects->back().back() : NULL,
					vecsF ? &vecsF->back().back() : NULL);
			}

			// Free sent images.
			for ( unsigned k=0; k<imagesin.size(); ++k )
				cvReleaseImage(&images_send[k]);
		}

		// Free rotatable Ipls.
		for ( unsigned k=0; k<rimgs.size(); ++k )
			delete rimgs[k];
	}
	_detector->popDiscarder();
}

// Classify at multiple aspects ratios and scales.
void svlMultiAspectAngleSWD::classifyImage(const vector<IplImage*>& imagesin,
    vector<vector<svlObject2dFrame> > *objects,
    vector<vector<svlFeatureVectors> > *vecs,
    vector<float> *aspectsin,
    vector<float> *anglesin)
{
    classifyImage(imagesin, objects, vecs, NULL, aspectsin, anglesin);
}
// Float version.
void svlMultiAspectAngleSWD::classifyImageF(const vector<IplImage*>& imagesin,
    vector<vector<svlObject2dFrame> > *objects,
    vector<vector<svlFeatureVectorsF> > *vecs,
    vector<float> *aspectsin,
    vector<float> *anglesin)
{
    classifyImage(imagesin, objects, NULL, vecs, aspectsin, anglesin);
}


