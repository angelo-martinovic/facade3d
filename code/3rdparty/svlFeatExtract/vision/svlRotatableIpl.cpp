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
** FILENAME:    svlRotatableIpl.cpp
** AUTHOR(S):   Paul Baumstarck <pbaumstarck@stanford.edu>
**
*****************************************************************************/

#include "svlBase.h"
#include "svlRotatableIpl.h"

svlRotatableIpl::svlRotatableIpl()
{
	_img = NULL;
	_translate = cvCreateMat(2, 3, CV_32FC1);
}

svlRotatableIpl::svlRotatableIpl(const IplImage *img)
{
	// Maximum image dimension.
	_orig_w = img->width;
	_orig_h = img->height;
	int a = 2*int( ceil( sqrtf( powf(float(img->width)*0.5f, 2.0f) + powf(float(img->height)*0.5f, 2.0f) ) ) );
	_img  = cvCreateImage(cvSize(a,a), IPL_DEPTH_32F, 1);
	_cent = a/2;
	cvZero(_img);

	// Copy img into _img at offset locations to be ready for rotation.
	int r_off = (a - img->height)/2, c_off = (a - img->width)/2;
	for ( int r=0; r<img->height; ++r ) {
		memcpy(&CV_IMAGE_ELEM(_img, float, r_off+r, c_off), &CV_IMAGE_ELEM(img, float, r, 0), sizeof(float)*img->width);
	}
	_translate = cvCreateMat(2, 3, CV_32FC1);
}

svlRotatableIpl::svlRotatableIpl(const svlRotatableIpl *c)
{
	assert(false);
}

svlRotatableIpl::~svlRotatableIpl()
{
	if ( _img )
		cvReleaseImage(&_img);
	cvReleaseMat(&_translate);
}


// *** Rotation and return functions ***

// Tim an image of zero rows and columns.
pair<pair<int,int>,pair<int,int> > svlRotatableIpl::trimImageBounds(const IplImage *img)
{
	assert(img->depth == IPL_DEPTH_32F && img->nChannels == 1); // Not to be hassled by implementation differences now.

	//IplImage *ret = NULL;
	int r1 = 0, r2 = img->height-1, c1 = 0, c2 = img->width-1;
	bool gb;

	// Get starting row bounds.
	gb = true;
	for ( ; gb && r1<img->height; ++r1 )
		for ( int c=0; gb && c<img->width; ++c )
			if ( CV_IMAGE_ELEM(img, float, r1, c) != 0 )
				gb = false;
	if ( r1 == img->height ) // Entire image is zero.
		return pair<pair<int,int>,pair<int,int> >( pair<int,int>(0,0), pair<int,int>(0,0) );

	// Get ending row bound.
	gb = true;
	for ( ; gb && r2>=0; --r2 )
		for ( int c=0; gb && c<img->width; ++c )
			if ( CV_IMAGE_ELEM(img, float, r2, c) != 0 )
				gb = false;
	++r2; // Exclusive bound [r1,r2)
	
	// Get starting column bound.
	gb = true;
	for ( ; gb && c1<img->width; ++c1 )
		for ( int r=r1; gb && r<r2; ++r )
			if ( CV_IMAGE_ELEM(img, float, r, c1) != 0 )
				gb = false;

	// Get ending column bound.
	gb = true;
	for ( ; gb && c2>=0; --c2 )
		for ( int r=r1; gb && r<r2; ++r )
			if ( CV_IMAGE_ELEM(img, float, r, c2) != 0 )
				gb = false;
	++c2; // Exclusive bound [c1,c2)

	return pair<pair<int,int>,pair<int,int> >( pair<int,int>(r1,r2), pair<int,int>(c1,c2) );
	//printf("Bound: [%d,%d), [%d,%d)\n", r1, r2, c1, c2);
	//// Chip out image.
	//ret = cvCreateImage(cvSize(c2-c1,r2-r1), img->depth, 1);
	//for ( int r=r1; r<r2; ++r )
	//	memcpy( &CV_IMAGE_ELEM(ret, float, r-r1, 0), &CV_IMAGE_ELEM(img, float, r, c1), sizeof(float)*(c2-c1) );
	//return ret;
}

// Trim image with bounds.
IplImage* svlRotatableIpl::trimImage(const IplImage *img, pair<pair<int,int>,pair<int,int> > &bounds)
{
	int r1 = bounds.first.first, r2 = bounds.first.second,
		c1 = bounds.second.first, c2 = bounds.second.second;
	if ( r1 < 0 && c1 < 0 || r2 >= img->height || c2 >= img->width ) {
		SVL_LOG(SVL_LOG_FATAL, "Trimming past image bounds");
	}

	// Chip out image.
	IplImage *ret = cvCreateImage(cvSize(c2-c1,r2-r1), img->depth, 1);
	for ( int r=r1; r<r2; ++r )
		memcpy( &CV_IMAGE_ELEM(ret, float, r-r1, 0), &CV_IMAGE_ELEM(img, float, r, c1), sizeof(float)*(c2-c1) );
	return ret;
}

// Trim image.
IplImage* svlRotatableIpl::trimImage(const IplImage *img)
{
	pair<pair<int,int>,pair<int,int> > rrcc = trimImageBounds(img);
	int r1 = rrcc.first.first, r2 = rrcc.first.second;
	//c1 = rrcc.second.first, c2 = rrcc.second.second;
	if ( r1 == 0 && r2 == 0 )
		return NULL;

	return trimImage(img, rrcc);
}

// The above, with multiple images.
void svlRotatableIpl::tandemTrimImage(const vector<IplImage*> &imgs, vector<IplImage*> &rets)
{
	pair<pair<int,int>,pair<int,int> > rrcc;
	rrcc = trimImageBounds(imgs[0]);
	int r1 = rrcc.first.first, r2 = rrcc.first.second,
		c1 = rrcc.second.first, c2 = rrcc.second.second;
	if ( r1 == 0 && r2 == 0 )
		return; // Null image.

	for ( unsigned i=1; i<imgs.size(); ++i ) {
		rrcc = trimImageBounds(imgs[i]);
		// Take min of lower bounds, max of upper.
		r1 = min(r1, rrcc.first.first);
		r2 = max(r2, rrcc.first.second);
		c1 = min(c1, rrcc.second.first);
		c2 = max(c2, rrcc.second.second);
		if ( r1 == 0 && r2 == 0 )
			return; // Null image.
	}

	// Trim images.
	rets = vector<IplImage*>(imgs.size(),NULL);
	rrcc = pair<pair<int,int>,pair<int,int> >( pair<int,int>(r1,r2), pair<int,int>(c1,c2) );
	for ( unsigned i=0; i<imgs.size(); ++i )
		rets[i] = trimImage(imgs[i], rrcc);
}

// Returns a rotated copy of the image.
IplImage* svlRotatableIpl::rotate(float ang)
{
	CvPoint2D32f cent;
	cent.x = cent.y = float(_cent);
	cvSetZero(_translate);
	cv2DRotationMatrix(cent, ang, 1.0, _translate);
	IplImage *ret = cvCreateImage(cvSize(_img->width,_img->height), IPL_DEPTH_32F, 1);
	cvWarpAffine(_img, ret, _translate);
	return ret;
}

// Rotates the image, overwrites it, and returns the point to it.
IplImage* svlRotatableIpl::rotateInPlace(float ang)
{
	IplImage* ret = rotate(ang);
	cvCopy(ret, _img);
	cvReleaseImage(&ret);
	return _img;
}

// Returns a copy of the image that has been trimmed of zero columns and rows.
IplImage* svlRotatableIpl::getTrimmed()
{
	return trimImage(_img);
}

// Rotate and return the trimmed image.
IplImage* svlRotatableIpl::rotateGetTrimmed(float ang)
{
	IplImage *rot = rotate(ang);
	IplImage *trimmed = trimImage(rot);
	cvReleaseImage(&rot);
	return trimmed;
}

// Rotate in place and return the trimmed image.
IplImage* svlRotatableIpl::rotateInPlaceGetTrimmed(float ang)
{
	return trimImage(rotateInPlace(ang)); // RIP returns _img anyway, so just catch here.
}

// Rotate and trim in tandem.
void svlRotatableIpl::tandemRotateGetTrimmed(const vector<svlRotatableIpl*> &rimgs, float ang, vector<IplImage*> &imgs)
{
	vector<IplImage*> temps(rimgs.size(),NULL);
	for ( unsigned i=0; i<rimgs.size(); ++i )
		temps[i] = rimgs[i]->rotate(ang);

	// Trim in tandem.
	tandemTrimImage(temps, imgs);

	// Free temporaries
	for ( unsigned i=0; i<temps.size(); ++i )
		cvReleaseImage(&temps[i]);
}


