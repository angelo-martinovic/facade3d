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
** FILENAME:    svlSpinImagess.cpp
** AUTHOR(S):   Chih-Wei Chen <louistw@stanford.edu>
**
*****************************************************************************/

#include <iostream>
#include <cmath>

#include "cv.h"
#include "cxcore.h"

#include "svlBase.h"
#include "svlVision.h"

#include "svlSpinImages.h"

using namespace std;

svlSpinImages::svlSpinImages(int height, int width)
{
    w = width;
	h = height;
	X = cvCreateMat(h, w, CV_32FC1);
	Y = cvCreateMat(h, w, CV_32FC1);
	Z = cvCreateMat(h, w, CV_32FC1);
	nX = cvCreateMat(h, w, CV_32FC1);
	nY = cvCreateMat(h, w, CV_32FC1);
	nZ = cvCreateMat(h, w, CV_32FC1);
	// spin image generation parameters (dependent on data)
	binSize = 0.002f;
	// binSize is usually set to be the resolution of the point clouds,
	// i.e. the mean Euclidean distance of adjacent points.
	imageWidth = 15;
	// when calculating the spin-images at (x,y),
	// only adjacent (x +- imageWidth, y +- imageWidth) points are considered. 
}

void svlSpinImages::setscene(CvMat *x, CvMat *y, CvMat *z)
{
    X = x;
	Y = y;
	Z = z;
	estimatePointNormals(X, Y, Z, nX, nY, nZ);
	return;
}

void svlSpinImages::setscene(CvMat *x, CvMat *y, CvMat *z, CvMat *nx, CvMat *ny, CvMat *nz)
{
    X = x;
	Y = y;
	Z = z;
	nX = nx;
	nY = ny;
	nZ = nz;
	return;
}

void svlSpinImages::calculateNormals()
{
    estimatePointNormals(X, Y, Z, nX, nY, nZ);
	return;
}

svlSpinImages::~svlSpinImages()
{
    cvReleaseMat(&X);
	cvReleaseMat(&Y);
	cvReleaseMat(&Z);
	cvReleaseMat(&nX);
	cvReleaseMat(&nY);
	cvReleaseMat(&nZ);
}

CvMat *svlSpinImages::siCalculate(int y, int x)
{
    CvMat *si = cvCreateMat(imageWidth, imageWidth, CV_32FC1);
	for(int dy = 0; dy<imageWidth; dy++)
		for(int dx = 0; dx< imageWidth; dx++)
			cvmSet(si,dy,dx,0);
	double currentX, currentY, currentZ;
	double currentnX, currentnY, currentnZ;
	double iterateX, iterateY, iterateZ;
	double alpha, beta;
	double a,b;
	int i,j;
	currentX = cvmGet(X,y,x);
	currentY = cvmGet(Y,y,x);
	currentZ = cvmGet(Z,y,x);
	currentnX = cvmGet(nX,y,x);
	currentnY = cvmGet(nY,y,x);
	currentnZ = cvmGet(nZ,y,x);

	for (int dy = -imageWidth; dy <= imageWidth; dy++) {
		// make sure the index doesn't go out of bound
		if(y+dy > 0 && y+dy < int(h))
		{
			for (int dx = -imageWidth; dx <= imageWidth; dx++) {
				if(x+dx > 0 && x+dx < int(w))
				{
					iterateX = cvmGet(X,y+dy,x+dx);
					iterateY = cvmGet(Y,y+dy,x+dx);
					iterateZ = cvmGet(Z,y+dy,x+dx);
					beta = currentnX * (iterateX - currentX) + currentnY * (iterateY - currentY) + currentnZ * (iterateZ - currentZ);
					alpha = sqrt((iterateX - currentX)*(iterateX - currentX) + (iterateY - currentY)*(iterateY - currentY) + (iterateZ - currentZ)*(iterateZ - currentZ) - beta*beta);
					i = (int)floor((imageWidth*binSize/2 - beta)/binSize);
					j = (int)floor(alpha/binSize);
					a = (alpha - j*binSize)/binSize;
					b = (imageWidth*binSize/2 - beta - i*binSize)/binSize;
					if(i>=0 && i<imageWidth && j<imageWidth)
						cvmSet(si,i,j,cvmGet(si,i,j)+(1-a)*(1-b));
					if(i>=-1 && i<imageWidth-1 && j<imageWidth)
						cvmSet(si,i+1,j,cvmGet(si,i+1,j)+(a)*(1-b));
					if(i>=0 && i<imageWidth && j<imageWidth-1)
						cvmSet(si,i,j+1,cvmGet(si,i,j+1)+(1-a)*(b));
					if(i>=-1 && i<imageWidth-1 && j<imageWidth-1)
						cvmSet(si,i+1,j+1,cvmGet(si,i+1,j+1)+(a)*(b));
				}
				else
					continue;
			}
		}
		else
			continue;
	}
    return si;
}

double svlSpinImages::calculateCorrelation(CvMat *siA, CvMat *siB)
{
	// make sure that the two images have same dimension
	if(siA->height != siB->height || siA->width != siB->width)
		cerr<<"Dimensions don't agree!";
	int binsNum = siA->height*siA->width;
	double aSum = 0.0;
    double bSum = 0.0;
	double aSquareSum = 0.0;
    double bSquareSum = 0.0;
	double crossSum = 0.0;
	double correlation;
	for (int i = 0; i<siA->height; i++){
		for (int j = 0; j<siA->width; j++)
		{
			aSum += cvmGet(siA,i,j);
			bSum += cvmGet(siB,i,j);
			aSquareSum += cvmGet(siA,i,j) * cvmGet(siA,i,j);
			bSquareSum += cvmGet(siB,i,j) * cvmGet(siB,i,j);
			crossSum += cvmGet(siA,i,j) * cvmGet(siB,i,j);
		}
	}
	correlation = (binsNum*crossSum - aSum*bSum)/sqrt((binsNum*aSquareSum-aSum*aSum)*(binsNum*bSquareSum-bSum*bSum));
	return correlation;
}

double svlSpinImages::calculateCorrelation(vector<CvMat *>& si, unsigned sceneWidth, unsigned sceneHeight, CvRect r,CvMat *t)
{
	double max = 0;
	double current = 0;
	unsigned index; 
	for(unsigned dy = 0; dy < (unsigned)r.height; dy++)
		for(unsigned dx = 0; dx < (unsigned)r.width; dx++)
		{
			index = (dy + r.y) * sceneWidth + (dx + r.x);
			if(index<si.size()){
				current = calculateCorrelation(si[index],t);
				if(current > max) max = current;
			}
			else
				continue;
		}
	return max;
}

void svlSpinImages::readPointCloud(string sceneFileName)
{
	string currentFileName;
	currentFileName = sceneFileName + ".x.txt"; 
	cout<<"Loading X coordinates from "<<currentFileName<<endl;
	ifstream ifs(currentFileName.c_str());
	if (ifs.fail()) { cerr<<"Error Reading X file!"<<endl; }
	readMatrix(X,ifs);
	ifs.close();
	// read y coordinates
	currentFileName = sceneFileName + ".y.txt"; 
	cout<<"Loading Y coordinates from "<<currentFileName<<endl;
	ifs.open(currentFileName.c_str());
	if (ifs.fail()) { cerr<<"Error Reading Y file!"<<endl; }
	readMatrix(Y,ifs);
	ifs.close();
	// read z coordinates
	currentFileName = sceneFileName + ".z.txt";
	cout<<"Loading Z coordinates from "<<currentFileName<<endl;
	ifs.open(currentFileName.c_str());
	if (ifs.fail()) { cerr<<"Error Reading Z file!"<<endl; }
	readMatrix(Z,ifs);
	ifs.close();
	return;
}

