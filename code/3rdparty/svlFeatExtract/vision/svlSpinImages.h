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
** FILENAME:    svlSpinImages.h
** AUTHOR(S):   Chih-Wei Chen <louistw@stanford.edu>
** DESCRIPTION:
**   Stores the point cloud data, and generate the spin-images.
**
*****************************************************************************/

#pragma once

#include <vector>

#include "cv.h"
#include "cxcore.h"

using namespace std;

class svlSpinImages {
 public:
    unsigned w, h; // width, height of the point cloud scene
    CvMat *X; // X, Y, Z coordinates of the point cloud
    CvMat *Y;
    CvMat *Z;
    CvMat *nX; // X, Y, Z coordinates of the point cloud normal vectors
    CvMat *nY;
    CvMat *nZ;
    // spin image generation parameters
    float binSize;
    int imageWidth;
    
 public:
    svlSpinImages(int height, int width);
    virtual ~svlSpinImages();
    
    void setscene(CvMat *x, CvMat *y, CvMat *z);
    void setscene(CvMat *x, CvMat *y, CvMat *z, CvMat *nx, CvMat *ny, CvMat *nz);
    void calculateNormals();
    // Compute spin-image at a given point. Returns the 2D spin-image
    CvMat *siCalculate(int y, int x);
    // Compute the maximum correlation of a scene to a spin-image over a window
    static double calculateCorrelation(vector<CvMat *>& si, unsigned sceneWidth, unsigned sceneHeight, CvRect r,CvMat *t);
    // Compute the correlation between two spin-images 
    static double calculateCorrelation(CvMat *siA, CvMat *siB);
    // Read point cloud data from file
    void readPointCloud(string sceneFileName);
};

