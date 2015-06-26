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
** FILENAME:    svlTextonFilterBank.h
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
** DESCRIPTION:
**  Implements the 17-dimensional filter bank defined in
**    [1] J. Shotton, J. Winn, C. Rother, A. Criminisi.
**        "TextonBoost for Image Understanding: Multi-Class Object Recognition
**        and Segmentation by Jointly Modeling Texture, Layout, and Context,"
**        IJCV 2008.
**    [2] J. Winn, A. Criminisi, and T. Minka.
**        "Categorization by learned universal visual dictionary," ICCV 2005.
**
*****************************************************************************/

#pragma once

#include <vector>

#include "cv.h"
#include "cxcore.h"

// svlTextonFilterBank ------------------------------------------------------

class svlTextonFilterBank {
 protected:
    double _kappa;

 public:
    static const int NUM_FILTERS = 17;

    svlTextonFilterBank(double k = 1.0);
    virtual ~svlTextonFilterBank();

    // Filtering function. The caller must provide a vector of CV32F destination 
    // images the same size as the source image or NULL. The source image should
    // be a 3-channel RGB color image (it is automatically converted to CIELab).
    void filter(IplImage *img, std::vector<IplImage *> &response);
};


