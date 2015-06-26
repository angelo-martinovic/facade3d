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
** FILENAME:    svlObjectList.cpp
** AUTHOR(S):   Ian Goodfellow <ia3n@cs.stanford.edu>
**
** DESCRIPTION:
**  Extensions to the standard svlObject2d
**
*****************************************************************************/

#include "svlObject2dExtensions.h"

svlObject2dRenderable::svlObject2dRenderable(const svlObject2d & o) : svlObject2d(o)
{
  color = svlPoint3d(1.0,1.0,0);
  thickness = 1;
}

svlObject2dRenderable::svlObject2dRenderable(const svlObject2d & o, const svlPoint3d & color) : svlObject2d(o), color(color)
{
  thickness = 1;
}

void svlObject2dRenderable::render(IplImage * frame) const
{
  SVL_ASSERT(frame->depth == IPL_DEPTH_8U);
  SVL_ASSERT(frame->nChannels == 3);

  cout << x << " " << y << " " << x+w << " " << y + h << endl;
  cout << frame->width << " " << frame->height <<  endl;

  cvRectangle(frame, cvPoint(x,y), cvPoint((int)(x+w),(int)(y+h)), cvScalar(color.x*255.0, color.y*255.0,color.z*255.0), thickness);

}