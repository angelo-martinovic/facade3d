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
** FILENAME:    svlObjectList.h
** AUTHOR(S):   Ian Goodfellow <ia3n@cs.stanford.edu>
**
** DESCRIPTION:
**  Extensions to the standard svlObject2d
**
*****************************************************************************/

#pragma once

#include "svlObjectList.h"
#include "cv.h"

//color is assumed to be RGB [0,1]. Will render with appropriate
//transformations//when drawing to other formats (e.g., will multiply
//by 255 automatically when rendering to 8UC3 format)
class svlObject2dRenderable : public svlObject2d
{
 public:

  svlObject2dRenderable(const svlObject2d & o);
  svlObject2dRenderable(const svlObject2d & o, const svlPoint3d & color);

  void render(IplImage * frame) const; // currently only 8UC3 is supported

  svlPoint3d color;

  int thickness;

};

typedef std::vector<svlObject2dRenderable> svlObject2dFrameRenderable;
