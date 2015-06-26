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
** FILENAME:    svlLabelVisualizer.h
** AUTHOR(S):   Ian Goodfellow <ia3n@cs.stanford.edu>
** DESCRIPTION: A class for visualizing object detection labels
**     
**
*****************************************************************************/

#pragma once

#include <string>
using namespace std;
#include "svlObjectList.h"
#include "svlObject2dExtensions.h"
#include "cv.h"

enum svlLabelType
  {
    SVL_TRUTH,
    SVL_DETECTION
  };

class svlLabelVisualizer
{

 public:

  //name is just the display name for the title bar of the window
  svlLabelVisualizer(const string & name);
  ~svlLabelVisualizer();

  //Display the window
  void enableWindow();

  //Add objects to the visualizer (TODO: most people would probably like to be able to add a background image too)
  void addLabel(const svlObject2d & label, svlLabelType type);
  void addLabel(const svlObject2dRenderable & label, svlLabelType type);
  
  //Use freeze to prevent each call to addLabel from re-rendering the window
  void freezeWindow() { _windowFrozen = true; }
  void thawWindow() { _windowFrozen = false; _updateWindow(); }

  //If true, there are no labels/images in the visualizer
  bool hasNoContent() const;

 protected:

  void _updateWindow();
  pair<int,int> _getBounds() const;

 protected:


  string _name;
  bool _windowEnabled;
  bool _windowFrozen;

  svlObject2dFrameRenderable _truth;
  svlObject2dFrameRenderable _detections;

  IplImage * _content;


};
