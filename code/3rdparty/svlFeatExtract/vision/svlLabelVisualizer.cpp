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
** FILENAME:    svlLabelVisualizer.cpp
** AUTHOR(S):   Ian Goodfellow <ia3n@cs.stanford.edu>
** DESCRIPTION: A class for visualizing object detection labels
**     
**
*****************************************************************************/

#include "svlLabelVisualizer.h"
#include "svlVision.h"

svlLabelVisualizer::svlLabelVisualizer(const string & name)
{
  _name = name;
  _windowEnabled = false;
  _windowFrozen = false;
  _content = NULL;
}

svlLabelVisualizer::~svlLabelVisualizer()
{
  if (_content)
    cvReleaseImage(&_content);
}


void svlLabelVisualizer::enableWindow()
{
  _windowEnabled = true;
  cvNamedWindow(_name.c_str(),CV_WINDOW_AUTOSIZE);
  _updateWindow();
}

void svlLabelVisualizer::addLabel(const svlObject2d & label, svlLabelType type)
{
  switch (type)
    {
    case SVL_TRUTH:
      _truth.push_back(label);
      break;
    case SVL_DETECTION:
      _detections.push_back(label);
      break;
    }

  _updateWindow();
}

void svlLabelVisualizer::addLabel(const svlObject2dRenderable & label, svlLabelType type)
{
  switch (type)
    {
    case SVL_TRUTH:
      _truth.push_back(label);
      break;
    case SVL_DETECTION:
      _detections.push_back(label);
      break;
    }

  _updateWindow();
}


bool svlLabelVisualizer::hasNoContent() const
{
  return (_truth.size() + _detections.size() == 0);
}

void svlLabelVisualizer::_updateWindow()
{
  if (_windowFrozen || !_windowEnabled)
    return;

  if (hasNoContent())
    {
      if (_content)
	{
	  cvReleaseImage(&_content);
	  cvDestroyWindow(_name.c_str());
	  cvNamedWindow(_name.c_str(), CV_WINDOW_AUTOSIZE);
	}
      return;
    }


  pair<int,int> results = _getBounds();

  int w = results.first +1 ;
  int h = results.second + 1;

  if (_content && (w != _content->width || h != _content->height))
      cvReleaseImage(&_content);

  if (!_content)
    _content = cvCreateImage(cvSize(w,h), IPL_DEPTH_8U, 3);
      

  cvZero(_content);

  for (unsigned i = 0; i < _truth.size(); i++)
    _truth[i].render(_content);

  for (unsigned i = 0; i < _detections.size(); i++)
    _detections[i].render(_content);

  cout << "Showing image... " << endl;
  cvShowImage(_name.c_str(), _content);
}
    
pair<int,int> svlLabelVisualizer::_getBounds() const
{
  int w = 0;
  int h = 0;

  for (unsigned i = 0; i < _truth.size(); i++)
    {
      if (int(_truth[i].w+_truth[i].x) > w)
	w = int(_truth[i].w+_truth[i].x);
      if (int(_truth[i].h+_truth[i].y) > h)
	h = int(_truth[i].h+_truth[i].y);
    }

  for (unsigned i = 0; i < _detections.size(); i++)
    {
      if (int(_detections[i].w+_detections[i].x) > w)
	w = int(_detections[i].w+_detections[i].x);
      if (int(_detections[i].h+_detections[i].y) > h)
	h = int(_detections[i].h+_detections[i].y);
    }

  return pair<int,int>(w,h);
}
