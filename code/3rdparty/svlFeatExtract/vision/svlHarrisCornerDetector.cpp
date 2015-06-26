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
** FILENAME:    svlHarrisCornerDetector.cpp
** AUTHOR(S):   Ian Goodfellow <ia3n@cs.stanford.edu>
**
*****************************************************************************/


#include "svlVision.h"
#include "svlHarrisCornerDetector.h"

svlHarrisCornerDetector::svlHarrisCornerDetector(int blockSize,
						 int apertureSize,
						 double k,
						 int maxWinWidth,
						 int maxWinHeight,
						 double responseThreshold) :
  _blockSize(blockSize), _apertureSize(apertureSize), _k(_k), 
  _maxWinWidth(maxWinWidth), _maxWinHeight(maxWinHeight),
  _responseThreshold(responseThreshold), _showResponse(false)
{
}

void svlHarrisCornerDetector::findInterestPoints(const IplImage * img,
						 vector<CvPoint> & output)
{
  CvMat * harrisResponse = cvCreateMat(img->height, img->width, CV_32FC1);
  assert(harrisResponse);


  cvCornerHarris(img, harrisResponse, _blockSize, _apertureSize, _k);

 



  svlFindLocalMaxima(harrisResponse, output, _maxWinWidth, _maxWinHeight);
  
  vector<CvPoint>::iterator i = output.begin();

  while (i != output.end())
    {
      if (cvmGet(harrisResponse, i->y, i->x) < _responseThreshold)
	output.erase(i);
      else
	++i;
    }
   

  if (_showResponse)
    {
#define WINDOW_NAME "Harris Response"
 scaleToRange(harrisResponse,0,1);
      cvNamedWindow(WINDOW_NAME, CV_WINDOW_AUTOSIZE);
      cvShowImage(WINDOW_NAME, harrisResponse);
      cvWaitKey(0);
    }

  cvReleaseMat(&harrisResponse);
}

void svlHarrisCornerDetector::write(ofstream & ofs) const
{
  ofs << "<IPD \n";
  ofs << "type =\"harris\"\n";
  ofs << "blockSize =\""<<_blockSize<<"\"\n";
  ofs << "apertureSize =\""<<_apertureSize<<"\"\n";
  ofs << "k = \""<<_k<<"\"\n";
  ofs << "maxWinWidth = \"" << _maxWinWidth << "\"\n";
  ofs << "maxWinHeight = \"" << _maxWinHeight << "\"\n";
  ofs << "responseThreshold = \"" << _responseThreshold << "\"\n";
  ofs << "showResponse = \"" << _showResponse << "\"\n";
  ofs << ">\n";
  ofs << "</IPD>\n";
}


svlHarrisCornerDetector:: svlHarrisCornerDetector(XMLNode & node)
{
  _blockSize = atoi(node.getAttribute("blockSize"));
  _apertureSize = atoi(node.getAttribute("apertureSize"));
  _k = atof(node.getAttribute("k"));
  _maxWinWidth = atoi(node.getAttribute("maxWinWidth"));
  _maxWinHeight = atoi(node.getAttribute("maxWinHeight"));
  _responseThreshold = atof(node.getAttribute("responseThreshold"));
  _showResponse = (atoi(node.getAttribute("showResponse")) != 0);
}

bool svlHarrisCornerDetector::operator!=(const svlInterestPointDetector & other) const
{
  
  const svlHarrisCornerDetector * o = dynamic_cast<const svlHarrisCornerDetector *>( &other);

if (o == NULL)
  return true;


if (_blockSize != o->_blockSize)
  return true;

if (_apertureSize != o->_apertureSize)
  return true;


if (!svlFloatCompare(_k,o->_k))
  return true;


if (_maxWinWidth != o->_maxWinWidth)
  return true;

if (_maxWinHeight != o->_maxWinHeight)
  return true;

if (_responseThreshold != o->_responseThreshold)
  return true;

if (_showResponse != o-> _showResponse)
  return true;


return false;
}
