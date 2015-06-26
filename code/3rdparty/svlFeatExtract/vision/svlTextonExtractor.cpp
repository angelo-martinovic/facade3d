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
** FILENAME:    svlTextonExtractor.cpp
** AUTHOR(S):   Olga Russakovsky <olga@cs.stanford.edu>
** DESCRIPTION:
**
*****************************************************************************/

#include "svlTextonExtractor.h"

svlTextonExtractor::svlTextonExtractor(const svlTextonExtractor &t)
{
  _windowSize = t._windowSize;
  _numBuffers = t._numBuffers;
}

void svlTextonExtractor::preprocessImage(vector<IplImage *> &responses,
					 const IplImage *cimg, float scale) const
{
  IplImage *img = cvCreateImage(cvGetSize(cimg), cimg->depth, cimg->nChannels);
  int n = kernelSize(scale);
  cvSmooth(cimg, img, CV_GAUSSIAN, n, n, scale);

  // resize image (locations resized in extractWithScales)
  resizeInPlace(&img, (int)(img->height/scale), (int)(img->width/scale));

  svlTextonFilterBank tfb;
  tfb.filter(img, responses);
  cvReleaseImage(&img);
}

void svlTextonExtractor::getDescriptor(const vector<IplImage *> &responses,
				       CvPoint loc, vector<double> &output, int offset) const
{
  output.resize(offset + responses.size());
  for (unsigned i = 0; i < responses.size(); i++)
    output[offset + i] = CV_IMAGE_ELEM(responses[i], float, loc.y, loc.x);
}

bool svlTextonExtractor::writeOut(ofstream &ofs)
{
  if (ofs.fail()) return false;
  ofs << "<svlFeatureExtractor id=\"TextonExtractor\"/>\n";  
  return true;
}

bool svlTextonExtractor::load(const char * filepath)
{
  XMLNode root = XMLNode::parseFile(filepath);
  return load(root);
}


bool svlTextonExtractor::load(XMLNode &root)
{
  if (root.isEmpty()) {
    SVL_LOG(SVL_LOG_WARNING, "Texton dictionary file is missing or has incorrect root.");
    return false;
  }

  if (string(root.getAttribute("id")) != "TextonExtractor" ) {
    SVL_LOG(SVL_LOG_WARNING, "Attempt to read texton dictionary from XMLNode that is not a texton dictionary.");
    return false;
  }

  return true;
}

svlFeatureExtractor* svlTextonExtractor::getPrunedExtractor(const vector<bool> &featureUsedBits) const
{
  SVL_LOG(SVL_LOG_FATAL, "not implemented yet");
  return NULL; // to suppress warnings
}

svlFeatureExtractor * svlTextonExtractor::clone() const
{
  return dynamic_cast<svlFeatureExtractor *>(new svlTextonExtractor(*this));
}




