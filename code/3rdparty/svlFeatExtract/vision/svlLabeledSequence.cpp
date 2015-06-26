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
** FILENAME:    svlLabeledSequence.cpp
** AUTHOR(S):   Ian Goodfellow <ia3n@cs.stanford.edu>
**         
**             
** DESCRIPTION:
**  Wrapper class for svlImageSequence and svlObject2dSequence that makes it
**  easy to access both an image and its labels in the same location
**
*****************************************************************************/

#include "svlLabeledSequence.h"

void svlLabeledSequence::load(const char * imageSequencePath, const char * labelSequencePath)
{
  loadImages(imageSequencePath);
  loadLabels(labelSequencePath);
}

void svlLabeledSequence::loadImages(const char * filepath)
{
  _images.load(filepath);
}

void svlLabeledSequence::loadLabels(const char * filepath)
{
  _labels.read(filepath);
}

unsigned svlLabeledSequence::numImages() const
{
  return _images.numImages();
}

svlLabeledFrame svlLabeledSequence::frame(unsigned index)
{


  svlLabeledFrame rval;
  rval.image = image(index);
  rval.labels = labels(index);

  assert(rval.image);
  

  return rval;
}

const IplImage * svlLabeledSequence::image(unsigned index)
{
  const IplImage * rval =  _images.image(index);


  return rval;
}

const svlObject2dFrame * svlLabeledSequence::labels(unsigned index)
{
  string imageId = strBaseName(_images[index]);
  svlObject2dSequence::const_iterator iter = _labels.find(imageId);
  if (iter == _labels.end())
    return NULL;
  else
    return & iter->second;
}

const string & svlLabeledSequence::operator[](unsigned index)
{
  return _images[index];
}
