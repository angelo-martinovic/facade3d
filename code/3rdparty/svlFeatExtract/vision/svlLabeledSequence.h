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
** FILENAME:    svlLabeledSequence.h
** AUTHOR(S):   Ian Goodfellow <ia3n@cs.stanford.edu>
**         
**             
** DESCRIPTION:
**  Wrapper class for svlImageSequence and svlObject2dSequence that makes it
**  easy to access both an image and its labels in the same location
**
*****************************************************************************/

#pragma once
#include "svlObjectList.h"
#include "svlImageSequence.h"

struct svlLabeledFrame
{
  const svlObject2dFrame * labels;
  const IplImage * image;

};

class svlLabeledSequence
{
 public:

  //File loading methods  
  void load(const char * imageSequencePath, const char * labelSequencePath);
  void loadImages(const char * filepath);
  void loadLabels(const char * filepath);

  //Access methods
  unsigned numImages() const;
  svlLabeledFrame frame(unsigned index);
  const IplImage * image(unsigned index);
  const svlObject2dFrame * labels(unsigned index);
  const string & operator[](unsigned index); //return name of image at index


 private:

  svlImageSequence _images;
  svlObject2dSequence _labels;

};
