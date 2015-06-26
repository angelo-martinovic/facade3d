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
** FILENAME:    svlFeatureExtractor.cpp
** AUTHOR(S):   Ethan Dreyfuss <ethan@cs.stanford.edu>
**              Ian Goodfellow <ia3n@cs.stanford.edu>
** DESCRIPTION:
**
*****************************************************************************/

#include "svlFeatureExtractor.h"
#include "svlFeatureExtractorFileChecker.h"
#include "svlVision.h"
//#include "svlImageFrame.h"
#include "svlHaarFeatureExtractor.h"
#include "svlCompositeFeatureExtractor.h"
#include "svlPatchDictionary.h"

bool svlFeatureExtractor::write(const string &filename)
{
  return this->write(filename.c_str());
}

bool svlFeatureExtractor::write(const char *filename)
{
  ofstream ofs(filename);
  bool rval = this->writeOut(ofs);
  ofs.close();
  return rval;
}

//Default implementation of extract that takes IplImage* channels and
//boxes them up into svlImageFrames
void svlFeatureExtractor::extract(const vector<CvPoint> &locations, 
				  const vector<IplImage*> &channels, 
				  vector< vector<double> > &output,
				  bool sparse,
				  int outputOffset) const
{
  vector<svlDataFrame*> boxedChannels(channels.size());
  for(int i=0; i<int(channels.size()); i++)
    {
      svlImageFrame *newFrame = new svlImageFrame();
      newFrame->image = channels[i];
      boxedChannels[i] = newFrame;
    }

  this->extract(locations, boxedChannels, output, sparse, outputOffset);
  
  for(int i=0; i<int(boxedChannels.size()); i++)
    delete boxedChannels[i];
}

void svlFeatureExtractor::windowResponse(const vector<IplImage *> & channels, vector<double> & output,
					 int outputOffset, const vector<bool> *usedFetures) const
{
  CvPoint location = cvPoint(0,0);
  vector<CvPoint> locations;
  locations.push_back(location);
  vector<vector<double> > outputs;

  /* TODO: fix */
  extract(locations, channels, outputs, true, outputOffset);

  assert(outputs.size() == 1);

  if (output.size())
    {
      if (output.size() < outputOffset + outputs[0].size())
	output.resize(outputOffset + outputs[0].size());

      //TODO-- come up with something better than this slow copy operation
      for (unsigned i =  0; i < outputs[0].size(); i++)
	output[outputOffset + i] = outputs[0][i];
    }
  else
    output = outputs[0];
}

string svlFeatureExtractor::summary() const
{
  return string("[No summary available]");
}

bool svlFeatureExtractor::load(const char * filepath)
{
  XMLNode root = XMLNode::parseFile(filepath);

  return load(root);
}

void svlFeatureExtractor::setSize(CvSize size)
{
  _windowSize = size;
}

svlFactory<svlFeatureExtractor> & svlFeatureExtractorFactory()
{
  static svlFactory<svlFeatureExtractor> factory("svlFeatureExtractor");
  return factory;
}


SVL_AUTOREGISTER_FILECHECKER_CPP( svlFeatureExtractorFileChecker, svlFeatureExtractor);
SVL_AUTOREGISTER_CPP( CompositeFeatureExtractor , svlFeatureExtractor, svlCompositeFeatureExtractor);
SVL_AUTOREGISTER_CPP( PatchDictionary , svlFeatureExtractor, svlPatchDictionary);
SVL_AUTOREGISTER_CPP( HaarFeatureExtractor , svlFeatureExtractor, svlHaarFeatureExtractor);
SVL_AUTOREGISTER_CPP( RIFTdescriptor , svlFeatureExtractor, svlRIFTdescriptor);
SVL_AUTOREGISTER_CPP( SpinDescriptor , svlFeatureExtractor, svlSpinDescriptor);
SVL_AUTOREGISTER_CPP( PixelDictionary , svlFeatureExtractor, svlPixelDictionary);
SVL_AUTOREGISTER_CPP( TextonExtractor , svlFeatureExtractor, svlTextonExtractor);
