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
** FILENAME:    svlCompositeFeatureExtractor.cpp
** AUTHOR(S):   Ethan Dreyfuss <ethan@cs.stanford.edu>
**              Ian Goodfellow <ia3n@cs.stanford.edu>
** DESCRIPTION:
**   See svlCompositeFeatureExtractor.h.
**
*****************************************************************************/

#include "svlCompositeFeatureExtractor.h"
#include "xmlParser/xmlParser.h"
#include "svlBase.h"

#define FOREACH for(extractorVec::const_iterator itr = delegates.begin(); itr != delegates.end(); ++itr)

svlCompositeFeatureExtractor::svlCompositeFeatureExtractor() 
{

}

svlCompositeFeatureExtractor::svlCompositeFeatureExtractor(const extractorVec &vec)
  : delegates(vec)
{

}

unsigned svlCompositeFeatureExtractor::numFeatures() const
{
  unsigned rval = 0;
  FOREACH
    {
      rval += (*itr)->numFeatures();
    }
  return rval;
}

bool svlCompositeFeatureExtractor::writeOut(ofstream &ofs)
{
  bool rval = true;
  ofs<<"<CompositeFeatureExtractor Version=1.0>"<<endl;
  FOREACH
    {
      rval &= (*itr)->writeOut(ofs);
    }
  ofs<<"</CompositeFeatureExtractor>"<<endl;
  return rval;
}

bool svlCompositeFeatureExtractor::load(XMLNode &root)
{
  if (root.isEmpty()) {
    SVL_LOG(SVL_LOG_WARNING, "could not read CompositeFeatureExtractor ");
    return false;
  }

  if (string(root.getName()) != "CompositeFeatureExtractor" ) {
    SVL_LOG(SVL_LOG_WARNING, "Not a composite feature extractor file: ");
    return false;
  }

  int numChildExtractors = root.nChildNode();
  for(int i=0; i<numChildExtractors; i++)
    {
      SVL_LOG(SVL_LOG_MESSAGE, "Loading composite extractor child "<<i);

      XMLNode child = root.getChildNode(i);
      svlFeatureExtractor* childExtractor = svlFeatureExtractorFactory().load(child);

      if (childExtractor)
	{
	  SVL_LOG(SVL_LOG_MESSAGE, "childExtractor has width "<<childExtractor->windowWidth());
	}
      else
	SVL_LOG(SVL_LOG_MESSAGE, "childExtractor failed to load");

      if(childExtractor == NULL)
	{
	  SVL_LOG(SVL_LOG_WARNING, "Child node #"<<i<<" of CompositeFeatureExtractor is malformed or unreadable");
	  return false;
	}

      if (i == 0)
	{
	  _windowSize.width = childExtractor->windowWidth();
	  _windowSize.height = childExtractor->windowHeight();
	}
      else
	{
	  if (childExtractor->windowWidth() != (unsigned) _windowSize.width ||  childExtractor->windowHeight() != (unsigned) _windowSize.height)
	    {
	      SVL_LOG(SVL_LOG_WARNING, "Composite extractor file specifies child extractors with inconsistent window sizes ( ("<< _windowSize.width << ", "<<_windowSize.height<<") versus ("<<childExtractor->windowWidth() <<", "<<childExtractor->windowHeight()<<") ");
	      delete childExtractor;
	      return false;
	    }
	}

      delegates.push_back(childExtractor);
    }

  return true;
}

svlFeatureExtractor* 
svlCompositeFeatureExtractor::getPrunedExtractor(const vector<bool> &featureUsedBits) const
{
  extractorVec newDelegates;
  vector<bool>::const_iterator start = featureUsedBits.begin();
  FOREACH
    {
      int numFeatsInCurrent=(*itr)->numFeatures();
      vector<bool> usedBits(start, start+numFeatsInCurrent);
      newDelegates.push_back((*itr)->getPrunedExtractor(usedBits));
      start += numFeatsInCurrent;
    }
  return new svlCompositeFeatureExtractor(newDelegates);
}

svlFeatureExtractor*
svlCompositeFeatureExtractor::clone() const
{
  extractorVec newDelegates;
  FOREACH
    {
      newDelegates.push_back((*itr)->clone());
    }
  return new svlCompositeFeatureExtractor(newDelegates);
}

void svlCompositeFeatureExtractor::extract(const vector<CvPoint> &locations, 
		       const vector<svlDataFrame*> &frames, 
		       vector< vector<double> > &output,
		       bool sparse,
		       int outputOffset) const
{  
  int offset = 0;
  FOREACH
    {
      (*itr)->extract(locations, frames, output, sparse, outputOffset+offset);
      offset += (*itr)->numFeatures();
    }
}

void svlCompositeFeatureExtractor::windowResponse(const vector<IplImage *> & channels,
                              vector<double> & output,
                              int outputOffset) const
{
  int offset = 0;
  FOREACH
    {
      (*itr)->windowResponse(channels, output, offset);
      offset += (*itr)->numFeatures();
    }
}

string svlCompositeFeatureExtractor::summary() const
{
  stringstream s;
  s << "[";
  for (unsigned i = 0; i < delegates.size(); i++)
    s << delegates[i]->summary();
  
  s << "]";

  return s.str();
}



