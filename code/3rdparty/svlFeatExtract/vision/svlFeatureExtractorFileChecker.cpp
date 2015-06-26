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
** FILENAME:    svlFeatureExtractorFileChecker.cpp
** AUTHOR(S):   Ian Goodfellow <ia3n@cs.stanford.edu>
** DESCRIPTION:
**
*****************************************************************************/

#include "svlFeatureExtractorFileChecker.h"

static const char* knownSubclasses[] = {
  "PatchDictionary",
  "CompositeFeatureExtractor",
  "HaarFeatureExtractor",
  NULL
  };

string svlFeatureExtractorFileChecker(const char * filepath)
{

  //TODO-- convert all feature extractors to use the same file format
  //as svlClassifier, and then default file checker will work instead
  //of having this function. I guess we'd still need this function around
  //to support legacy files though.

  //Can assume file exists, factory enforces that

   XMLNode root = XMLNode::parseFile(filepath);
  
   if (root.isEmpty())
    {
      SVL_LOG(SVL_LOG_WARNING, "Detector feature extractor file could not be parsed");
      return NULL;
    }

   if (!root.getName())
     {
       if (root.nChildNode() != 1)
	 {
	   SVL_LOG(SVL_LOG_WARNING, "Detector feature extractor file contains multiple roots");
	   return NULL;
	 }
       
       root = root.getChildNode(0);
       
       if (!root.getName())
	 {
	   SVL_LOG(SVL_LOG_WARNING, "Detector feature extractor file has no named XML root in any of the usual places");
	   return NULL;
	 }

     }


   //Try all possible root nodes until we find one that works
   int subclassIdx = 0;
   for(; knownSubclasses[subclassIdx]!=NULL; subclassIdx++)
     {
       
       //cout << filename << " " << knownSubclasses[subclassIdx] << endl;
       if (!strcmp(root.getName(), knownSubclasses[subclassIdx]))
	 return string(knownSubclasses[subclassIdx]);
     }
   
   return "";
}
