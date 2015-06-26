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
** FILENAME:    svlPixelFeatureExtractor.cpp
** AUTHOR(S):   Olga Russakovsky <olga@cs.stanford.edu>
**
*****************************************************************************/

#include "svlPixelFeatureExtractor.h"

// svlPixelFeatureExtractor ----------------------------------------------------

static inline CvPoint scaleDownCvPoint(CvPoint pt, double scale)
{
  return cvPoint((int)(pt.x / scale), (int)(pt.y / scale));
}

void svlPixelFeatureExtractor::extract(const vector<CvPoint> &locations,
					 const vector<svlDataFrame*> &frames,
					 vector<vector<double> > &output,
					 bool sparse,
					 int outputOffset) const
{
  svlImageFrame * imgFrame = dynamic_cast<svlImageFrame *>(frames[0]);

  if (!imgFrame)
    SVL_LOG(SVL_LOG_FATAL, "Channel 0" 
	    << " is not an image channel, yet was passed to basic svlPixelFeatureExtractor");

  getDescriptors(imgFrame->image, locations, output, NULL, outputOffset);
}

// turns on the parallelized implementation of getDescriptors;
// currently, slower than the straight-forward implementation but
// may be helpful in the future...
//#define PARALLELIZE_GET_DESCRIPTORS

#ifndef PARALLELIZE_GET_DESCRIPTORS
void svlPixelFeatureExtractor::getDescriptors(const IplImage *image,
					      const vector<CvPoint> &locations,
					      vector<vector<double> > &output,
					      const vector<float> *scales,
					      int outputOffset) const
{
  if (locations.size() == 0) return;
  if (output.size() != locations.size())
    output.resize(locations.size());

  svlImageBufferManager manager(_numBuffers);

  if (scales == NULL || scales->size() == 0) {
    // if scales not passed in, assume all are 1
    vector<IplImage *> responses;
    preprocessImage(responses, image);

    for (unsigned i = 0; i < locations.size(); i++)
      getDescriptor(responses, locations[i], output[i], outputOffset, manager);
    
    for (unsigned i = 0; i < responses.size(); i++)
      cvReleaseImage(&responses[i]);
  } else {
    SVL_ASSERT(locations.size() == scales->size());
    
    unsigned curr_scale_index = 0;
    while (curr_scale_index < scales->size()) {
      float curr_scale = scales->at(curr_scale_index);
      unsigned next_scale_index = curr_scale_index;
      
      while (next_scale_index < scales->size() &&
	     scales->at(next_scale_index) == curr_scale)
	next_scale_index++;

      vector<IplImage *> responses;
      preprocessImage(responses, image, curr_scale);
      for (unsigned i = curr_scale_index; i < next_scale_index; i++)
	getDescriptor(responses, scaleDownCvPoint(locations[i], curr_scale),
		      output[i], outputOffset, manager);
            
      for (unsigned i = 0; i < responses.size(); i++)
	cvReleaseImage(&responses[i]);
      curr_scale_index = next_scale_index;
    } 
  }

  SVL_ASSERT(output.size() == locations.size());
}
#endif 

void svlPixelFeatureExtractor::preprocessImage(vector<IplImage *> &responses,
					       const IplImage *cimg, float scale) const {
  // smooth the image at the required scale
  IplImage *result = create32Fimage(cimg);
  if (result->nChannels > 1) {
    svlChangeColorModel(result, SVL_COLOR_GRAY);
  }

  int n = kernelSize(scale);
  cvSmooth(result, result, CV_GAUSSIAN, n, n, scale);

  // resize image (locations resized in extractWithScales)
  resizeInPlace(&result, (int)(result->height/scale), (int)(result->width/scale));

  responses.clear();
  responses.push_back(result);
}






#ifdef PARALLELIZE_GET_DESCRIPTORS

#define NUM_THREADS 8

struct getDescriptorThreadedArgs {
  getDescriptorThreadedArgs(const svlPixelFeatureExtractor *e,
			    vector<vector<IplImage *> > *r,
			    CvPoint loc, vector<double> *output, int oo,
			    vector<svlImageBufferManager> *ms) :
    extractor(e), responses(r), loc(loc), output(output), 
    outputOffset(oo), managers(ms) {}
  const svlPixelFeatureExtractor *extractor;
  vector<vector<IplImage *> > *responses;
  CvPoint loc;
  vector<double> *output;
  int outputOffset;
  vector<svlImageBufferManager> *managers;
};

void *getDescriptorThreaded(void *voidArgs, unsigned tid)
{
  getDescriptorThreadedArgs *args = (getDescriptorThreadedArgs *)voidArgs;
  args->extractor->getDescriptor(args->responses->at(tid),
				 args->loc, *(args->output),
				 args->outputOffset, args->managers->at(tid));
  delete args;
  return NULL;
}

static void makeCopies(vector<vector<IplImage *> > &v)
{
  for (unsigned i = 1; i < v.size(); i++) {
    v[i].resize(v[0].size());
    for (unsigned j = 0; j < v[0].size(); j++)
      v[i][j] = cvCloneImage(v[0][j]);
  }
}

void svlPixelFeatureExtractor::getDescriptors(const IplImage *image,
					      const vector<CvPoint> &locations,
					      vector<vector<double> > &output,
					      const vector<float> *scales,
					      int outputOffset) const
{
  if (locations.size() == 0) return;
  if (output.size() != locations.size())
    output.resize(locations.size());

  svlThreadPool threadPool(NUM_THREADS);
  unsigned numThreads = threadPool.numThreads();
  unsigned numCopies = numThreads > 0 ? numThreads : 1; // to make thread-safe
  vector<svlImageBufferManager> managers(numCopies, svlImageBufferManager(_numBuffers));

  if (scales == NULL || scales->size() == 0) {
    // if scales not passed in, assume all are 1
    vector<vector<IplImage *> > responses(numCopies);
    preprocessImage(responses[0], image);
    makeCopies(responses); // to make thread-safe

    threadPool.start();
    for (unsigned i = 0; i < locations.size(); i++)
      threadPool.addJob(getDescriptorThreaded,
			new getDescriptorThreadedArgs(this, &responses,
						      locations[i], &(output[i]),
						      outputOffset, &managers));
    threadPool.finish();

    for (unsigned i = 0; i < responses.size(); i++)
      for (unsigned j = 0; j < responses[i].size(); j++)
	cvReleaseImage(&responses[i][j]);
  } else {
    SVL_ASSERT(locations.size() == scales->size());
    
    unsigned curr_scale_index = 0;
    while (curr_scale_index < scales->size()) {
      float curr_scale = scales->at(curr_scale_index);
      unsigned next_scale_index = curr_scale_index;
      
      while (next_scale_index < scales->size() &&
	     scales->at(next_scale_index) == curr_scale)
	next_scale_index++;

      vector<vector<IplImage *> > responses(numCopies);
      preprocessImage(responses[0], image, curr_scale);
      makeCopies(responses); // to make thread-safe
      threadPool.start();
      for (unsigned i = curr_scale_index; i < next_scale_index; i++)
	threadPool.addJob(getDescriptorThreaded,
			  new getDescriptorThreadedArgs(this, &responses,
							scaleDownCvPoint(locations[i], curr_scale),
							&(output[i]), outputOffset, &managers));
      threadPool.finish();
      
      for (unsigned i = 0; i < responses.size(); i++)
	for (unsigned j = 0; j < responses[i].size(); j++)
	  cvReleaseImage(&responses[i][j]);
      curr_scale_index = next_scale_index;
    } 
  }

  SVL_ASSERT(output.size() == locations.size());
}
#endif
