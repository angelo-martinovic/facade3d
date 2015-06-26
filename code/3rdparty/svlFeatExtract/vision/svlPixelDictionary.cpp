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
** FILENAME:    svlPixelDictionary.cpp
** AUTHOR(S):   Olga Russakovsky <olga@cs.stanford.edu>
** DESCRIPTION:
**
*****************************************************************************/

#include "svlPixelDictionary.h"

svlPixelDictionary::svlPixelDictionary(int w, int h) :
  _ipDetector(NULL), _numPointsPerSample(0),  _codebookTrained(false), _validChannel(0)
{
  _windowSize = cvSize(w, h);
}

svlPixelDictionary::svlPixelDictionary(const svlPixelDictionary &d) :
  _numPointsPerSample(d._numPointsPerSample),
  _codebook(d._codebook), _codebookTrained(d._codebookTrained),
  _validChannel(d._validChannel)
{
  _windowSize = d._windowSize;

  if (d._ipDetector) {
    _ipDetector = new svlHarrisLaplaceDetector(*(d._ipDetector));
  } else {
    _ipDetector = NULL;
  }

  _descriptors.resize(d._descriptors.size());
  for (unsigned i = 0; i < d._descriptors.size(); i++)
    _descriptors[i] = dynamic_cast<svlPixelFeatureExtractor *>(d._descriptors[i]->clone());
}

svlPixelDictionary::~svlPixelDictionary()
{
  clear();
}

void svlPixelDictionary::clear() {
  if (_ipDetector) {
    delete _ipDetector;
    _ipDetector = NULL;
  }

  for (unsigned i = 0; i < _descriptors.size(); i++) {
    if (_descriptors[i])
      delete _descriptors[i];
  }
  _descriptors.clear();

  _codebookTrained = false;
}

void svlPixelDictionary::clearDetector()
{
  if (_ipDetector)
    delete _ipDetector;
  _ipDetector = NULL;
}

void svlPixelDictionary::buildDictionary(const vector<IplImage*> &samples,
							int numClusters,
							vector<vector<double> > *featureVectors)
{
  SVL_ASSERT_MSG(_descriptors.size() > 0, "must set at least one descriptor before training");

  SVL_LOG(SVL_LOG_VERBOSE, "Building dictionary from " << samples.size() << " examples");

  if (samples.size() == 0) {
    SVL_LOG(SVL_LOG_WARNING, "No samples to train dictionary on; quitting");
    return;
  }

  if (!_ipDetector)
    srand(time(NULL)); // will  be choosing random points

  // if width and height are not specified, use the given from the
  // samples (assumes all are same size)
  if (_windowSize.width == -1 || _windowSize.height == -1) {
    _windowSize = cvGetSize(samples[0]);
  }

  // structure to save the descriptors for every training sample
  vector<vector<vector<double> > > allDescriptors;
  allDescriptors.resize(samples.size());
  
  vector<CvPoint> points; // these don't really need to be saved for every image
  unsigned totalNumSamples = 0;
  int descriptorSize = 0;
  for (unsigned i = 0; i < samples.size(); i++) {
    extractDescriptors(samples[i], points, allDescriptors[i]);
    SVL_LOG(SVL_LOG_VERBOSE, "Sample " << i << ": " << allDescriptors[i].size() << " descriptors");
    totalNumSamples += allDescriptors[i].size();
    if (allDescriptors[i].size() > 0)
      descriptorSize = allDescriptors[i][0].size();
  }
  // all descriptors assumed to be the same size
  CvMat *examples = cvCreateMat(totalNumSamples, descriptorSize, CV_32FC1);

  // iterate over all images
  int index = 0;
  for (unsigned i = 0; i < allDescriptors.size(); i++) {
    // iterate over all interest points within this image
    for (unsigned j = 0; j < allDescriptors[i].size(); j++, index++) {
      // iterate over the descriptor of that interest point
      for (unsigned k = 0; k < allDescriptors[i][j].size(); k++) {
	cvmSet(examples, index, k, allDescriptors[i][j][k]);
      }
    }
  }

  _codebook.learn(examples, numClusters);
  _codebookTrained = true;
  cvReleaseMat(&examples);

  if (!featureVectors) return;

  // builds the histograms of codewords to serve as location descriptors
  // for each location
  if (featureVectors->size() < samples.size())
    featureVectors->resize(samples.size());

  svlHistogram hist(numClusters);
  for (unsigned i = 0; i < allDescriptors.size(); i++) {
    hist.set(&(featureVectors->at(i)));

    // iterate over all interest points within this image
    for (unsigned j = 0; j < allDescriptors[i].size(); j++) {
      hist.insert(_codebook.encode(allDescriptors[i][j]));
    }

    hist.normalize();
  }
}

void svlPixelDictionary::extract(const vector<CvPoint> &locations,
						const vector<svlDataFrame*> &frames,
						vector< vector<double> > &output,
						bool sparse,
						int outputOffset) const
{
  SVL_ASSERT_MSG(_descriptors.size() > 0,
		 "must set at least one descriptor beforehand");

  svlImageFrame * imgFrame = dynamic_cast<svlImageFrame *>(frames[_validChannel]);

  if (!imgFrame)
    SVL_LOG(SVL_LOG_FATAL, "Channel " << _validChannel
	    << " is not an image channel, yet was passed to svlPixelDictionary");

  IplImage * img = imgFrame->image;

  if (output.size() != locations.size())
    output.resize(locations.size());

  // if codebook wasn't trained, just extracts all of the descriptors
  if (!_codebookTrained) {
    unsigned offset = outputOffset;
    for (unsigned i = 0; i < _descriptors.size(); i++) {
      _descriptors[i]->getDescriptors(img, locations, output, NULL, offset);
      offset += _descriptors[i]->numFeatures();
    }
    return;
  }

  // invariant: codebook is trained, thus will be extracting features over
  // _windowSize regions. If IP detector is not specified, want about
  // _numPointsPerSample in each of those regions. Thus if the input is
  // not sparse and we're running sliding windows, the total number
  // of points that we want is
  int numPointsFullImage = (int)((float)_numPointsPerSample * 
				 (float)img->width / _windowSize.width *
				 (float)img->height / _windowSize.height);

  vector<CvPoint> points;
  vector<vector<double> > descriptors;
  extractDescriptors(img, points, descriptors, sparse ? NULL : &numPointsFullImage);
  SVL_LOG(SVL_LOG_VERBOSE, "Sample: " << output.size() << " descriptors");
  
  SVL_ASSERT(descriptors.size() == points.size());

  // builds a structure of codebook words out of the descriptors
  // that's easily accessible for all sliding window locations
  int w = img->width;
  int h = img->height;
  int **words = new int*[w];
  for (int i = 0; i < w; i++) {
    words[i] = new int[h];
    for (int j = 0; j < h; j++)
      words[i][j] = -1;
  }
  for (unsigned i = 0; i < descriptors.size(); i++) {
    int x = points[i].x;
    int y = points[i].y;
    int word = _codebook.encode(descriptors[i]);
    words[x][y] = word;
  }

  // builds the histograms of codewords to serve as location descriptors
  // for each location
  svlHistogram hist(_codebook.size());
  for (unsigned index = 0; index < locations.size(); index++) {
    hist.set(&output[index], outputOffset);
    int x = locations[index].x;
    int y = locations[index].y;

    // iterates over all positions within the current window in search
    // of interest points and corresponding codewords
    for (int i = x; i < x + _windowSize.width; i++) {
      for (int j = y; j < y + _windowSize.height; j++) {
	if (words[i][j] != -1)
	  hist.insert(words[i][j]);
      }
    }
    hist.normalize();
  }

  // clear the words structure
  for (int i = 0; i < w; i++)
    delete[] words[i];
  delete[] words;
}

bool svlPixelDictionary::writeOut(ofstream &ofs)
{
  if (ofs.fail()) { return false; }

  ofs << "<svlFeatureExtractor id=\"PixelDictionary\"\n"
      << "  version=\"1\"\n"
      << "  width=\"" <<  _windowSize.width << "\"\n"
      << "  height=\"" << _windowSize.height << "\"\n"
      << "  channel=\"" << _validChannel << "\"\n";
  if (_ipDetector) {
    ofs << "  det_thresh=\"" << _ipDetector->getThreshold() << "\"";
  } else {
    ofs << "  num_points=\"" << _numPointsPerSample << "\"";
  }
  ofs << ">\n";
   
  for (unsigned i = 0; i < _descriptors.size(); i++) {
    //ofs << "<descriptor>" << endl;
    _descriptors[i]->writeOut(ofs);
    //ofs << "</descriptor>" << endl;
  }

  if (_codebookTrained) {
    ofs << "<codebook>" << endl;
    if (!_codebook.save(ofs)) { return false; }
    ofs << "</codebook>" << endl;
  }

  ofs << "</svlFeatureExtractor>\n";
  
  return true;
}

bool svlPixelDictionary::load(XMLNode &root)
{
  if (root.isEmpty()) {
    SVL_LOG(SVL_LOG_WARNING, "Pixel dictionary file is missing or has incorrect root.");
    return false;
  }

  if (string(root.getAttribute("id")) != "PixelDictionary" ) {
    SVL_LOG(SVL_LOG_WARNING, "Attempt to read pixel dictionary from XMLNode that is not it.");
    return false;
  }

  clear();

  _windowSize.width = atoi(root.getAttribute("width"));
  _windowSize.height = atoi(root.getAttribute("height"));
  _validChannel = atoi(root.getAttribute("channel"));

  if (root.isAttributeSet("num_points")) {
    _numPointsPerSample = atoi(root.getAttribute("num_points"));
  }

  if (root.isAttributeSet("det_thresh")) {
    _ipDetector = new svlHarrisLaplaceDetector(atof(root.getAttribute("det_thresh")));
  }

  SVL_LOG(SVL_LOG_VERBOSE, root.nChildNode("svlFeatureExtractor") << " descriptors in svlPixelDictionary");

  for (int i = 0; i < root.nChildNode("svlFeatureExtractor"); i++) {
    XMLNode child = root.getChildNode("svlFeatureExtractor", i);
    svlFeatureExtractor *e = svlFeatureExtractorFactory().load(child);
    svlPixelFeatureExtractor *p = dynamic_cast<svlPixelFeatureExtractor *>(e);
    SVL_ASSERT_MSG(p, "invalid pixel feature extractor");
    _descriptors.push_back(p);
  }

  if (root.nChildNode("codebook") > 0) {
    SVL_ASSERT(root.nChildNode("codebook") == 1);
    XMLNode book = root.getChildNode("codebook");
    stringstream ss;
    ss << book.getText();
    if (!_codebook.load(ss))
      return false;

    _codebookTrained = true;
  } 

  return true;
}

unsigned svlPixelDictionary::numFeatures() const {
  if (_codebookTrained)
    return _codebook.size();
  unsigned result = 0;
  for (unsigned i = 0; i < _descriptors.size(); i++)
    result += _descriptors[i]->numFeatures();
  return result;
}

svlFeatureExtractor* svlPixelDictionary::getPrunedExtractor(const vector<bool> &featureUsedBits) const
{
  SVL_LOG(SVL_LOG_FATAL, "not implemented yet");
  return NULL; // to suppress warnings
}

svlFeatureExtractor * svlPixelDictionary::clone() const
{
  return dynamic_cast<svlFeatureExtractor *>(new svlPixelDictionary(*this));
}

void svlPixelDictionary::extractDescriptors(IplImage *img, vector<CvPoint> &points,
					    vector<vector<double> > &descriptors,
					    int *numPoints) const
{
  points.clear();
  descriptors.clear();
  vector<float> scales;

  int n = numPoints ? *numPoints : _numPointsPerSample;

  // extracts the interest points
  if (_ipDetector) {
    _ipDetector->findInterestPointsScaled(img, points, scales);
  } else if (n >= 0) {
    points.reserve(n);
    for (int i = 0; i < n; i++)
      points.push_back(cvPoint(rand() % img->width, rand() % img->height));
  } else {
    points.reserve(img->width * img->height);
    for (int y = 0; y < img->height; y++)
      for (int x = 0; x < img->width; x++)
	points.push_back(cvPoint(x, y));
  }

  // finds all the descriptors 
  unsigned offset = 0;
  for (unsigned i = 0; i < _descriptors.size(); i++) {
    _descriptors[i]->getDescriptors(img, points, descriptors, &scales, offset);
    offset += _descriptors[i]->numFeatures();
  }
}


