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
** FILENAME:    svlObjectDetectionAnalyzer.h
** AUTHOR(S):   Olga Russakovsky <olga@cs.stanford.edu>
**              based on applications by Stephen Gould <sgould@stanford.edu>
**              Ian Goodfellow <ia3n@cs.stanford.edu>
** DESCRIPTION:
**  Provides helper functions for analyzing the detections of the
**  sliding window classifier.
**
*****************************************************************************/

#pragma once

#include <cassert>
#include <iostream>
#include <vector>
#include <map>

#include "cv.h"
#include "cxcore.h"
#include "ml.h"

#include "svlBase.h"
#include "svlObjectList.h"
#include "svlML.h"

using namespace std;

// svlObjectDetectionAnalyzer

static const string NO_NAME = "";

class svlObjectDetectionAnalyzer {
 public:
  static float DEFAULT_AREA_RATIO;
  static bool USE_NONMAX;
  static bool INCLUDE_ALL_FRAMES;
  static int MAX_OBJECTS_PER_FRAME;

 protected:
  // if just interested in one object
  string _objectName;  
  svlPRCurve _objectScores;

  svlObject2dSequence _gtObjectList;
  svlObject2dSequence _objectList;
  bool _useNonmax;
  float _areaRatio;
  bool _includeAllFrames;
  int _maxObjectsPerFrame;

 public:
  svlObjectDetectionAnalyzer(string name = NO_NAME);
    
    // to suppress warnings
  virtual ~svlObjectDetectionAnalyzer();

  // loads the gt file; discards all that don't match 'object_name'
  // if it was set
  virtual bool loadGt(const char *filename);

  // careful with this, make sure names match
  void copyGt(svlObjectDetectionAnalyzer &d) {
      _gtObjectList = d._gtObjectList;
  }

  void setNonmax(bool useNonmax) { _useNonmax = useNonmax; }

  // loads the detection file, discards all that don't match 'object_name'
  // if it was set, and filters non-maximal objects
  virtual bool loadDetections(const char *filename);
  virtual void loadDetections(const svlObject2dSequence & s);

  // outputs after loading detections
  void outputDetections(const char *filename, float threshold = 0);
  svlObject2dSequence getDetections() const { return _objectList; }

  // based on the groundtruth and detections that are currently loaded
  virtual void accumulateScores();

  // will not actually store the detections, but will continuously
  // accumulate objectScores
  bool accumulateScoresForFrame(string frameName, svlObject2dFrame &objects);

  bool evaluateScoresForFrame(string frameName,
			      svlObject2dFrame &detectedObjects,
			      svlPRCurve &_scores) const;

  // returns true if positive detection, false otherwise
  bool evaluateDetection(string frameName, svlObject2d &object);

  // based on the accumulated scores, prints a summary of results
  virtual void printResults(float threshold) const;

  virtual void quantizeScores(double minThreshold = 0.0, double maxThreshold = 1, int numBins = 100) {
    _objectScores.quantize(minThreshold, maxThreshold, numBins);
  }

  // prints all the PR curves currently stored
  virtual void writePRCurves(const char *filestem) const;
  virtual void writeNormalizedPRCurves(const char *filestem) const;

  // loads and save the statistics from/to file
  virtual bool load(const char *filename);
  virtual bool save(const char *filename);

  // gets the highest F-score points on the PR curve (must be already created)
  float computeHighestFscore() const;

  // gets the area of the currently stored PR curve
  float computePRarea() const;

  int numDetectionsStored() const { return _objectScores.numSamples(); }

  /// Writes TP, FP, FN, thresh to a file
  virtual void writeAllInfo(const char * filepath);

  virtual void clear();

  bool writeScores(const char *filename);
  bool addScores(const char *filename);
};


typedef map<string, svlPRCurve> TResponses;

class svlMultObjectsDetectionAnalyzer : public svlObjectDetectionAnalyzer {
 private:
  TResponses _allObjectScores;

 public:
  // to suppress warnings
  ~svlMultObjectsDetectionAnalyzer() {}

  // defining virtual functions from above
  bool loadGt(const char *filename);
  bool loadDetections(const char *filename);
  void accumulateScores();
  void printResults(float threshold) const;
  void writePRCurves(const char *filestem) const;

  // loads and saves a PR curve with a specific name
  bool save(const char *filename); // will return false
  bool save(const char *filename, string name);

  void quantizeScores(double minThreshold = 0.0, double maxThreshold = 1.0, int numBins = 100);

  // gets the highest F-score points on one of the PR curves
  // (must be already created); name doesn't need to be specified
  // iff there's only one curve currently stored
  float computeHighestFscore(string name = NO_NAME);

  // gets the area of one of the currently stored PR curves
  float computePRarea(string name = NO_NAME);


  virtual void writeAllInfo(const char * filepath)
  {
    SVL_LOG(SVL_LOG_ERROR, "ignoring request to write info to " << filepath << ", writeAllInfo is not implemented for svlMultObjectsDetectionAnalyzer");
  }

  void clear();

};
