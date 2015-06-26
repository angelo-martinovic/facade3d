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
** FILENAME:    svlObjectDetectionAnalyzer.cpp
** AUTHOR(S):   Olga Russakovsky <olga@cs.stanford.edu>
**              based on applications by Stephen Gould <sgould@stanford.edu>
**              Ian Goodfellow <ia3n@cs.stanford.edu>
**
*****************************************************************************/

#include <cassert>
#include <iostream>
#include <fstream>
#include <vector>
#include <sys/types.h>
#include <limits>

#include "svlBase.h"
#include "svlVision.h"

using namespace std;

// constants for the configuration manager ---------------------------------------

float svlObjectDetectionAnalyzer::DEFAULT_AREA_RATIO = 0.5;
bool svlObjectDetectionAnalyzer::USE_NONMAX = true;
bool svlObjectDetectionAnalyzer::INCLUDE_ALL_FRAMES = false;
int svlObjectDetectionAnalyzer::MAX_OBJECTS_PER_FRAME = SVL_INT_MAX;

// static function prototypes -----------------------------------------------------

static void filterDetections(svlObject2dSequence &objects, bool filterOverlapping = false,
    bool filterByName = false, string name = NO_NAME);
static bool loadDetectionsFile(const char *filename, svlObject2dSequence &objects,
    bool filterOverlapping = false, bool filterByName = false,
    string name = NO_NAME);
static void updateAccumulatedScores(const svlObject2dFrame &detectedObjects,
    const svlObject2dFrame &gtObjects,
    float areaRatio, svlPRCurve &objectScores);

// svlObjectDetectionAnalyzer class -----------------------------------------------

svlObjectDetectionAnalyzer::svlObjectDetectionAnalyzer(string name) :
    _objectName(name), _useNonmax(USE_NONMAX), _areaRatio(DEFAULT_AREA_RATIO),
    _includeAllFrames(INCLUDE_ALL_FRAMES), _maxObjectsPerFrame(MAX_OBJECTS_PER_FRAME)
{
    // do nothing
}

svlObjectDetectionAnalyzer::~svlObjectDetectionAnalyzer()
{
    // do nothing
}

bool svlObjectDetectionAnalyzer::loadGt(const char *filename)
{
    return loadDetectionsFile(filename, _gtObjectList, false, true, _objectName);
}

bool svlObjectDetectionAnalyzer::loadDetections(const char *filename)
{
    return loadDetectionsFile(filename, _objectList, _useNonmax, true, _objectName);
}

void svlObjectDetectionAnalyzer::loadDetections(const svlObject2dSequence & s)
{
    _objectList = s;
    filterDetections(_objectList,  _useNonmax, true, _objectName);
}

void svlObjectDetectionAnalyzer::outputDetections(const char *filename,
						  float threshold)
{
    svlObject2dSequence outputObjects;

    for (svlObject2dSequence::const_iterator it = _objectList.begin();
         it != _objectList.end(); ++it) {
        for (unsigned k = 0; k < it->second.size(); k++) {
            if (it->second[k].pr >= threshold) {
                outputObjects[it->first].push_back(it->second[k]);
            }
        }
    }

    outputObjects.write(filename);
}

void svlObjectDetectionAnalyzer::accumulateScores()
{
    for (svlObject2dSequence::const_iterator itg = _gtObjectList.begin();
         itg != _gtObjectList.end(); ++itg) {
        const svlObject2dFrame& groundTruthObjects = itg->second;

        svlObject2dSequence::const_iterator itd = _objectList.find(itg->first);
        if (itd == _objectList.end()) {
            if (_includeAllFrames) {
	      // notch up missing for all objects in the frame
	      for (unsigned i = 0; i < groundTruthObjects.size(); i++) {
		if (groundTruthObjects[i].ignore != true) {
		  _objectScores.accumulateMisses();
		}
	      }
            }
            continue;
        }

        svlObject2dFrame detectedObjects = itd->second;
        if (_maxObjectsPerFrame < SVL_INT_MAX) {
            int total = detectedObjects.size();
            int removed = keepMostProbableObjects(detectedObjects, _maxObjectsPerFrame);
            SVL_LOG(SVL_LOG_DEBUG, removed << " out of " << total << " least probable objects");
        }

	updateAccumulatedScores(detectedObjects, groundTruthObjects, _areaRatio, _objectScores);
    }
}

// will not actually store the detections, but will continuously
// accumulate objectScores
bool svlObjectDetectionAnalyzer::accumulateScoresForFrame(string frameName,
							  svlObject2dFrame &detectedObjects)
{
  return evaluateScoresForFrame(frameName, detectedObjects, _objectScores);
}

bool svlObjectDetectionAnalyzer::evaluateScoresForFrame(string frameName,
							svlObject2dFrame &detectedObjects,
							svlPRCurve &_scores) const
{
  const svlObject2dFrame *gtObjects = NULL;
  const svlObject2dFrame noObjects;
  svlObject2dSequence::const_iterator it = _gtObjectList.find(frameName);
  if (it != _gtObjectList.end()) {
    gtObjects = &(it->second);
  } else {
    gtObjects = &(noObjects);
  }

  // filter the detections as you would when loading them from file
  if (_useNonmax) {
    int total = detectedObjects.size();
    int removed = nonMaximalSuppression(detectedObjects, 0.25, 0.25, 0.64);
    SVL_LOG(SVL_LOG_VERBOSE, removed << " out of " << total << " non-maximal objects suppressed");
  }
  
  if (_maxObjectsPerFrame < SVL_INT_MAX) {
    int total = detectedObjects.size();
    int removed = keepMostProbableObjects(detectedObjects, _maxObjectsPerFrame);
    SVL_LOG(SVL_LOG_DEBUG, removed << " out of " << total << " least probable objects");
  }
  
  updateAccumulatedScores(detectedObjects, *gtObjects, _areaRatio, _scores);
  
  return true;
}

bool svlObjectDetectionAnalyzer::evaluateDetection(string frameName, svlObject2d &detection)
{
    svlObject2dFrame &gtObjects = _gtObjectList[frameName];
    for (unsigned i = 0; i < gtObjects.size(); i++) {
        if (gtObjects[i].areaOverlap(detection) >= _areaRatio)
            return true;
    }
    return false;
}

void svlObjectDetectionAnalyzer::printResults(float threshold) const  {
    cout << endl << "OBJECT\tCOUNT\tHIT\tERROR\tRECALL\tPREC.\tF1("
         << threshold << ")" << endl;
    cout << _objectName << "\t" << _objectScores.summaryAt(threshold) << endl;
}


void svlObjectDetectionAnalyzer::writePRCurves(const char *filestem) const
{
    string filename = string(filestem) + string(".") + _objectName + string(".txt");
    _objectScores.writeCurve(filename.c_str());
}

void svlObjectDetectionAnalyzer::writeAllInfo(const char * filepath)
{
  _objectScores.writeAllInfo(filepath);
}

void svlObjectDetectionAnalyzer::writeNormalizedPRCurves(const char *filestem) const
{
  SVL_LOG(SVL_LOG_FATAL, "svlObjectDetectionAnalyzer::writeNormalizedPRCurves is not implemented correctly. It computers false positives per (window with a score above threshold) rather than false positives per window, so it strongly penalizes classifiers that throw out lots of windows");

  string filename = string(filestem) + string(".") + _objectName + string(".txt");
  _objectScores.writeNormalizedCurve(filename.c_str());
}

bool svlObjectDetectionAnalyzer::load(const char *filename)
{
    return _objectScores.read(filename);
}

bool svlObjectDetectionAnalyzer::save(const char *filename)
{
    return _objectScores.write(filename);
}

float svlObjectDetectionAnalyzer::computeHighestFscore() const
{
    return _objectScores.bestF1Score();
}

float svlObjectDetectionAnalyzer::computePRarea() const
{
    return _objectScores.averagePrecision();
}

void svlObjectDetectionAnalyzer::clear()
{
    _objectScores.clear();
}

bool svlObjectDetectionAnalyzer::writeScores(const char *filename)
{
  return _objectScores.write(filename);
}

bool svlObjectDetectionAnalyzer::addScores(const char *filename)
{
  svlClassifierSummaryStatistics stats;
  bool success = stats.read(filename);
  if (!success) return false;

  _objectScores.accumulate(stats);
  return true;
}



// svlMultObjectsDetectionAnalyzer class ------------------------------------------

bool svlMultObjectsDetectionAnalyzer::loadGt(const char *filename)
{
  return loadDetectionsFile(filename, _gtObjectList);
}

bool svlMultObjectsDetectionAnalyzer::loadDetections(const char *filename)
{
  return loadDetectionsFile(filename, _objectList, _useNonmax);
}

void svlMultObjectsDetectionAnalyzer::accumulateScores()
{
  for (svlObject2dSequence::const_iterator itg = _gtObjectList.begin();
         itg != _gtObjectList.end(); ++itg) {

        const svlObject2dFrame& groundTruthObjects = itg->second;

        svlObject2dSequence::const_iterator itd = _objectList.find(itg->first);
        if (itd == _objectList.end()) {
            if (_includeAllFrames) {
                // notch up missing for all objects in the frame
                for (unsigned i = 0; i < groundTruthObjects.size(); i++) {
		  _allObjectScores[groundTruthObjects[i].name].accumulateMisses();
                }
            }
            continue;
        }

        const svlObject2dFrame& detectedObjects = itd->second;
        SVL_LOG(SVL_LOG_DEBUG, "processing frame " << itd->first << ", " << detectedObjects.size() << " detections");

        // list of objects already matched
        vector<bool> alreadyMatched(groundTruthObjects.size(), false);

        // sort detections by score
        vector<pair<float, int> > sortedDetections;
        for (unsigned i = 0; i < detectedObjects.size(); i++) {
            sortedDetections.push_back(make_pair((float)(-detectedObjects[i].pr), i));
        }
        stable_sort(sortedDetections.begin(), sortedDetections.end());

        // find overlapping detections
        for (unsigned i = 0; i < sortedDetections.size(); i++) {
            bool bMatched = false;
            svlObject2d detection = detectedObjects[sortedDetections[i].second];

            for (unsigned j = 0; j < alreadyMatched.size(); j++) {
	      if (alreadyMatched[j])
                    continue;
                if (groundTruthObjects[j].name != detection.name)
                    continue;
                if (groundTruthObjects[j].areaOverlap(detection) < _areaRatio)
                    continue;

                // we have a match
                bMatched = true;
                alreadyMatched[j] = true;
                break;
            }

	    if (bMatched)
	      _allObjectScores[detection.name].accumulatePositives(detection.pr);
	    else
	      _allObjectScores[detection.name].accumulateNegatives(detection.pr);
        }

        // all detections not matched by now are misses
        for (unsigned i = 0; i < alreadyMatched.size(); i++) {
            if (!alreadyMatched[i])
	      _allObjectScores[itg->second[i].name].accumulateMisses();
        }
    }
}

void svlMultObjectsDetectionAnalyzer::printResults(float threshold) const
{
    cout << endl << "OBJECT\tCOUNT\tHIT\tERROR\tRECALL\tPREC.\tF1("
         << threshold << ")" << endl;
    for (TResponses::const_iterator it = _allObjectScores.begin();
         it != _allObjectScores.end(); ++it)
        cout << it->first << "\t" << it->second.summaryAt(threshold) << endl;
}

void svlMultObjectsDetectionAnalyzer::quantizeScores(double minThreshold, double maxThreshold, int numBins)
{
  for (TResponses::iterator it = _allObjectScores.begin();
       it != _allObjectScores.end(); ++it) {
    it->second.quantize(minThreshold, maxThreshold, numBins);
  }
}

void svlMultObjectsDetectionAnalyzer::writePRCurves(const char *filestem) const
{
    for (TResponses::const_iterator it = _allObjectScores.begin();
         it != _allObjectScores.end(); ++it) {
        string filename = string(filestem) + string(".") + it->first + string(".txt");
        it->second.writeCurve(filename.c_str());
    }
}

bool svlMultObjectsDetectionAnalyzer::save(const char *filename)
{
    SVL_LOG(SVL_LOG_WARNING, "Should provide a name for the PR curve");
    return false;
}

bool svlMultObjectsDetectionAnalyzer::save(const char *filename, string name)
{
  if (name == NO_NAME)
    SVL_LOG(SVL_LOG_WARNING, "Should provide a non-empty name for the PR curve");
  _allObjectScores[name].writeCurve(filename);
  return true;
}

float svlMultObjectsDetectionAnalyzer::computeHighestFscore(string name)
{
    // no name is specified, try to guess
    if (name == NO_NAME) {
        if (_allObjectScores.size() == 1) {
            TResponses::iterator single_curve = _allObjectScores.begin();
            return single_curve->second.bestF1Score();
        } else {
            SVL_LOG(SVL_LOG_WARNING, "No object specified, ambiguous");
            return 0;
        }
    }

    // a name is given
    TResponses::iterator curve = _allObjectScores.find(name);
    if (curve != _allObjectScores.end()) {
        return curve->second.bestF1Score();
    } else {
        SVL_LOG(SVL_LOG_WARNING, "No such object exists");
        return 0;
    }
}

float svlMultObjectsDetectionAnalyzer::computePRarea(string name)
{
    // no name is specified, try to guess
    if (name == NO_NAME) {
        if (_allObjectScores.size() == 1) {
            TResponses::iterator curve = _allObjectScores.begin();
            return curve->second.averagePrecision();
        } else {
            SVL_LOG(SVL_LOG_WARNING, "No object specified, ambiguous");
            return 0;
        }
    }

    // a name is given
    TResponses::iterator curve = _allObjectScores.find(name);
    if (curve != _allObjectScores.end()) {
        return curve->second.averagePrecision();
    } else {
        SVL_LOG(SVL_LOG_WARNING, "No such object exists");
        return 0;
    }
}

void svlMultObjectsDetectionAnalyzer::clear()
{
    _allObjectScores.clear();
}


// static functions -----------------------------------------------------------------

static void filterDetections(  svlObject2dSequence &objects, bool filterOverlapping,
			       bool filterByName, string name)
{
    if (filterByName && name != NO_NAME) {
        for (svlObject2dSequence::iterator it = objects.begin() ;
             it != objects.end(); ++it)
            removeNonMatchingObjects(it->second, name.c_str());
    }

    SVL_LOG(SVL_LOG_VERBOSE, objects.size() << " frames read");

    if (filterOverlapping) {
        int total = countObjects(objects);
        int removed = nonMaximalSuppression(objects, 0.25, 0.25, 0.64);
        SVL_LOG(SVL_LOG_VERBOSE, removed << " out of " << total << " non-maximal objects suppressed");
    }
}

static bool loadDetectionsFile(const char *filename, svlObject2dSequence &objects,
			       bool filterOverlapping, bool filterByName, string name)
{
    if (!objects.read(filename)) {
        SVL_LOG(SVL_LOG_ERROR, "...failed to read " << filename);
        return false;
    }

    filterDetections(objects, filterOverlapping, filterByName, name);

    return true;
}

static void updateAccumulatedScores(const svlObject2dFrame &detectedObjects,
    const svlObject2dFrame &gtObjects, float areaRatio, svlPRCurve &objectScores)
{
    // list of objects already matched
    vector<bool> alreadyMatched(gtObjects.size(), false);

    // sort detections by score
    vector<pair<float, int> > sortedDetections(detectedObjects.size());
    for (unsigned i = 0; i < detectedObjects.size(); i++) {
        sortedDetections[i] = make_pair((float)(-detectedObjects[i].pr), i);
    }
    stable_sort(sortedDetections.begin(), sortedDetections.end());
    // assign detections to groundtruth objects
    for (unsigned i = 0; i < sortedDetections.size(); i++) {
        const svlObject2d &detection = detectedObjects[sortedDetections[i].second];

        double bestOverlap = areaRatio;
        int bestIndex = -1;
        for (unsigned j = 0; j < alreadyMatched.size(); j++) {
            if (alreadyMatched[j]) {
                continue;
            }

            double areaOverlap = gtObjects[j].areaOverlap(detection);
            if (areaOverlap > bestOverlap) {
                bestOverlap = areaOverlap;
                bestIndex = j;
            }
        }

        if (bestIndex != -1) {
            // we have a match
            if (!gtObjects[bestIndex].ignore) {
                objectScores.accumulatePositives(detection.pr);
            }
            alreadyMatched[bestIndex] = true;
        } else {
            objectScores.accumulateNegatives(detection.pr);
        }
    }

    // all ground truth objects not matched by now are misses
    for (unsigned i = 0; i < alreadyMatched.size(); i++) {
        if (!alreadyMatched[i] && ! gtObjects[i].ignore)
            objectScores.accumulateMisses();
    }
}

// configuration manager -------------------------------------------------------------

class svlObjectDetectionAnalyzerConfig : public svlConfigurableModule {
public:
    svlObjectDetectionAnalyzerConfig() : svlConfigurableModule("svlVision.svlObjectDetectionAnalyzer") {}

    void usage(ostream &os) const {
        os << "      areaRatio    :: amount of overlap to be considered a positive (default: "
           << svlObjectDetectionAnalyzer::DEFAULT_AREA_RATIO << ")\n"
           << "      useNonmax    :: use non-maximal suppression (default: "
           << (svlObjectDetectionAnalyzer::USE_NONMAX ? "true" : "false") << ")\n"
           << "      includeAllFrames :: include all groundtruth frames (default: "
           << (svlObjectDetectionAnalyzer::INCLUDE_ALL_FRAMES ? "true" : "false") << ")\n"
           << "      maxPerImage  :: maximum detections per image (default: "
           << svlObjectDetectionAnalyzer::MAX_OBJECTS_PER_FRAME << ")\n";
    }

    void setConfiguration(const char *name, const char *value) {
        if (!strcasecmp(name, "areaRatio")) {
            svlObjectDetectionAnalyzer::DEFAULT_AREA_RATIO = atof(value);
        } else if (!strcasecmp(name, "useNonmax")) {
            svlObjectDetectionAnalyzer::USE_NONMAX =
                (!strcasecmp(value, "TRUE") || !strcmp(value, "1"));
        } else if (!strcasecmp(name, "includeAllFrames")) {
            svlObjectDetectionAnalyzer::INCLUDE_ALL_FRAMES =
                (!strcasecmp(value, "TRUE") || !strcmp(value, "1"));
        } else if (!strcasecmp(name, "maxPerImage")) {
            svlObjectDetectionAnalyzer::MAX_OBJECTS_PER_FRAME = atoi(value);
        } else {
            SVL_LOG(SVL_LOG_FATAL, "unrecognized configuration option for " << this->name());
        }
    }
};

static svlObjectDetectionAnalyzerConfig gObjectDetectionAnalyzerConfig;












