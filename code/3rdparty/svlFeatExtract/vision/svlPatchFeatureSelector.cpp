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
** FILENAME:    svlPatchFeatureSelector.cpp
** AUTHOR(S):   Olga Russakovsky <olga@cs.stanford.edu>
**
*****************************************************************************/

#include <cassert>
#include <iostream>
#include <fstream>
#include <vector>
#include <sys/types.h>
#include <limits>

#include "svlPatchFeatureSelector.h"

#if defined(_WIN32)||defined(WIN32)||defined(__WIN32__)
#define _CRT_SECURE_NO_DEPRECATE
#include "win32/dirent.h"
#else
#include <dirent.h>
#endif

using namespace std;

double svlPatchFeatureSelector::F_SCORE_THRESHOLD = 0.75;
double svlPatchFeatureSelector::CCORR_THRESHOLD = 0.9;

// threaded function

#define TOTAL_THREADS 8

struct computeFeatureFScoreArgs {
  computeFeatureFScoreArgs(double &Fscore,
			   const vector<vector<double> > &posSamples,
			   const vector<vector<double> > &negSamples,
			   int featureIndex) :
    _Fscore(&Fscore), _posSamples(&posSamples), 
    _negSamples(&negSamples), _featureIndex(featureIndex) {}

  double *_Fscore;
  const vector<vector<double> > *_posSamples;
  const vector<vector<double> > *_negSamples;
  int _featureIndex;
};

void *computeFeatureFScore(void *voidArgs, unsigned tid) {
  computeFeatureFScoreArgs *args = (computeFeatureFScoreArgs *) voidArgs;
 
  double &Fscore = *(args->_Fscore);
  const vector<vector<double> > &posSamples = *(args->_posSamples);
  const vector<vector<double> > &negSamples = *(args->_negSamples);
  const int i = args->_featureIndex; 

  svlPRCurve curve;
  for (unsigned j = 0; j < posSamples.size(); j++)
    curve.accumulatePositives(posSamples[j][i]);
  for (unsigned j = 0; j < negSamples.size(); j++)
    curve.accumulateNegatives(negSamples[j][i]);
  curve.normalize();

  Fscore = curve.bestF1Score();

  delete args;
  return NULL;
}

static bool patch_scores_sort_decreasing(pair<float, int> i, pair<float, int> j) {
  return i.first > j.first;
}

bool svlPatchFeatureSelector::cacheExamples(const vector<vector<double> > &posSamples,
					    const vector<vector<double> > &negSamples)
{
  if (_dict->numFeatures() != _numFeatures) {
    SVL_LOG(SVL_LOG_WARNING, "Number of features in training examples not the same as in dictionary");
    return false;
  }

  _patchFScores.clear();
  _patchFScores.resize(_numFeatures);
  for (unsigned i = 0; i < _numFeatures; i++)
    _patchFScores[i].second = i;

  // analyze each of the dictionary features in turn
  // to see how good it is at discriminating between +/- examples
  // (note: the +/- examples are weighted s.t. each set contributed 1/2 the weight
  // even though the number of each might be different)  
  svlThreadPool threadPool(TOTAL_THREADS);
  threadPool.start();    
  for (unsigned i = 0; i < _numFeatures; i++) {
    threadPool.addJob(computeFeatureFScore,
		      new computeFeatureFScoreArgs(_patchFScores[i].first,
						   posSamples, negSamples, i));				      
  }
  threadPool.finish();

  stable_sort(_patchFScores.begin(), _patchFScores.end(), patch_scores_sort_decreasing);

  return true;
}




bool svlPatchFeatureSelector::chooseFeatures(vector<bool> &selectedFeatures)
{
  vector<bool> availableForSelection(_numFeatures, true);

  // iterate over patches in order of decreasing Fscores
  for (unsigned i = 0; i < _numFeatures; i++) {
    if (_numSelectedFeatures >= MAX_SIZE) break;
    if (_patchFScores[i].first < F_SCORE_THRESHOLD && _numSelectedFeatures >= MIN_SIZE) break;

    if (!availableForSelection[i]) continue;

    SVL_LOG(SVL_LOG_VERBOSE, "Analyzing patch " << i << " of " << _numFeatures
	    << ", with value " << _patchFScores[i].first);
    int index1 = _patchFScores[i].second;
    selectedFeatures[index1] = true;
    _numSelectedFeatures++;
	
    int num = 0;
    for (unsigned j = i+1; j < _numFeatures; j++) {
      if (!availableForSelection[j]) continue;

      int index2 = _patchFScores[j].second;
      if (_dict->entriesTooCorrelated(index1, index2, CCORR_THRESHOLD)) {
	availableForSelection[j] = false;
	num++;
      }
    }
    SVL_LOG(SVL_LOG_VERBOSE, "Deleted " << num << " patches as a result");
  }

  return true;
}


// configuration --------------------------------------------------------

class svlPatchFeatureSelectorConfig : public svlConfigurableModule {
public:
    svlPatchFeatureSelectorConfig() : svlConfigurableModule("svlML.svlPatchFeatureSelector") { }

    void usage(ostream &os) const {
      os << "      fThreshold   :: threshold for F-score of patches (default: " << svlPatchFeatureSelector::F_SCORE_THRESHOLD << ")\n"
	 << "      ccThreshold  :: threshold for cross-correlation of patches (default: \"" << svlPatchFeatureSelector::CCORR_THRESHOLD << ")\n";
    }

    void setConfiguration(const char *name, const char *value) {
        if (!strcmp(name, "fThreshold")) {
	  svlPatchFeatureSelector::F_SCORE_THRESHOLD = atof(value);
        } else if (!strcmp(name, "ccThreshold")) {
            svlPatchFeatureSelector::CCORR_THRESHOLD = atof(value);
        } else {
            SVL_LOG(SVL_LOG_FATAL, "unrecognized configuration option for " << this->name());
        }
    }
};

static svlPatchFeatureSelectorConfig gPatchFeatureSelectorConfig;



