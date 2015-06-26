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
** FILENAME:    svlTrainingDatasetBuilder.h
** AUTHOR(S):   Olga Russakovsky <olga@cs.stanford.edu>
**              based on applications by
**                     Stephen Gould <sgould@stanford.edu>
**                     Ian Goodfellow <ia3n@cs.stanford.edu>
** DESCRIPTION:
**  Builds the entire training image dataset. The main advantage of
**  this class is to encapsulate the large number of parameters in a
**  clean way. Currently can only write to files; if needed can easily 
**  implement the functionality to return the extracted windows instead.
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
#include "svlImageSequence.h"
#include "svlSlidingWindowDetector.h"
using namespace std;

class svlTrainingDatasetBuilder {
 public:
  // related to all extractions
  static set<string> OBJECTS;
  static double OVERLAP_THRESHOLD;
  static bool ALL_WINDOWS;
  static bool BASE_SCALE_ONLY; 
  static bool TEXTURE;

  // related to negatives
  static bool SKIP_NEG;
  static bool FPOS_ONLY;
  static string DETECTIONS_FILE;
  static double THRESHOLD;
  static bool INCLUDE_OTHER_OBJS;
  static int NUM_NEG;
  static int NEG_HEIGHT;
  static double NEG_ASPECT_RATIO;
  static bool USE_NONMAX;

  // related to positives
  static bool SKIP_POS;
  static bool POS_BEST_WINDOW;

  // related to writer
  static string BASE_DIR;
  static bool CREATE_DIRS;
  static string NEG_SUBDIR_NAME;
  static string POS_PREFIX;
  static string NEG_PREFIX;
  static bool INCLUDE_FLIPPED;
  static int RESIZE_WIDTH;
  static int RESIZE_HEIGHT;
  static bool USE_WIN_REFS;

 public:
  svlTrainingDatasetBuilder(const char *imageSeq, const char *gtLabels);
  ~svlTrainingDatasetBuilder();

  void writeDataset(int parallelCount = 0, int parallelId = 0);

 private:
  // related to all extractions
  bool _bAllWindows;

  // related to negatives
  bool _bSkipNeg;
  bool _bFPonly;
  double _threshold;
  bool _bIncludeOthers;
  unsigned _numNeg;
  int _negHeight;
  double _negAspectRatio;
  bool _bNonMax;

  // related to positives
  bool _bSkipPos;
  bool _bPosBestWindow;

  // write out the window references instead of individual clips
  bool _useWindowReferences;

  // image information
  svlImageSequence _imageSequence;
  svlObject2dSequence _groundTruth;
  svlObject2dSequence _objectDetections;

  // classes that encapsulate some of the functionality
  // in a cleaner way
  class svlImageLoader *_loader;
  class svlImageWindowExtractor *_wExtractor;
  class svlClipWriter *_writer;
};


// svlClipWriter helper class --------------------------------------------------------------

class svlClipWriter { 
public:
  svlClipWriter();

  void storeClips(const svlObject2dFrame &regions, const string &imageId, bool pos);
  void countClips(const svlObject2dFrame &regions, bool pos);

  void createDirectories(const svlObject2dFrame &regions);

  // will not write out "almost uniform" random negative regions
  void writeClips(IplImage *image, const svlObject2dFrame &regions,
		  svlColorType cType, const string &ext,
		  bool positive);

  // will write to _baseDirectory/"imageSeq.${objectName}.xml"
  void writeWindowReferences(const string &dir);

  void clear() { _counts.clear(); _windowReferences.clear(); }
  void printCounts() const;

 private:
  // initialized from svlTrainingDatasetBuilder's static vars
  string _baseDirectory;
  string _negSubdirName;
  bool _createPosDirs; // assumes negDir already created
  
  string _posPrefix;
  string _negPrefix;

  bool _bIncludeFlipped;
  int _resizeWidth;
  int _resizeHeight;

  // map from object names to corresponding lists of windows
  map<string, svlImageRegionsSequence> _windowReferences;
  // map from object names to counts of windows
  map<string, unsigned> _counts;
};



