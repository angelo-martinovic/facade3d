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
** FILENAME:    svlTrainingDatasetBuilder.cpp
** AUTHOR(S):   Olga Russakovsky <olga@cs.stanford.edu>
**              based on applications by
**                     Stephen Gould <sgould@stanford.edu>
**                     Ian Goodfellow <ia3n@cs.stanford.edu>
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
#include "svlTrainingDatasetBuilder.h"

#if defined(_WIN32)||defined(WIN32)||defined(__WIN32__)
#define _CRT_SECURE_NO_DEPRECATE
#include "win32/dirent.h"
#else
#include <dirent.h>
#endif

using namespace std;

// related to all extractions
set<string> svlTrainingDatasetBuilder::OBJECTS;
double svlTrainingDatasetBuilder::OVERLAP_THRESHOLD = 0.5;
bool svlTrainingDatasetBuilder::ALL_WINDOWS = false;
bool svlTrainingDatasetBuilder::BASE_SCALE_ONLY = false;
bool svlTrainingDatasetBuilder::TEXTURE = false;

// related to negatives
bool svlTrainingDatasetBuilder::SKIP_NEG = false;
bool svlTrainingDatasetBuilder::FPOS_ONLY = false;
string svlTrainingDatasetBuilder::DETECTIONS_FILE = string("");
double svlTrainingDatasetBuilder::THRESHOLD = 0.0;
bool svlTrainingDatasetBuilder::INCLUDE_OTHER_OBJS = false;
int svlTrainingDatasetBuilder::NUM_NEG = 100;
int svlTrainingDatasetBuilder::NEG_HEIGHT = -1;
double svlTrainingDatasetBuilder::NEG_ASPECT_RATIO = -1;
bool svlTrainingDatasetBuilder::USE_NONMAX = false;

// related to positives
bool svlTrainingDatasetBuilder::SKIP_POS = false;
bool svlTrainingDatasetBuilder::POS_BEST_WINDOW = false;

// related to writer
string svlTrainingDatasetBuilder::BASE_DIR = string(".");
bool svlTrainingDatasetBuilder::CREATE_DIRS = false;
string svlTrainingDatasetBuilder::NEG_SUBDIR_NAME = string("negative");
string svlTrainingDatasetBuilder::POS_PREFIX = string("");
string svlTrainingDatasetBuilder::NEG_PREFIX = string("");
bool svlTrainingDatasetBuilder::INCLUDE_FLIPPED = false;
int svlTrainingDatasetBuilder::RESIZE_WIDTH = -1;
int svlTrainingDatasetBuilder::RESIZE_HEIGHT = -1;
bool svlTrainingDatasetBuilder::USE_WIN_REFS = false;


svlTrainingDatasetBuilder::svlTrainingDatasetBuilder(const char *imageSeqFilename,
						     const char *gtFilename) :
  _bAllWindows(ALL_WINDOWS), _bSkipNeg(SKIP_NEG), _bFPonly(FPOS_ONLY),
  _threshold(THRESHOLD), _bIncludeOthers(INCLUDE_OTHER_OBJS),
  _numNeg(NUM_NEG), _negHeight(NEG_HEIGHT), _negAspectRatio(NEG_ASPECT_RATIO), _bNonMax(USE_NONMAX),
  _bSkipPos(SKIP_POS), _bPosBestWindow(POS_BEST_WINDOW), _useWindowReferences(USE_WIN_REFS)
{
  // loads the image filenames
  SVL_LOG(SVL_LOG_VERBOSE, "Reading image sequence from " << imageSeqFilename << "...");
  _imageSequence.load(imageSeqFilename);
  SVL_LOG(SVL_LOG_VERBOSE, "..." << _imageSequence.size() << " images read");

  if (!hasHomogeneousExtensions(_imageSequence)) {
    SVL_LOG(SVL_LOG_FATAL, "Sequence file " << imageSeqFilename
	    << " has multiple file extensions, so no standardized swapping "
	    << "of file extensions will work for managing channels.");
  }

  // load the object labels
  if (gtFilename) {
    SVL_LOG(SVL_LOG_VERBOSE, "Reading object labels from " << gtFilename << "...");
    _groundTruth.read(gtFilename);
    SVL_LOG(SVL_LOG_VERBOSE, "..." << _groundTruth.size() << " labeled frames read");
  }

  // loads the detections, if any
  if (DETECTIONS_FILE.length() > 0) {
    SVL_LOG(SVL_LOG_VERBOSE, "Reading detections from " << DETECTIONS_FILE << "...");
    _objectDetections.read(DETECTIONS_FILE.c_str());
    SVL_LOG(SVL_LOG_VERBOSE, "..." << _objectDetections.size() << " labeled frames read");
    int total = countObjects(_objectDetections);
    if (_bNonMax) {
        int removed = nonMaximalSuppression(_objectDetections, 0.25, 0.25, 0.64);
        SVL_LOG(SVL_LOG_VERBOSE, "..." << removed << " out of " << total << " non-maximal objects suppressed");
    } else {
        SVL_LOG(SVL_LOG_VERBOSE, "..." << total << " objects");
    }
  }  

  // create the directory for negative objects
  if (CREATE_DIRS && !_bSkipNeg) {
    string dirName = string(BASE_DIR) + string("/") + string(NEG_SUBDIR_NAME);
    svlCreateDirectory(dirName.c_str());
  }
  
  // loads the base extension for file reader from the image sequence
  svlImageLoader::BASE_EXT = getExtension(_imageSequence);

  // _writer by default initialized with relevant parameters
  _writer = new svlClipWriter();
  _loader = new svlImageLoader(false); // no edge channel
  _wExtractor = new svlImageWindowExtractor(OBJECTS, OVERLAP_THRESHOLD, TEXTURE,
					    RESIZE_WIDTH, RESIZE_HEIGHT, BASE_SCALE_ONLY);

  if (_negAspectRatio == -1 && RESIZE_WIDTH > 0 && RESIZE_HEIGHT > 0)
    _negAspectRatio = (float)RESIZE_WIDTH / RESIZE_HEIGHT;
}

svlTrainingDatasetBuilder::~svlTrainingDatasetBuilder()
{
  delete _loader;
  delete _wExtractor;
  delete _writer;
}

void svlTrainingDatasetBuilder::writeDataset(int parallelCount, int parallelId)
{
  SVL_ASSERT(_loader->numChannels() > 0);

  for (unsigned chNum = 0; chNum < _loader->numChannels(); chNum++) {
    if (_loader->channelTypes(chNum) == SVL_CHANNEL_EDGE) {
      SVL_LOG(SVL_LOG_WARNING, "trainingDatasetBuilder does not export edge patches. "
	      << "Just make them on the fly in later apps.");
      continue;
    }

    _loader->printChannelInfo(chNum);
  }

  _writer->clear();

  // will always use the first channel to generate windows and if it
  // happens to be an intensity channel will also check for uniformity
  // of patches to make a more reasonable dataset
  bool check_for_uniformity = (_loader->channelTypes(0) == SVL_CHANNEL_INTENSITY);

  // analyze images and extract regions
 for (unsigned i = 0; i < _imageSequence.size(); i++) {
    if (parallelCount > 0) {
      if ((i % unsigned(parallelCount)) != unsigned(parallelId)) continue ;
    }

    SVL_LOG(SVL_LOG_VERBOSE, "Processing image " << _imageSequence[i] << "...");

    string filename = _imageSequence.directory() + string("/") + _imageSequence[i];

    // load the image from the 0^th channel
    IplImage *image = _loader->getSimpleImageFromFilename(filename.c_str(), 0);
    if (!image) 
      continue;
     
    //look up the positive examples for this frame
    string imageId = strBaseName(_imageSequence[i]);
    if (_groundTruth.find(imageId) != _groundTruth.end()) 
      _wExtractor->setGroundTruthLabels(_groundTruth[imageId]);
    else
      _wExtractor->clearGroundTruthLabels();
    
    _wExtractor->setImageSize(image->width, image->height);
    
    svlObject2dFrame negWindows;
    svlObject2dFrame posWindows;
    
    if (_bAllWindows) {
      // includes all windows
      
      // avoids having to fill in the vector with a potentially very
      // long list of negatives unless necessary
      if (_bSkipPos) {
	_wExtractor->getAllNegWindows(negWindows);
      } else if (_bSkipNeg) {
	_wExtractor->getAllPosWindows(posWindows);
      } else {
	_wExtractor->getAllWindows(posWindows, negWindows);
      }
    }
    
    else {
      // not all windows should be included
      
      if (!_bSkipPos) {
	// finds positive windows
	if (_bPosBestWindow)
	  _wExtractor->getBestPositiveWindows(posWindows);
	else
	  _wExtractor->getPositives(posWindows);
      }
      if (!_bSkipNeg) {
	// finds negative windows
	
	// (1) false positive detections, if available
	SVL_LOG(SVL_LOG_VERBOSE, "adding false positives (numLeft = " 
                << (_numNeg - negWindows.size()) << ")...");
	if (_objectDetections.find(imageId) != _objectDetections.end())
	  _wExtractor->getFalsePositives(negWindows, _objectDetections[imageId], 
					 _threshold, _numNeg);
	
	// (2) if any room still, get other objects if necessary
	int numLeft = _numNeg - negWindows.size();
	if (!_bFPonly && _bIncludeOthers && numLeft > 0) {
	  SVL_LOG(SVL_LOG_VERBOSE, "adding other objects (numLeft = "  << numLeft << ")...");
	  _wExtractor->getOtherObjects(negWindows, numLeft);
	}
	
	// (3) if any room still, extract random detections
	numLeft = _numNeg - negWindows.size();
	if (!_bFPonly && numLeft > 0) {
	  SVL_LOG(SVL_LOG_VERBOSE, "adding random detections (numLeft = "  << numLeft << ")...");
	  // hack for now: will oversample, so that when some are discarded based on being uniform
	  // or not interesting enough, the correct number will still remain with high probability
	  _wExtractor->getRandomNegatives(negWindows,
					  check_for_uniformity ? numLeft * 3 : numLeft,
					  _negAspectRatio, _negHeight);

	  // check for uniformity of random extractions up to the number of negatives
	  // that will actually be written
	  if (check_for_uniformity) {
	    int start = max(0, (int)negWindows.size() - 3*numLeft);
	    for (int i = start; i < (int)negWindows.size() && i < (int)_numNeg; i++) {
	      svlObject2d window = negWindows[i];
	      cvSetImageROI(image, window.getRect());
	      if (svlImageAlmostUniform(image)) {
		negWindows.erase(negWindows.begin() + i);
		i--;
	      }
	    }
	  }
	}

	// erase all the negatives past the number you want to write out
	if (negWindows.size() > _numNeg)
	  negWindows.erase(negWindows.begin() + _numNeg, negWindows.end());	

	SVL_LOG(SVL_LOG_VERBOSE, "... finished adding negatives (numLeft = "  
		<< ((int)_numNeg - negWindows.size()) << ")...");
      }
    }
    
    if (_useWindowReferences) {
      // store references and don't actually create the files
      if (posWindows.size() > 0) 
	_writer->storeClips(posWindows, _imageSequence[i], true);
      if (negWindows.size() > 0)
	_writer->storeClips(negWindows, _imageSequence[i], false);
      cvReleaseImage(&image);
    } else {
      // create the necessary directories (unless already exist; negative was already created)
      if (posWindows.size() > 0)
	_writer->createDirectories(posWindows);
      
      // now iterate through all the channels and load the images individually and
      // write them out
      for (unsigned chNum = 0; chNum < _loader->numChannels(); chNum++) {
	
	if (_loader->channelTypes(chNum) == SVL_CHANNEL_EDGE) continue;
	
	svlColorType chColorType = _loader->colorType(chNum);
	string chExt = _loader->extensions(chNum);
	
	if (chNum > 0) {
	  image = _loader->getSimpleImageFromFilename(filename.c_str(), chNum);
	  if (!image) continue;
	}
	
	if (posWindows.size() > 0)
	  _writer->writeClips(image, posWindows, chColorType, chExt, true);
	if (negWindows.size() > 0)
	  _writer->writeClips(image, negWindows, chColorType, chExt, false);
	
	cvReleaseImage(&image);
      } //end iterating through channels 
    }

    // finally, count the number of clips
    if (posWindows.size() > 0) 
      _writer->countClips(posWindows, true);
    if (negWindows.size() > 0)
      _writer->countClips(negWindows, false);

  } //end iterating through images
  
  _writer->printCounts();
  
  if (_useWindowReferences)
    _writer->writeWindowReferences(_imageSequence.directory());
}








// svlClipWriterClass implementation -------------------------------------------------------------------------------------

svlClipWriter::svlClipWriter() :
  _baseDirectory(svlTrainingDatasetBuilder::BASE_DIR), 
  _negSubdirName(svlTrainingDatasetBuilder::NEG_SUBDIR_NAME),
  _createPosDirs(svlTrainingDatasetBuilder::CREATE_DIRS), 
  _posPrefix(svlTrainingDatasetBuilder::POS_PREFIX),
  _negPrefix(svlTrainingDatasetBuilder::NEG_PREFIX), 
  _bIncludeFlipped(svlTrainingDatasetBuilder::INCLUDE_FLIPPED),
  _resizeWidth(svlTrainingDatasetBuilder::RESIZE_WIDTH), 
  _resizeHeight(svlTrainingDatasetBuilder::RESIZE_HEIGHT)
{
  // do nothing
}

static IplImage *createPatch(CvRect rect, svlColorType type)
{
  switch (type) {
  case SVL_COLOR_GRAY:
    return cvCreateImage(cvSize(rect.width, rect.height), IPL_DEPTH_8U, 1);
  case SVL_COLOR_UNDEFINED:
    return cvCreateImage(cvSize(rect.width, rect.height), IPL_DEPTH_32F, 1);
  default:
    return cvCreateImage(cvSize(rect.width, rect.height), IPL_DEPTH_8U, 3);
  }
}

static void savePatch(const string & filename, const IplImage * patch, svlColorType type)
{
  SVL_LOG(SVL_LOG_DEBUG, "saving patch to " << filename);
  switch(type)
    {
    case SVL_COLOR_UNDEFINED:
      writeMatrixAsIplImage(patch, filename.c_str());
      break;
    default:
      cvSaveImage(filename.c_str(), patch);
    }
}

void svlClipWriter::storeClips(const svlObject2dFrame &objects, const string &imageId,
			       bool positive)
{
  if (!positive) {
    svlImageRegionsSequence &seq = _windowReferences[_negSubdirName];
    seq.push_back(imageId, objects);
  } else {
    for (unsigned i = 0; i < objects.size(); i++) {
      svlObject2d object = objects[i];
      svlImageRegionsSequence &seq = _windowReferences[object.name];
      if (seq.numImages() > 0 && seq.back() == imageId) {
	seq.push_back(seq.numImages() - 1, object.getRect());
      } else {
	seq.push_back(imageId, object.getRect());
      }
    }
  }
}

void svlClipWriter::countClips(const svlObject2dFrame &objects, bool positive)
{
  if (!positive) {
    if (_counts.find(_negSubdirName) == _counts.end())
      _counts[_negSubdirName] = objects.size();
    else
      _counts[_negSubdirName] += objects.size();
  } else {
    for (unsigned i = 0; i < objects.size(); i++) {
      string name = objects[i].name;
      if (_counts.find(name) == _counts.end())
	_counts[name] = 1;
      else
	_counts[name]++;
    }
  }
}

void svlClipWriter::createDirectories(const svlObject2dFrame &objects)
{
  for (unsigned i = 0; i < objects.size(); i++) {
    string name = objects[i].name;
    if (_counts.find(name) == _counts.end()) {
      if (_createPosDirs) {
	string dirName = string(_baseDirectory) + string("/") + name;
	svlCreateDirectory(dirName.c_str());
      }
      _counts[name] = 0; // to make sure we remember later
    }     
  }
}

void svlClipWriter::writeClips(IplImage *image, const svlObject2dFrame &objects,
			       const svlColorType cType, const string &ext,
			       bool positive)
{
  // need to know what to name each clip
  map<string, unsigned> counts = _counts;
  map<string, unsigned>::iterator it;

  // small optimization
  if (!positive) {
    it = counts.find(_negSubdirName);
    if (it == counts.end()) {
      counts[_negSubdirName] = 0;
      it = counts.find(_negSubdirName);
    }
  }

  // iterate over each window and print it out
  for (unsigned i = 0; i < objects.size(); i++) {
    svlObject2d object = objects[i];

    cvSetImageROI(image, object.getRect());
    CvRect rect = cvGetImageROI(image);

    //patchtype specific format for image patch to clip out
    IplImage * clip = createPatch(rect, cType);

    assert(clip);
    cvCopy(image, clip);
    cvResetImageROI(image);
  
    if (_resizeWidth > 0 && _resizeHeight > 0)
      resizeInPlace(&clip, _resizeHeight, _resizeWidth);
    
    // extract current index
    unsigned idx = 0;
    if (positive) {
      it = counts.find(object.name);
      SVL_ASSERT(it != counts.end()); // this would mean directory never got created
      idx = it->second++;
    } else {
      idx = it->second++;
    }
    
    // creates the filename
    string baseFilename = _baseDirectory + string("/");
    if (positive) {
      baseFilename += object.name + string("/") + _posPrefix + object.name;
    } else {
      // Note: can use object.name for negatives as well, to know where
      // the clip came from originally (it could've been a false positive
      // detection, or random, or a groundtruth labeling of an object
      // we're not currently considering)
      baseFilename += _negSubdirName + string("/") + _negPrefix + string("image");
    }
    
    baseFilename += padString(toString(idx), 5);
    
    string filename = baseFilename + ext;
    
    savePatch(filename, clip, cType);
    
    if(_bIncludeFlipped) {
      cvFlip(clip, NULL, 1);
      string flippedFilename = baseFilename + "f" + ext;
      savePatch(flippedFilename.c_str(), clip, cType);
    }
    
    cvReleaseImage(&clip);
  }
}

void svlClipWriter::writeWindowReferences(const string &dir)
{
  map<string, svlImageRegionsSequence>::iterator it;
  for (it = _windowReferences.begin(); it != _windowReferences.end(); ++it)  {
    string filename;
    if (_baseDirectory != "")
      filename += string(_baseDirectory) + string("/");
    filename += string("imageSeq.") + it->first + string(".xml");
   cerr << "Writing to file " << filename << endl;
    it->second.setDirectory(dir);
    it->second.save(filename.c_str());
  }
}

void svlClipWriter::printCounts() const
{
  SVL_LOG(SVL_LOG_MESSAGE, "Generated dataset with:");
  map<string, unsigned>::const_iterator it;
  for (it = _counts.begin(); it != _counts.end(); ++it)  {
    SVL_LOG(SVL_LOG_MESSAGE, it->second << " instances of " << it->first);
  }
}






// Configuration class ----------------------------------------------------------------------------------

class svlTrainingDatasetBuilderConfig : public svlConfigurableModule {
public:
  svlTrainingDatasetBuilderConfig() : svlConfigurableModule("svlVision.svlTrainingDatasetBuilder") {}

    void usage(ostream &os) const {
        os << "      objects      :: space-separated object names\n"
           << "      allWindows   :: returns all sliding windows as examples\n"
           << "      skipPos      :: skip positive examples\n"
           << "      posBestWindow:: returns best sliding-window position for positives\n"
           << "      skipNeg      :: skip negative examples\n"
           << "      falsePositivesOnly :: only include false positive detections\n"
           << "      detectionsFilename :: file with object detections\n"
           << "      threshold    :: detection threshold\n"
           << "      includeOtherObjects :: include other objects as negatives\n"
           << "      numNegs      :: total number of negative examples per scene\n"
           << "      negHeight    :: desired height of negative examples\n"
           << "      negAspectRatio :: desired aspect ratio (w/h)\n"
           << "      baseDir      :: base directory for writing\n"
           << "      useWinRefs   :: write the example references to XML\n"
           << "      createDirs   :: create output directories if not present\n"
           << "      negSubdirName:: name of the subdirectory containing negative examples\n"
           << "      posPrefix    :: prefix for final name of positive images\n"
           << "      negPrefix    :: prefix for final name of negative images\n"
           << "      includeFlipped :: include horizontally flipped examples\n"
           << "      resizeWidth  :: resize examples before writing\n"
           << "      resizeHeight :: resize examples before writing\n";
    }

  void readConfiguration(XMLNode& node) {
    // process everything except the objects
    svlConfigurableModule::readConfiguration(node);

    // parse <object name=""/> (append name to existing objects)
    for (int i = 0; i < node.nChildNode("object"); i++) {
        XMLNode child = node.getChildNode("object", i);
        const char *object = child.getAttribute("name");
        SVL_ASSERT(object != NULL);
	svlTrainingDatasetBuilder::OBJECTS.insert(string(object));
    }
  }

  void setConfiguration(const char *name, const char *value) {
    // just for simplicity, used only if the name corresponds to a boolean
    // variable
    bool valueTrue = (!strcasecmp(value, "TRUE") || !strcmp(value, "1"));

    // related to all extractions
    if (!strcmp(name, "objects")) {
      svlTrainingDatasetBuilder::OBJECTS.clear(); // clear so command line can override config file
      string value_copy(value);
      const char *ptr = strtok((char *)value_copy.c_str(), " ");
      while (ptr != NULL) {
	svlTrainingDatasetBuilder::OBJECTS.insert(string(ptr));
	ptr = strtok(NULL, " ");
      }
    } else if (!strcmp(name, "overlapThreshold")) {
      svlTrainingDatasetBuilder::OVERLAP_THRESHOLD = atof(value);
    } else if (!strcmp(name, "allWindows")) {
      svlTrainingDatasetBuilder::ALL_WINDOWS = valueTrue;
      SVL_ASSERT_MSG(!svlTrainingDatasetBuilder::POS_BEST_WINDOW,
		     "allWindows incompatible with posBestWindow");
    } else if (!strcmp(name, "baseScaleOnly")) {
      svlTrainingDatasetBuilder::BASE_SCALE_ONLY = valueTrue;
    } else if (!strcmp(name, "texture")) {
      svlTrainingDatasetBuilder::TEXTURE = valueTrue;
      
      //related to negatives
    } else if (!strcmp(name, "skipNeg")) {
        svlTrainingDatasetBuilder::SKIP_NEG = valueTrue;
    } else if (!strcmp(name, "falsePositivesOnly")) {
        svlTrainingDatasetBuilder::FPOS_ONLY = valueTrue;
    } else if (!strcmp(name, "detectionsFilename")) {
        svlTrainingDatasetBuilder::DETECTIONS_FILE = string(value);
    } else if (!strcmp(name, "threshold")) {
        svlTrainingDatasetBuilder::THRESHOLD = atof(value);
    } else if (!strcmp(name, "includeOtherObjects")) {
        svlTrainingDatasetBuilder::INCLUDE_OTHER_OBJS = valueTrue;
    } else if (!strcmp(name, "numNegs")) {
        svlTrainingDatasetBuilder::NUM_NEG = atoi(value);
    } else if (!strcmp(name, "negHeight")) {
        svlTrainingDatasetBuilder::NEG_HEIGHT = atoi(value);
    } else if (!strcmp(name, "negAspectRatio")) {
        svlTrainingDatasetBuilder::NEG_ASPECT_RATIO = atof(value);
    } else if (!strcmp(name, "useNonMax")) {
        svlTrainingDatasetBuilder::USE_NONMAX = true;
        
      // related to positives
    } else if (!strcmp(name, "skipPos")) {
      svlTrainingDatasetBuilder::SKIP_POS = valueTrue;
    } else if (!strcmp(name, "posBestWindow")) {
      svlTrainingDatasetBuilder::POS_BEST_WINDOW = valueTrue;
      SVL_ASSERT_MSG(!svlTrainingDatasetBuilder::ALL_WINDOWS,
		     "allWindows incompatible with posBestWindow");

      // related to writer
    } else if (!strcmp(name, "baseDir")) {
      svlTrainingDatasetBuilder::BASE_DIR = string(value);
    } else if (!strcmp(name, "createDirs")) {
      svlTrainingDatasetBuilder::CREATE_DIRS = valueTrue;
    } else if (!strcmp(name, "negSubdirName")) {
      svlTrainingDatasetBuilder::NEG_SUBDIR_NAME = string(value);
    } else if (!strcmp(name, "posPrefix")) {
      svlTrainingDatasetBuilder::POS_PREFIX = string(value);
    } else if (!strcmp(name, "negPrefix")) {
      svlTrainingDatasetBuilder::NEG_PREFIX = string(value);
    } else if (!strcmp(name, "includeFlipped")) {
      svlTrainingDatasetBuilder::INCLUDE_FLIPPED = valueTrue;
    } else if (!strcmp(name, "resizeWidth")) {
      svlTrainingDatasetBuilder::RESIZE_WIDTH = atoi(value);
    } else if (!strcmp(name, "resizeHeight")) {
      svlTrainingDatasetBuilder::RESIZE_HEIGHT = atoi(value);
    } else if (!strcmp(name, "useWinRefs")) {
      svlTrainingDatasetBuilder::USE_WIN_REFS = valueTrue;
    } else {
      SVL_LOG(SVL_LOG_FATAL, "unrecognized configuration option " << name << " for " << this->name());
    }
  }
};

static svlTrainingDatasetBuilderConfig gTrainingDatasetBuilderConfig;

