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
** FILENAME:    svlImageLoader.h
** AUTHOR(S):   Ian Goodfellow <ia3n@cs.stanford.edu>
**              Olga Russakovsky <olga@cs.stanford.edu>
**             
** DESCRIPTION:
**  Functionality for reading and writing image files in
** the patch based object detector pipeline
**
*****************************************************************************/

#pragma once

#include "svlBase.h"
#include <limits.h>
#include "svlOpenCVUtils.h"
#include "svlSoftEdgeMap.h"
#include "svlPatchDefinition.h"
#include "svlImageSequence.h"
#include "svlExtensionTree.h"

enum svlChannelType
  {
    SVL_CHANNEL_ERROR, // to return from textToChannelType
    SVL_CHANNEL_INTENSITY,
    SVL_CHANNEL_EDGE,
    SVL_CHANNEL_DEPTH,
  };

string svlChannelTypeToText(svlChannelType type);

struct svlMaskedFrame
{
svlMaskedFrame() : mask(NULL) {}
  
  vector<IplImage *> channels;
  CvMat * mask;
};


class svlImageLoader
{
  // might want to initialize the base extension from the provided
  // image sequence
  friend class svlTrainingDatasetBuilder;

 public:
  static string BASE_EXT;
  static string MASK_EXT;
  static bool USE_MASK;
  static int DEPTH_MAP_WIDTH;
  static int DEPTH_MAP_HEIGHT;
  static int WIDTH;
  static int HEIGHT;
  static vector<pair<string, string> > CHANNELS;
  static vector<string> EXT_TO_IGNORE;

 public:
  // together with ConfigManager now encapsulates the functionality of
  // parseCommandLineItem and finishCommandLine; default_use_edge
  // determines if an edge channel will be used (together with an
  // intensity channel) by default if no channels are specified
  // default_use_color forces the first channel to be color instead of grayscale
  svlImageLoader(bool default_use_edge = true, bool default_use_color = false);
  svlImageLoader(const svlImageLoader & other);
  
  //Uses the channel definitions to load all of the appropriate
  //filepaths for a directory.
  void readDir(const char * dir);

  // Loads in the imageSequence, strips off the extension from
  // every filename and uses the appropriate channel extensions
  // instead.
  void readImageSeq(const char *imageSeqFilename);

  // ==========================================================
  // TODO: please move this to svlImageSequence
  // ==========================================================
  static void readImageSeq(const char *seqname, vector<string> &img_names);

  // Reads in the list of filenames, each one corresponding
  // to either an image sequence (.xml extension) or an image file;
  // reads the image sequences and stores all images together
  // for later access
  void readCommandLine(char** args, int num_args);
  
  //Gets all of the multi-channel patches referred to by the filepaths
  //already loaded internally from a call to one of the read
  //functions. On return, output[i][j] is frame i, channel j
  void getAllFrames(vector<vector<IplImage *> > & output,
		    unsigned maxNumToRead = UINT_MAX);

  //Same thing but allowing for masks
  void getAllFrames(vector<svlMaskedFrame > & output) ;

  //Gets a single specific frame. Again, call only after calling read
  //Return type is true on success.
  //On failure, return type is false and output is empty (cleared)
  bool getFrame(unsigned idx, vector<IplImage *> & output) ;
  bool getFrame(unsigned idx, svlMaskedFrame & output) ;

  // as above, except takes the name of a file and loads all channels
  // by swapping the extension for each channel
  bool getFrame(const string &filename, vector<IplImage *> &output);

  // Takes a vector of filenames to use for each
  // channel, rather than using an index into filenames obtained from
  // one of the read commands
  bool getFrameFromFiles(const vector<string> & files,
			 vector<IplImage *> & output);

  // Deleted to avoid confusion, since this will not actually change
  // the channels' extension in any way; only way to set is now through
  // the static BASE_EXT
  //void setBaseExt(const string & ext) { _baseExt = ext; }

  // no longer needed + confusing since basename often refers to name
  // without an extension

  //Like getFrame, but 
  //bool getFrameFromBasename(const string & baseName,
  //			    vector<IplImage *> & output);

  // Takes the name of a file, swaps the extension for this channel's
  // extension, and loads the single image corresponding to this
  // channel without any color conversion post-processing (but does
  // resize it appropriately)
  IplImage *getSimpleImageFromFilename(const char *filename,
				       unsigned channel);

  //Sets the size of the depth map (can be done from the command line as well)
  void setDepthMapSize(int width, int height);

  //Sets the size images are resized to for getFrame/getAllFrames
  //If the size of the depth map is invalid, this also sets the depth map size
  // (can be done from the command line as well)
  void setResizeSize(int width, int height);
  int getResizeWidth() { return _width; }
  int getResizeHeight() { return _height; }

  //Frames access functions
  unsigned numFrames() const;
  string getFrameBasename(unsigned i) const; // will have no extension

  //Channel definition access functions
  unsigned numChannels() const { return (int)_channelTypes.size(); }
  svlColorType colorType(unsigned idx) const { return _colorTypes[idx]; }
  svlChannelType channelTypes(unsigned idx) const { return _channelTypes[idx]; }
  svlPatchDefinitionType patchTypes(unsigned idx) const { return _channelPatchTypes[idx]; }
  string extensions(unsigned idx) const { return _channelExts[idx]; }

  void printChannelInfo(unsigned chNum) const;

  protected:

  // !!!! if you add new fields, also add them to the copy constructor !!!!

  //Definitions of the channels
  vector<svlChannelType> _channelTypes;
  vector<svlColorType> _colorTypes;
  vector<svlPatchDefinitionType> _channelPatchTypes;
  vector<string> _channelExts;

  svlSoftEdgeMap _softEdgeCalculator;

    //The file extension of the mask files
  bool _useMask;
  string _maskExt;

  // stores the filenames without the extension; can later be filled in
  // with the appropriate channel extensions
  svlImageRegionsSequence _files;

  //Used to get files out of a directory and map the files to the
  //correct channel
  svlExtensionTree _eTree;


  int _depthMapWidth;
  int _depthMapHeight;

  int _width;
  int _height;

  bool loadFullFrame(const vector<string> &files, vector<IplImage *> &output,
		 CvRect *ROI = NULL, bool resize = true, bool colorConversion = true);
  IplImage *loadImage(const string &path, unsigned ch,
		      CvRect *ROI, bool resize, bool colorConversion);

  // command line processing
  svlChannelType textToChannelType(const char *text) const;
  svlColorType textToColorType(const char *text) const;
  svlPatchDefinitionType channelTypeToPatchType(svlChannelType type) const;

  bool getFrameFromFilesHelper(const vector<string> & files,
			       vector<IplImage *> & output,
			       CvRect cropToROI = cvRect(0, 0, -1, -1));


};





