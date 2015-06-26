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
** FILENAME:    svlImageLoader.cpp
** AUTHOR(S):   Ian Goodfellow <ia3n@cs.stanford.edu>
**              Olga Russakovsky <olga@cs.stanford.edu>
**      based on applications by Stephen Gould and Sid Batra
**             
** DESCRIPTION:
**   Commonly re-used subroutines for processing command line arguments related
**   to patch types, mapping from channel types to patch types, or reading
**   and writing image files in the patch based object detector pipeline
**
*****************************************************************************/
#include "svlImageLoader.h"
#include "svlImageSequence.h"

// If true, then frames are associated with a mask matrix file, where
// 1s indicate that pixels should be treated as unobserved. Right now
// the only use of this is to block buildPatchDictionary from using
// certain image regions
bool svlImageLoader::USE_MASK = false;
string svlImageLoader::MASK_EXT = string(".occ");
string svlImageLoader::BASE_EXT = string(".jpg");
int svlImageLoader::DEPTH_MAP_WIDTH = -1;
int svlImageLoader::DEPTH_MAP_HEIGHT = -1;
int svlImageLoader::WIDTH = -1;
int svlImageLoader::HEIGHT = -1;
vector<pair<string, string> > svlImageLoader::CHANNELS;
vector<string> svlImageLoader::EXT_TO_IGNORE;

string svlChannelTypeToText(svlChannelType type)
{
  switch(type) {
  case SVL_CHANNEL_ERROR: return "ERROR";
  case SVL_CHANNEL_INTENSITY: return "INTENSITY";
  case SVL_CHANNEL_EDGE: return "EDGE";
  case SVL_CHANNEL_DEPTH: return "DEPTH";
  }
  return "";
}

// combination of parseCommandLineItem and finishCommandLine from before
svlImageLoader::svlImageLoader(bool default_use_edge, bool default_use_color) : 
  _useMask(USE_MASK), _maskExt(MASK_EXT),
  _depthMapWidth(DEPTH_MAP_WIDTH),
  _depthMapHeight(DEPTH_MAP_HEIGHT),
  _width(WIDTH), _height(HEIGHT)
{
  if (_depthMapWidth == -1 || _depthMapHeight == -1) {
    _depthMapWidth = WIDTH;
    _depthMapHeight = HEIGHT;
  }

  // Parsing the channels
  if (CHANNELS.size() == 0) { // create the default channels
    
    //Always assume there is an INTENSITY channel.
    _channelTypes.push_back(SVL_CHANNEL_INTENSITY);
    _channelPatchTypes.push_back(SVL_INTENSITY_PATCH);
    _channelExts.push_back(BASE_EXT);
    if (!default_use_color) {
      _colorTypes.push_back(SVL_COLOR_GRAY);
    } else {
      _colorTypes.push_back(SVL_COLOR_BGR);
    }

    // If necessary, populate the edge channel as well
    if (default_use_edge) {
      _channelTypes.push_back(SVL_CHANNEL_EDGE);
      _colorTypes.push_back(SVL_COLOR_GRAY);
      _channelPatchTypes.push_back(SVL_INTENSITY_PATCH);
      _channelExts.push_back(BASE_EXT);
    }
  } else { // channels specified
    for (unsigned i = 0; i < CHANNELS.size(); i++) {
      const char *type = CHANNELS[i].first.c_str();
      const char *ext = CHANNELS[i].second.c_str();
      
      svlChannelType chType = textToChannelType(type);

      SVL_ASSERT_MSG(chType != SVL_CHANNEL_ERROR, 
		     "unknown channel type " << type);

      svlColorType crType = textToColorType(type);

      SVL_ASSERT_MSG(chType != SVL_CHANNEL_ERROR, 
		     "unknown color type " << type);
      
      svlPatchDefinitionType patchType = channelTypeToPatchType(chType);
      
      _channelTypes.push_back(chType);
      _colorTypes.push_back(crType);
      _channelPatchTypes.push_back(patchType);
      
      _channelExts.push_back((!strcmp(ext, "-")) ? BASE_EXT : ext);
    }
  }

  SVL_LOG(SVL_LOG_DEBUG, "Channels\tTypes\tColors\tExts");

  for (unsigned i = 0; i < _channelTypes.size(); i++) {
    SVL_LOG(SVL_LOG_DEBUG, svlChannelTypeToText(_channelTypes[i]) << "\t"
	    << svlPatchDefinitionTypeToText(_channelPatchTypes[i]) << "\t"
	    << svlColorTypeToText(_colorTypes[i]) << "\t" << _channelExts[i]);
  }


  //use this to tell eTree not to load files ending in some
  //extension. TODO-- make patch dictionary smart enough to ignore
  //arbitrary channels, right now you can only ignore channels at the
  //end or the patch dictionary will come out with the wrong
  //validChannel indices
  for (unsigned i = 0; i < EXT_TO_IGNORE.size(); i++) {
    _eTree.ignore(EXT_TO_IGNORE[i].c_str());
  }

  // Prepare for loading files later on
  for (unsigned i = 0; i < _channelExts.size(); i++)
    _eTree.addChannel(i, _channelExts[i]);
  
  if (_useMask)
    _eTree.addChannel(_channelExts.size(), _maskExt);

}

svlImageLoader::svlImageLoader(const svlImageLoader & o) :
   _channelTypes(o._channelTypes), _colorTypes(o._colorTypes), 
   _channelPatchTypes(o._channelPatchTypes), _channelExts(o._channelExts), 
   _softEdgeCalculator(o._softEdgeCalculator), 
   _useMask(o._useMask), _maskExt(o._maskExt),
   _files(o._files),_eTree(o._eTree),
   _depthMapWidth(o._depthMapWidth), _depthMapHeight(o._depthMapHeight),
   _width(o._width), _height(o._height)
{
  // do nothing
}

static string stripExtension(const string & str, const string & ext)
{
  SVL_ASSERT(str.size() >= ext.size());
  return str.substr(0, str.size()-ext.size());
}

void svlImageLoader::readDir(const char * dir)
{
  //  _dir = string(dir) + string("/");

  _eTree.readDirectory(dir);

  vector<vector<string> > names;
  names.resize(_channelTypes.size());

  for (unsigned i = 0; i < names.size(); i++)
    _eTree.getFilesForChannel(i, names[i]);

  vector<string> &baseNames = names[0];
  for (unsigned i = 0; i < baseNames.size(); i++)
    baseNames[i] = stripExtension(baseNames[i], _channelExts[0]);

  //Check that the files all correspond to each other
  if (names.size() > 1)
    {
      for (unsigned i = 1; i < names.size(); i++)
	{
	  string extension;

	  if (i < _channelExts.size())
	    extension = _channelExts[i];
	  else
	    extension = _maskExt;

	  if (names[i].size() != names[0].size())
	    SVL_LOG(SVL_LOG_FATAL, "Channel " << i 
		    << " and channel 0 do not have the same number of frames: channel "
		    << i << " has " <<names[i].size() << " while channel 0 has " 
		    << names[0].size() << ". (Aborting after first difference)");
	  
	  for (unsigned j = 0; j < baseNames.size(); j++)
	    {
	      SVL_ASSERT(!strcmp(baseNames[j].c_str(), 
			     stripExtension(names[i][j], extension).c_str()));
	    }
	}
    }

  _files.clear();
  _files.setDirectory(string(dir));
  for (unsigned i = 0; i < baseNames.size(); i++) 
    _files.push_back(baseNames[i]);
}

void svlImageLoader::readImageSeq(const char *imageSeqFilename)
{
  _files.clear();
  _files.load(imageSeqFilename);

  if (!hasHomogeneousExtensions(_files)) {
    SVL_LOG(SVL_LOG_WARNING, "Sequence file " << imageSeqFilename
	    << " has multiple file extensions; be aware that they will be blindly"
	    << " stripped off and replaced by the standard channel extensions");
  }

  for (unsigned i = 0; i < _files.numImages(); i++)
    _files[i] = strWithoutExt(_files[i]);
}

void svlImageLoader::readImageSeq(const char *seqname, vector<string> &img_names)
{
  svlImageSequence sequence;
  sequence.load(seqname);
  if ( !hasHomogeneousExtensions(sequence) ) {
    SVL_LOG(SVL_LOG_WARNING, "Sequence file " << seqname
	    << " has multiple file extensions; be aware that they will be blindly"
	    << " stripped off and replaced by the standard channel extensions");
  }
  
  img_names.reserve(img_names.size() + sequence.size());
  // note: you might not want a "/" after the directory...
  for ( int i=0; i<int(sequence.size()); ++i )
    img_names.push_back(sequence.directory() + string("/") + sequence[i]);
}

void svlImageLoader::readCommandLine(char **args, int num_args)
{
  SVL_ASSERT(num_args > 0);

  _files.clear();

  string imageFilename = string(args[0]);
  if (strExtension(imageFilename).compare("xml") == 0) {
    readImageSeq(imageFilename.c_str());
  } else {
    _files.setDirectory("");
    for (int index = 0; index < num_args; index++) {
      _files.push_back(strWithoutExt(string(args[index])));
    }
  }
}


void svlImageLoader::getAllFrames(vector<vector<IplImage *> > & output,
				  unsigned maxToRead)
{
  output.clear();

  bool newFrameNeeded = true;
  for (unsigned int i = 0; i < _files.size() && i < maxToRead; i++)
    {
      if (newFrameNeeded)
	output.push_back(vector<IplImage *>() );

      newFrameNeeded = getFrame(i, output.back());
    }
}

void svlImageLoader::getAllFrames(vector<svlMaskedFrame> & output) 
{
  // TODO: optimize if regions come from the same file so don't have to
  // load it in every time
  output.clear();

  bool newFrameNeeded = true;
  for (unsigned int i = 0; i < _files.size(); i++)
    {
      if (newFrameNeeded)
	output.push_back(svlMaskedFrame() );

      newFrameNeeded = getFrame(i, output.back());
    }
}

bool svlImageLoader::getFrame(const string &filename, vector<IplImage *> &output) 
{
  string basename = strWithoutExt(filename);
  vector<string> filenames;
  for (unsigned ch = 0; ch < _channelTypes.size(); ch++)
    filenames.push_back(basename + _channelExts[ch]);
  return loadFullFrame(filenames, output);
}

bool svlImageLoader::getFrame(unsigned idx, vector<IplImage *> & output) 
{
  pair<string, CvRect> window = _files.get(idx);

  SVL_LOG(SVL_LOG_VERBOSE, "Getting frame " << idx << ", from " << window.first);

  vector<string> filenames;
  for (unsigned ch = 0; ch < _channelTypes.size(); ch++) {
    filenames.push_back(window.first + _channelExts[ch]);
  }

  return loadFullFrame(filenames, output, &(window.second));
}

bool svlImageLoader::getFrame(unsigned idx, svlMaskedFrame & output) 
{
  pair<string, CvRect> window = _files.get(idx);

  if (_useMask) {
    SVL_LOG(SVL_LOG_VERBOSE, "Getting masked frame " << window.first);
    if (window.second.width >= 0 && window.second.height >= 0)
      SVL_LOG(SVL_LOG_FATAL, "Mask not implemented yet for image sequences containing regions");
  } else {
    SVL_LOG(SVL_LOG_VERBOSE, "Getting frame " << window.first);
  }

  output.channels.clear();
  
  if (output.mask)
    cvReleaseMat(&output.mask);

  vector<string> filenames;
  for (unsigned ch = 0; ch < _channelTypes.size(); ch++)
    filenames.push_back(window.first + _channelExts[ch]);

  bool success1 = loadFullFrame(filenames, output.channels);

  if (success1)
    {
      if (_useMask)
	{
	  SVL_ASSERT(output.channels.size());
	  output.mask = cvCreateMat(output.channels[0]->height,
				     output.channels[0]->width,
				     CV_32FC1);
	  SVL_ASSERT(output.mask);

	  bool success2 = readMatrix(output.mask, (window.first + _maskExt).c_str() );

	  return success2;
	}
      else
	{
	  output.mask = NULL;
	}

      return true;
    }
  else
    return false;

  //unreached
  SVL_ASSERT(false);
  return false;
}

bool svlImageLoader::getFrameFromFiles(const vector<string> & files,
				       vector<IplImage *> & output) 
{
  return loadFullFrame(files, output);
}

/*
bool svlImageLoader::getFrameFromBasename(const string & baseName,
					  vector<IplImage *> & output) 
{
  string strippedName = stripExtension(baseName);

  vector<string> filenames;

  for (unsigned ch = 0; ch < _channelExts.size(); ch++)
    filenames.push_back(strippedName + _channelExts[ch]);

  return getFrameFromFiles(filenames, output);
  }*/


IplImage *svlImageLoader::getSimpleImageFromFilename(const char *filename,
						     unsigned channel)
{
  string fullname = strWithoutExt(string(filename)) + _channelExts[channel];

  // don't do any cropping, do resize the image appropriately, don't
  // do any color conversion
  return loadImage(fullname, channel, NULL, true, false);
}

void svlImageLoader::setDepthMapSize(int width, int height)
{
  _depthMapWidth = width;
  _depthMapHeight = height;
}

void svlImageLoader::setResizeSize(int width, int height)
{

  _width = width;
  _height = height;

  if (_depthMapWidth == -1)
    setDepthMapSize(width, height);
}

unsigned svlImageLoader::numFrames() const
{
  return (unsigned)_files.size();
}

string svlImageLoader::getFrameBasename(unsigned i) const
{
  pair<string, CvRect> result = _files.get(i);
  return result.first;
}

void svlImageLoader::printChannelInfo(unsigned chNum) const
{
  SVL_ASSERT(chNum < _channelPatchTypes.size());

  svlPatchDefinitionType chPatchType = _channelPatchTypes[chNum];
  string chExt = _channelExts[chNum];
  
  string msg = "Channel:\t";
  switch (chPatchType) {
  case SVL_INTENSITY_PATCH:
    msg += string("INTENSITY");
    break;
  case SVL_DEPTH_PATCH:
    msg += string("DEPTH");
    break;
  default:
    SVL_ASSERT(false);
  }
  msg += string("\t") + chExt;
  SVL_LOG(SVL_LOG_MESSAGE, msg);
}  


// private functions ----------------------------------------------------

bool svlImageLoader::loadFullFrame(const vector<string> & files, vector<IplImage *> & output,
				   CvRect *ROI, bool resize, bool colorConversion)
{
  if (files.size() != _channelTypes.size()) {
    SVL_LOG(SVL_LOG_ERROR, "file size doesn't match number of channels");
    return false;
  }
  
 for (unsigned ch = 0; ch < _channelTypes.size(); ch++) {
    const string & filename = files[ch];

    IplImage * img = loadImage(filename, ch, ROI, resize, colorConversion);
    
    if (img == NULL) {
      for (unsigned ch2 = 0; ch2 < ch; ch2++)
	cvReleaseImage(&output[ch2]);

      output.clear();
      
      return false;
    } else {
      output.push_back(img);	  
    }    
  }
  
  return true;
}

IplImage *svlImageLoader::loadImage(const string &path, unsigned ch,
				    CvRect *ROI, bool resize,
				    bool colorConversion)
{
  IplImage *img = NULL;
  
  switch (_colorTypes[ch]) {
  case SVL_COLOR_GRAY:
    img = cvLoadImage(path.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
    break;
  case SVL_COLOR_UNDEFINED:
    img = readMatrixAsIplImage(path.c_str(), _depthMapWidth, _depthMapHeight);
    break;
  default:
    img = cvLoadImage(path.c_str(), CV_LOAD_IMAGE_ANYCOLOR);
  }
  
  if (img == NULL) {
    SVL_LOG(SVL_LOG_MESSAGE, "...failed to load " << path);
    return img;
  }

  // crop if necessary
  if (ROI && ROI->width > 0 && ROI->height > 0) {
    cropInPlace(&img, *ROI);
  }
	   
  // resize the remaining region if necessary
  if (resize && ((img->width != (int) _width) || (img->height != (int) _height))) {
    if (_height != -1) {
      resizeInPlace(&img, _height, _width);
    }
  }

  // do the necessary color conversion for non-grayscale images
  if (colorConversion) {
    if (_colorTypes[ch] != SVL_COLOR_GRAY && _colorTypes[ch] != SVL_COLOR_UNDEFINED)
      svlChangeColorModel(img, _colorTypes[ch]);

    if (_channelTypes[ch] == SVL_CHANNEL_EDGE) {
      IplImage *temp = cvCloneImage(_softEdgeCalculator.processImage(img));
      cvReleaseImage(&img);
      img = temp;
    }
  }

  return img;
}

// command line processing

svlChannelType svlImageLoader::textToChannelType(const char *text) const {
  if (!strncmp(text, "INTENSITY", 9)) 
    return SVL_CHANNEL_INTENSITY;
  
  if (!strncmp(text, "EDGE", 4)) 
    return SVL_CHANNEL_EDGE;
   
  if (!strcmp(text, "DEPTH")) 
    return SVL_CHANNEL_DEPTH; 
    
  return SVL_CHANNEL_ERROR;
}

svlColorType svlImageLoader::textToColorType(const char *text) const {
  if (!strncmp(text, "DEPTH", 5))
    return SVL_COLOR_UNDEFINED;

    // if just "EDGE" or just "INTENSITY", the color is gray
  if (!strcmp(text, "INTENSITY") || !strcmp(text, "EDGE"))
    return SVL_COLOR_GRAY;
    
  const char *color = NULL;

  if (!strncmp(text, "INTENSITY_", 10)) {
    // if "INTENSITY_<colortype>"
     color = text + 10;
  } else if (!strncmp(text, "EDGE_", 5)) {
    // otherwise, should be EDGE_<colortype>
    color = text + 5;
  } else {
    return SVL_COLOR_ERROR;
  }

  if (!strcmp(color, "RGB"))
    return SVL_COLOR_BGR;
  
  if (!strcmp(color, "BG"))
    return SVL_COLOR_BG;
  
  if (!strcmp(color, "BR"))
    return SVL_COLOR_BR;
  
  if (!strcmp(color, "GR"))
    return SVL_COLOR_GR;
  
  if (!strcmp(color, "HSV"))
    return SVL_COLOR_HSV;
  
  if (!strcmp(color, "HS"))
    return SVL_COLOR_HS;
  
  if (!strcmp(color, "HV"))
    return SVL_COLOR_HV;
  
  if (!strcmp(color, "YCrCb"))
    return SVL_COLOR_YCrCb;
  
  if (!strcmp(color, "CrCb"))
    return SVL_COLOR_CrCb;
  
  if (!strcmp(color, "H"))
    return SVL_COLOR_H;
  
  if (!strcmp(color, "R"))
    return SVL_COLOR_R;
  
  if (!strcmp(color, "G"))
    return SVL_COLOR_G;
  
  if (!strcmp(color, "B"))
    return SVL_COLOR_B;

  return SVL_COLOR_ERROR;
}

svlPatchDefinitionType svlImageLoader::channelTypeToPatchType(svlChannelType type) const {
  switch(type) {

  case SVL_CHANNEL_INTENSITY:
  case SVL_CHANNEL_EDGE:
    return SVL_INTENSITY_PATCH;

  case SVL_CHANNEL_DEPTH:
    return SVL_DEPTH_PATCH;

  case SVL_CHANNEL_ERROR:
    SVL_ASSERT(false); 
  }

  SVL_ASSERT(false); 
  return SVL_INTENSITY_PATCH; // to suppress warnings
}


class svlImageLoaderConfig : public svlConfigurableModule {
public:
    svlImageLoaderConfig() : svlConfigurableModule("svlVision.svlImageLoader") {}

    void usage(ostream &os) const {
        os << "      channels     :: space-separated channel-extension pairs\n"
           << "      defaultExtension :: image extension (default: " << svlImageLoader::BASE_EXT << ")\n"
           << "      resizeWidth  :: resize each image after loading\n" 
           << "      resizeHeight :: resize each image after loading\n" 
           << "      depthMapWidth  :: required if using depth maps\n"
           << "      depthMapHeight :: required if using depth maps\n"
           << "      useMask        :: mask out region in image\n"
           << "      maskExtension  :: file extension for mask\n"
           << "      ignoreExtensions :: loader strips file extensions\n";
    }

    void readConfiguration(XMLNode& node) {
        // process everything except the channels and the extensionsToIgnore
        svlConfigurableModule::readConfiguration(node);
        
        // parse <channel type="" ext=""/>, one per channel
        for (int i = 0; i < node.nChildNode("channel"); i++) {
            XMLNode child = node.getChildNode("channel", i);
            const char *type = child.getAttribute("type");
            const char *extension = child.getAttribute("ext");
            SVL_ASSERT((type != NULL) && (extension != NULL));
            svlImageLoader::CHANNELS.push_back(make_pair<string, string>(type, extension));
        }
        
        // parse <ignore ext=""/>
        for (int i = 0; i < node.nChildNode("ignore"); i++) {
            XMLNode child = node.getChildNode("ignore", i);
            const char *extension = child.getAttribute("ext");
            SVL_ASSERT(extension != NULL);
            svlImageLoader::EXT_TO_IGNORE.push_back(string(extension));
        }
    }
    
  void setConfiguration(const char *name, const char *value) {
    if (!strcmp(name, "defaultExtension")) {
      svlImageLoader::BASE_EXT = string(value);
    } else if (!strcmp(name, "maskExtension")) {
      svlImageLoader::MASK_EXT = string(value);
    } else if (!strcmp(name, "useMask")) {
      svlImageLoader::USE_MASK = 
	(!strcasecmp(value, "TRUE") || !strcmp(value, "1"));
    } else if (!strcmp(name, "resizeWidth")) {
      svlImageLoader::WIDTH = atoi(value);
    } else if (!strcmp(name, "resizeHeight")) {
      svlImageLoader::HEIGHT = atoi(value);
    } else if (!strcmp(name, "depthMapWidth")) {
      svlImageLoader::DEPTH_MAP_WIDTH = atoi(value);
    } else if (!strcmp(name, "depthMapHeight")) {
      svlImageLoader::DEPTH_MAP_HEIGHT = atoi(value);
    } else if (!strcmp(name, "channels")) {
      // parse channels (clear existing so command line can override config file)
      svlImageLoader::CHANNELS.clear();
      string value_copy(value);
      // the following delimeters are valid; first, try parsing on spaces, and if that
      // fails, use ":" instead
      const char *delimeters[2] = {" ", ":"};
      bool success = false;
      for (int i = 0; i < 2; i++) {
	const char *ptr = strtok((char *)value_copy.c_str(), delimeters[i]);
	while (ptr != NULL) {
	  string channelType = string(ptr);
	  ptr = strtok(NULL, delimeters[i]);
	  if (!ptr) {
	    success = false;
	    break;
	  }
	  string extension = string(ptr);
	  svlImageLoader::CHANNELS.push_back(make_pair<string, string>(channelType, extension));
	  success = true;
	  ptr = strtok(NULL, delimeters[i]);
	}
	if (success) break;
      } 
      SVL_ASSERT_MSG(success, "Channel string must contain \"CHANNEL_TYPE EXTENSION\" pairs");
    } else if (!strcmp(name, "ignoreExtensions")) {
      // parse extensions to ignore (clear existing so command line can override config file)
      svlImageLoader::EXT_TO_IGNORE.clear();
      string value_copy(value);
      const char *ptr = strtok((char *)value_copy.c_str(), " ");
      while (ptr != NULL) {
	svlImageLoader::EXT_TO_IGNORE.push_back(string(ptr));
	ptr = strtok(NULL, " ");
      }
    } else {
      SVL_LOG(SVL_LOG_FATAL, "unrecognized configuration option for " << this->name());
    }
  }
};

static svlImageLoaderConfig gImageLoaderConfig;

