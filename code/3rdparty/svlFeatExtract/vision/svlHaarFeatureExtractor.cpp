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
** FILENAME:    svlHaarFeatureExtractor.cpp
** AUTHOR(S):   Ian Goodfellow <ia3n@cs.stanford.edu>
**
*****************************************************************************/

#include <sstream>

#include "svlHaarFeatureExtractor.h"
#include "svlVision.h"

//If enabled, it's OK to pass in rectangles that go outside the image
//boundaries. Any out of bounds pixels will be treated as 0
#define HAAR_BOUNDS_CHECK 1

static const svlObject2d topHalf(0,0,1,0.5);
static const svlObject2d bottomHalf(0,0.5,1.0,0.5);
static const svlObject2d leftHalf(0,0,0.5,1);
static const svlObject2d rightHalf(0.5,0,0.5,1);
static const svlObject2d ulQuad(0,0,0.5,0.5);
static const svlObject2d urQuad(0.5,0,0.5,0.5);
static const svlObject2d llQuad(0,0.5,0.5,0.5);
static const svlObject2d lrQuad(0.5,0.5,0.5,0.5);


void svlHaarFeatureExtractor::extract(const vector<CvPoint> &locations,
	      const vector<svlDataFrame*> &frames,
	      vector< vector<double> > &output,
	      bool sparse,
	      int outputOffset) const
{

  if (output.size() < locations.size())
    output.resize(locations.size());


  svlImageFrame * imgFrame = dynamic_cast<svlImageFrame *>(frames.at(_validChannel));

  if (!imgFrame)
    SVL_LOG(SVL_LOG_FATAL, "Channel "<<_validChannel<<" is not an image channel, yet was passed to Haar feature extractor");

  IplImage * img = imgFrame->image;

  IplImage * iImg = cvCreateImage(cvSize(img->width+1, img->height+1), IPL_DEPTH_64F,1);

  cvIntegral(img, iImg);

  CvRect focus = cvRect(0,0,_windowSize.width, _windowSize.height);

  SVL_ASSERT(_locations.size() == _featureTypes.size());

  for (unsigned i = 0; i < locations.size(); i++)
    {
      if (output[i].size() < _featureTypes.size() + outputOffset)
	output[i].resize(_featureTypes.size()+outputOffset);

      focus.x = locations[i].x;
      focus.y = locations[i].y;
      for (unsigned j = 0; j < _featureTypes.size(); j++)
	{
	  output[i][j+outputOffset] = calcHaarFeature(iImg, focus, _locations[j], _featureTypes[j]);
	}
    }

  cvReleaseImage(&iImg);
}

bool svlHaarFeatureExtractor::writeOut(ofstream & ofs)
{

  ofs << "<HaarFeatureExtractor version=\"1.0\" ";
  ofs << "numEntries=\""<<_locations.size()<<"\" ";
 ofs << "validChannel=\""<<_validChannel<<"\" ";
  ofs << "width = \"" << _windowSize.width << "\" ";
  ofs << "height = \"" << _windowSize.height << "\" ";
  ofs << ">" << endl;

  for (unsigned i = 0; i < _locations.size(); i++)
    {
      ofs << "\t<HaarFeature ";
      ofs << "type=\"" << haarTypeToStr(_featureTypes[i]) << "\" ";
     ofs << "x=\"" << _locations[i].x << "\" ";
      ofs << "y=\"" << _locations[i].y << "\" ";
      ofs << "w=\"" << _locations[i].w << "\" ";
      ofs << "h=\"" << _locations[i].h << "\" ";
      ofs << ">" << endl;
    }

  ofs << "</HaarFeatureExtractor>" << endl;

  return ! ofs.fail();
}

bool svlHaarFeatureExtractor::load(XMLNode & root)
{
  _locations.clear();
  _featureTypes.clear();
  
  if (root.isEmpty())
    {
      SVL_LOG(SVL_LOG_WARNING, "Haar Feature Extractor XML root is empty");
      return false;
    }

  if (strcmp(root.getName(), "HaarFeatureExtractor") != 0)
    {
      SVL_LOG(SVL_LOG_WARNING, "Attempt to read Haar feature extractor from XML node that is not a Haar feature extractor. Expected \"HaarFeatureExtractor\", received \""<<root.getName()<<"\"");
      

      return false;
    }

  //todo-- add some sort of check that all of the following getAttribute calls are working? (I just copied the method from PatchDictionary and it just assumes they work)

  _windowSize.width = atoi(root.getAttribute("width"));
  _windowSize.height = atoi(root.getAttribute("height"));

  SVL_LOG(SVL_LOG_MESSAGE, "Haar feature extractor loaded with size "<<_windowSize.width<<"x"<<_windowSize.height);

  _validChannel = atoi(root.getAttribute("validChannel"));

  const char * numEntries = root.getAttribute("numEntries");

  unsigned  n = 0;

  if (numEntries)
    n = atoi(numEntries);
  else
    {
      SVL_LOG(SVL_LOG_WARNING, "attempted to read HaarFeatureExtractor with no numEntries property");
      return false;
    }

  SVL_ASSERT(root.nChildNode("HaarFeature")==(int)n);
  _locations.resize(n);
  _featureTypes.resize(n);

  for (unsigned i = 0; i < n; i++)
    {
      XMLNode node = root.getChildNode("HaarFeature", i);
      _locations[i].x = atoi(node.getAttribute("x"));
      _locations[i].y = atoi(node.getAttribute("y"));
      _locations[i].w = atoi(node.getAttribute("w"));
      _locations[i].h = atoi(node.getAttribute("h"));
      _featureTypes[i] = strToHaarType(node.getAttribute("type"));
    }

 SVL_LOG(SVL_LOG_MESSAGE, "Haar feature extractor finished loading with size "<<_windowSize.width<<"x"<<_windowSize.height);





  return true;
}

svlFeatureExtractor* svlHaarFeatureExtractor::getPrunedExtractor
                                        (const vector<bool> &featureUsedBits) const
{
  svlHaarFeatureExtractor * other = new svlHaarFeatureExtractor(*this);

  SVL_ASSERT(other);

  vector<svlObject2d>::iterator locIter = other->_locations.begin();
  vector<svlHaarFeatureType>::iterator typeIter = other->_featureTypes.begin(), last = other->_featureTypes.end();
  unsigned i = 0;

  while (typeIter != other->_featureTypes.end())
    {
      if (featureUsedBits[i])
	{
	  ++locIter;
	  ++typeIter;
	}
      else
	{
	  other->_locations.erase(locIter);
	  other->_featureTypes.erase(typeIter);
	}
      i++;
    }

  svlFeatureExtractor * rval = dynamic_cast<svlFeatureExtractor *>(other);

  SVL_ASSERT(rval);

  if (!rval)
    delete other;

  return rval;
}

unsigned svlHaarFeatureExtractor::numFeatures() const
{
  //sanity check
  SVL_ASSERT(_locations.size() == _featureTypes.size());

  return _locations.size();
}

svlFeatureExtractor * svlHaarFeatureExtractor::clone() const
{
  svlHaarFeatureExtractor * temp = new svlHaarFeatureExtractor(*this);
  if (!temp)
    return NULL;

  svlFeatureExtractor * rval = dynamic_cast<svlFeatureExtractor *>(temp);

  SVL_ASSERT(rval);//this should always succeed, if not something is wrong with the code

  return rval;
}


double svlHaarFeatureExtractor::iImgSum(const IplImage * iImg, const CvRect & rect)
{

#if HAAR_BOUNDS_CHECK

  if (rect.x >= iImg->width)
    return 0;

  if (rect.y >= iImg->height)
    return 0;

  int right = rect.x + rect.width -1;

  if (right < 0)
    return 0;

  int top = rect.y + rect.height -1;

  if (top < 0)
    return 0;

  int left;

  if (rect.x < 0)
    left = 0;
  else
    left = rect.x;

  int bottom;

  if (rect.y < 0)
    bottom = 0;
  else
    bottom = rect.y;

  if (right >= iImg->width)
    right = iImg->width - 1;

  if (top >= iImg->height)
    top = iImg->height -1;

  double lr = cvGetReal2D(iImg, top, right);
  double ur = cvGetReal2D(iImg, bottom, right);
  double ll = cvGetReal2D(iImg, top, left);
  double ul = cvGetReal2D(iImg, bottom, left);

#else

  SVL_ASSERT(rect.x+rect.width-1 < iImg ->width);
  double lr = cvGetReal2D(iImg, rect.y+rect.height-1,rect.x+rect.width-1);
  double ur = cvGetReal2D(iImg, rect.y, rect.x+rect.width - 1);
  double ll = cvGetReal2D(iImg, rect.y+rect.height-1, rect.x);
  double ul = cvGetReal2D(iImg, rect.y, rect.x);
#endif

  return lr - ur - ll + ul;
}

CvRect svlHaarFeatureExtractor::rectInRect(const svlObject2d & propRect, const CvRect & imRect)
{
  CvRect rval;
  rval.width  = (int)round(propRect.w * double(imRect.width));
  rval.x = imRect.x + (int)round(propRect.x * double(imRect.width));
 
  if (rval.width == 0)
    rval.width = 1;
 
  if (rval.x+rval.width > imRect.x+double(imRect.width))
    rval.x -= 1;
  

  rval.height  = (int)round(propRect.h * double(imRect.height));

  if (rval.height == 0)
    rval.height = 1;

  rval.y = imRect.y + (int)round(propRect.y * double(imRect.height));
  
  if (rval.y+rval.height > imRect.y + imRect.height)
    rval.y -= 1;

  SVL_ASSERT(rval.x >= imRect.x);
  SVL_ASSERT(rval.y >= imRect.y);


  //cout<<endl;
  //cout<<"propRect.x" << propRect.x<<endl;
  //cout<<"imRect.x" << imRect.x<<endl;
  //cout<<"imRect.width"<< imRect.width<<endl;
  //cout<<"rval.x"<<rval.x<<endl;
  
  return rval;
}

double svlHaarFeatureExtractor::calcHaarFeature(const IplImage * iImg, const CvRect & focus, const svlObject2d & featureArea, svlHaarFeatureType type)
{

  CvRect haarFocus = rectInRect(featureArea, focus);

  double diff;

  switch(type)
    {
    case SVL_HAAR_V:
      {
	CvRect left = rectInRect(leftHalf, haarFocus);
	CvRect right = rectInRect(rightHalf, haarFocus);

	diff = iImgSum(iImg, left) - iImgSum(iImg, right);
      }
      break;

    case SVL_HAAR_H:
      {
	CvRect top = rectInRect(topHalf, haarFocus);
	CvRect bottom = rectInRect(bottomHalf, haarFocus);

	diff = iImgSum(iImg, top) - iImgSum(iImg, bottom);
      }
      break;

    case SVL_HAAR_D:
      {
	CvRect ul = rectInRect(ulQuad, haarFocus);
	CvRect ur = rectInRect(urQuad, haarFocus);
	CvRect ll = rectInRect(llQuad, haarFocus);
	CvRect lr = rectInRect(lrQuad, haarFocus);

	diff = iImgSum(iImg, ul) + iImgSum(iImg, lr) - iImgSum(iImg, ur) - iImgSum(iImg, ll);
      }
      break;

    case SVL_HAAR_TL:
      {
	CvRect bottom = rectInRect(bottomHalf, haarFocus);
	CvRect ul = rectInRect(ulQuad, haarFocus);
	CvRect ur = rectInRect(urQuad, haarFocus);

	diff = iImgSum(iImg, bottom) + iImgSum(iImg, ur) - iImgSum(iImg, ul);
      }
      break;

    case SVL_HAAR_TR:
      {
	CvRect bottom = rectInRect(bottomHalf, haarFocus);
	CvRect ul = rectInRect(ulQuad, haarFocus);
	CvRect ur = rectInRect(urQuad, haarFocus);

	diff = iImgSum(iImg, bottom) + iImgSum(iImg, ul) - iImgSum(iImg, ur);
      }
      break;

    case SVL_HAAR_BL:
      {
	CvRect top = rectInRect(topHalf, haarFocus);
	CvRect ll = rectInRect(llQuad, haarFocus);
	CvRect lr = rectInRect(lrQuad, haarFocus);

	diff = iImgSum(iImg, top) + iImgSum(iImg, lr) - iImgSum(iImg, ll);
      }
      break;

    case SVL_HAAR_BR:
      {
	CvRect top = rectInRect(topHalf, haarFocus);
	CvRect ll = rectInRect(llQuad, haarFocus);
	CvRect lr = rectInRect(lrQuad, haarFocus);

	diff = iImgSum(iImg, top) + iImgSum(iImg, ll) - iImgSum(iImg, lr);
      }
      break;

    case SVL_HAAR_FULL_AVE:
      {
	double regionSum = iImgSum(iImg, haarFocus);
	return regionSum / double(haarFocus.width*haarFocus.height);
      }
      break;

    default:
      SVL_ASSERT(false); //should be unreachable

    }


  double Z = iImgSum(iImg, focus);

  //Add 1 to normalization term in case it is zero
  return diff/(Z+1.0);
}


string svlHaarFeatureExtractor::haarTypeToStr(svlHaarFeatureType type)
{
  switch (type)
    {
    case SVL_HAAR_H:
      return "SVL_HAAR_H";

    case SVL_HAAR_V:
      return "SVL_HAAR_V";
    case SVL_HAAR_D:
      return "SVL_HAAR_D";
    case SVL_HAAR_TL:
      return "SVL_HAAR_TL";
    case SVL_HAAR_TR:
      return "SVL_HAAR_TR";
    case SVL_HAAR_BL:
      return "SVL_HAAR_BL";
    case SVL_HAAR_BR:
      return "SVL_HAAR_BR";
    case SVL_HAAR_FULL_AVE:
      return "SVL_HAAR_FULL_AVE";

    default:
      ;
    }

  SVL_ASSERT(false);//should be unreached
  return "There is an error in svlHaarFeatureExtractor::haarTypeToStr";
}

svlHaarFeatureType svlHaarFeatureExtractor::strToHaarType(const string & str)
{
  if (str == "SVL_HAAR_H")
    return SVL_HAAR_H;
  else if (str =="SVL_HAAR_V")
    return SVL_HAAR_V;
  else if (str =="SVL_HAAR_D")
    return SVL_HAAR_D;
  else if (str =="SVL_HAAR_TL")
    return SVL_HAAR_TL ;
  else if (str =="SVL_HAAR_TR")
    return SVL_HAAR_TR ;
  else if (str =="SVL_HAAR_BL")
    return SVL_HAAR_BL;
  else if (str =="SVL_HAAR_BR")
    return SVL_HAAR_BR;
  else if (str =="SVL_HAAR_FULL_AVE")
    return SVL_HAAR_FULL_AVE;

  SVL_ASSERT(false); //should be unreached
  return SVL_HAAR_ERROR;
}

string svlHaarFeatureExtractor::summary() const
{
  stringstream s;
  s << "["<<numFeatures()<<" haar features]";
  return s.str();
}
