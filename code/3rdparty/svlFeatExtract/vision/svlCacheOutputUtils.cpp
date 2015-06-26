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
** FILENAME:    svlCacheOutputUtils.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
**              Ian Goodfellow <ia3n@cs.stanford.edu>
**
*****************************************************************************/

#include <cstdlib>
#include <iostream>
#include <fstream>
#include <vector>
#include <limits>
#include <list>
#include <map>

#include "cv.h"
#include "cxcore.h"
#include "highgui.h"

#include "svlBase.h"
#include "svlCacheOutputUtils.h"

using namespace std;

vector<vector<double> > svlReadExamplesFromCache(const char *dirName, long int maxImages,
						 bool binary)
{
  vector<vector<double> > fv;
  
  if (!binary) {
    vector<string> filenames = svlDirectoryListing(dirName, ".txt");

    
    for (unsigned i = 0; i < filenames.size() && (maxImages < 0 || i < (unsigned)maxImages); i++) {
      ifstream ifs(filenames[i].c_str());
      if (ifs.fail()) {
	SVL_LOG(SVL_LOG_WARNING, "...failed on " << filenames[i]);
      } else {
	vector<double> v;
	double d;
	while (1) {
	  ifs >> d;
	  if (ifs.fail()) break;
	  v.push_back(d);
	}
	fv.push_back(v);
      }
    }
  } else {
    ifstream ifs(dirName, std::ios::in|std::ios::binary);

    while (maxImages < 0 || fv.size() < (unsigned)maxImages) {
      int size = -1;
      ifs.read((char *) &size, sizeof(int));
      if (ifs.fail() || size < 0) break;

      vector<double> v(size);
      for (int i = 0; i < size; i++)
	ifs.read((char *) &v[i], sizeof(double));
      fv.push_back(v);
    }

    ifs.close();
  }

  return fv;
}

void svlWriteExampleToCache(const char *dirName, unsigned sample_idx, vector<double> &v) {
  string outputFilename = string(dirName) + string("/") + "sample" + toString(sample_idx) + ".txt";
    
  //SVL_LOG(SVL_LOG_VERBOSE, "...writing output to " << outputFilename.c_str());
  
  ofstream ofs;
  SVL_ASSERT(!ofs.fail());
  ofs.open(outputFilename.c_str());
  if (ofs.fail()) {
    SVL_LOG(SVL_LOG_WARNING, "Could not write to "<<outputFilename);
  } else {
    for (unsigned i = 0; i < v.size(); i++) {
      ofs << v[i] << endl;
    }
  }
  ofs.close(); 
}

void svlWriteExampleToBinaryCache(std::ostream &ofs, vector<double> &v) {
    unsigned sz = v.size();
    ofs.write((const char*)&sz, sizeof(unsigned));
    //uint32_t sz = v.size();
    //ofs.write((const char*)&sz, sizeof(uint32_t));
    for (unsigned i = 0; i < v.size(); i++) {
      ofs.write((const char*)&v[i], sizeof(double));
    }
}

SWcacheInfo::SWcacheInfo(string dir, string baseName) :
  img_w(-1), img_h(-1), w(-1), h(-1), 
  delta_x(1), delta_y(1), delta_scale(-1), center(false)
{
  string name = dir + "/" + baseName + ".txt";
  ifstream file(name.c_str());  
  if (!file.is_open()) {
    SVL_LOG(SVL_LOG_WARNING, "SWcacheInfo::Can't read from " << name);
    return;
  }
  file >> w >> h >> delta_x >> delta_y >> delta_scale 
       >> img_w >> img_h >> center;
  if (file.fail()) {
    SVL_LOG(SVL_LOG_WARNING, "SWcacheInfo::Can't read from " << name);
  }
}

bool SWcacheInfo::write(string dir, string baseName)
{
  string name = dir + "/" + baseName + ".txt";
  ofstream file(name.c_str());
  if (!file.is_open()) {
    SVL_LOG(SVL_LOG_WARNING, "SWcacheInfo::Can't write to " << name);
    return false;
  }
  file << w << " " << h << " " << delta_x << " " << delta_y << " " 
       << delta_scale << " " << img_w << " " << img_h << " " << center << endl;
  file.close();
  return true;
}



