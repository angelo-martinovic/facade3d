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
** FILENAME:    svlExtensionTree.h
** AUTHOR(S):   Ian Goodfellow <ia3n@cs.stanford.edu>
** DESCRIPTION:
**  A class for managing different file extensions when saving and loading
**  images for use with the patch based object detector.
**  When using multiple intensity channels, we can no longer just scan a
**  directory for all files ending in the same extension. if one channel
**  ends in ".jpg" and another in ".processed.jpg", for example, the first
**  channel would end up with too many images. This tree class keeps track
**  of the subsequence relationships between the file extensions and loads
**  directories appropriately.
**
*****************************************************************************/

#pragma once

#include <vector>
#include <string>
using namespace std;

class svlExtensionTreeNode
{
  friend class svlExtensionTree;

 protected:
  svlExtensionTreeNode() {} //Constructor is protected so only an extension tree can make a tree node
  ~svlExtensionTreeNode();

  svlExtensionTreeNode(const svlExtensionTreeNode & o);

  const svlExtensionTreeNode * findChannel(unsigned channelId) const;
  void addChannel(unsigned channelId, string ext);
  void readDirectory(const char * dir, const vector<string> & ignore);
  void trickleDown(vector<string> & _parentFiles);
  bool findChildExtEndsWith(const string & ext, unsigned & output) const; //Returns true if successfully found a child that ext ends with, ie. the following would return true: endsWith(ext, _children[output]->_ext)
  bool findChildThatEndsWith(const string & ext, unsigned & output) const; //Returns true if successfuly found a child such that the following would return true: endsWith(_children[output]->_ext, ext)

  //String functions for improving readability
  static bool endsWith(const string & str, const string & other); //Returns true iff str ends with other
  static bool matches(const string & str, const string & other);

  //Vector functions for improving readability
  static bool contains(const vector<unsigned> & v, unsigned i);//binary searches sorted vector v for i
  static void insert(vector<unsigned> & v, unsigned i); //inserts i into v, maintaining sorted order

 protected:
  string _ext; //The extension shared by all channels at this node
  vector<unsigned> _channels; //A list of all channels sharing this extension. sorted, ascending order
  vector<svlExtensionTreeNode *> _children; //if we wanted to be purists we would maintain this in sorted order based on each node's extension string flipped backwards (ie, "gpj.")
  //each child satisfies these conditions:
  // child.ext ends in ext
  // child.ext is longer than ext
  // for all nodes other than this and child, it is not the case that child.ext ends in other.ext and other.ext ends in ext
  vector<string> _files; //A list of files from the last readDirectory call that have this node's extension TODO-- maybe we should switch this vector to something backed by a linked list so the removal operations in trickleDown are O(1) instead of O(N)


};

class svlExtensionTree
{
 public:

  svlExtensionTree();
  void addChannel(unsigned channelId, string ext);
  bool containsChannel(unsigned channelId) const; //warning-- uses DFS recursion, so if you have an absurdly large number of channels with nesting names this could overflow the stack. could be rewritten to use an iterator.
  void readDirectory(const char * dir);
  void getFilesForChannel(unsigned channelId, vector<string> & output) const;
  void ignore(const string & str);


 protected:
  svlExtensionTreeNode _root; 
  vector<string>  _ignore; //exclude these suffixes when reading directories
};



