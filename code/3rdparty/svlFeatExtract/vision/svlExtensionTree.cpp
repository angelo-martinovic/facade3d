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
** FILENAME:    svlExtensionTree.cpp
** AUTHOR(S):   Ian Goodfellow <ia3n@cs.stanford.edu>
**
*****************************************************************************/

#include <algorithm>
#include <assert.h>
using namespace std;

#include "svlBase.h"

#include "svlExtensionTree.h"

//svlExtensionTree implementation

svlExtensionTree::svlExtensionTree()
{
  _root._ext = "";//just to be explicit
}

void svlExtensionTree::addChannel(unsigned channelId, string ext)
{
  SVL_ASSERT(! containsChannel(channelId));

  _root.addChannel(channelId, ext);
}

bool svlExtensionTree::containsChannel(unsigned channelId) const
{
  return _root.findChannel(channelId) != NULL;
}

void svlExtensionTree::readDirectory(const char * dir)
{
  _root.readDirectory(dir, _ignore);
}

void svlExtensionTree::getFilesForChannel(unsigned channelId, vector<string> & output) const
{
  const svlExtensionTreeNode * node = _root.findChannel(channelId);

  SVL_ASSERT(node);

  output = node->_files;
}


//svlExtensionTreeNode implementation

svlExtensionTreeNode::~svlExtensionTreeNode()
{
  //delete children (sending them to college implements the same functionality
  //but is much more expensive)
  for (unsigned i = 0; i < _children.size(); i++)
    delete _children[i];
}

const svlExtensionTreeNode * svlExtensionTreeNode::findChannel(unsigned channelId) const
{
  if (contains(_channels, channelId))
    return this;

  for (unsigned i = 0; i < _children.size(); i++)
    {
      const svlExtensionTreeNode * childRes = _children[i]->findChannel(channelId);

      if (childRes)
	return childRes;
    }

  return NULL;
}

void svlExtensionTreeNode::addChannel(unsigned channelId, string ext)
{
  //The tree should already have checked that the tree does not contain
  //channelId, so we don't need to check this subtree

  SVL_ASSERT(endsWith(ext, _ext)); //Make sure this extension is compatible with this subtree

  if (matches(ext, _ext)) //This channel goes in this node
      insert(_channels, channelId);
  else
    {
      //We must find a place for this node in our subtree, or make one

      unsigned idx = 0;

      //If the new extension ends with some child node's extension,
      //then we make like the Bush administration and pass the burden
      //on to our children
      if (findChildExtEndsWith(ext, idx))
	_children[idx]->addChannel(channelId, ext);
      //Otherwise, if some child ends with this extension, then we make
      //an intermediate node for the extension
      else if (findChildThatEndsWith(ext,idx))
	{
	  //Save the link to the current child
	  svlExtensionTreeNode * temp = _children[idx];

	  //Make the new child
	  _children[idx] = new svlExtensionTreeNode;

	  //Link the new child to the former one
	  _children[idx]->_children.push_back(temp);

	  //Set the child's extension
	  _children[idx]->_ext = ext;

	  //Pass in the channel to be added
	  _children[idx]->addChannel(channelId, ext);
	}
      else
	{
	  //This extension is not along the same string trajectory as any
	  //of our children so we give it a new leaf node

	  _children.push_back(new svlExtensionTreeNode);
	  _children.back()->_ext = ext;
	  _children.back()->addChannel(channelId, ext);
	}
    }
}

void svlExtensionTreeNode::readDirectory(const char * dir, const vector<string> & ignore)
{
  if (!_channels.size())
    {
      //If no channels live in you, you are definitely root
      //It also means no one was silly enough to use the extension ""
      //(and thus populate root)
     
      //If we did not special case this, we would always read the entire
      //contents of the directory, then use trickleDown on that entire set
      //It would work, it would just be inefficient

      //Instead, let's just read those filenames that actually match a populated
      //child

      for (unsigned i = 0; i < _children.size(); i++)
	_children[i]->readDirectory(dir, ignore);
    }
  else
    {
      //Read in all files ending with our extension
      _files = svlDirectoryListing(dir, _ext.c_str(), false, true);

      vector<string>::iterator iter = _files.begin();

      //remove files on the ignore list
      while (iter != _files.end())
	{
	  bool retained = true;
	  for (unsigned i = 0; i < ignore.size(); i++)
	    {
	      if (endsWith(*iter, ignore[i]))
		{
		  retained = false;
		  iter = _files.erase(iter);
		  break;
		}
	    }
	  if (retained)
	    ++iter;
	}
      
      //Now let our children, who have longer names, remove the files that
      //are not actually ours
      for (unsigned i = 0; i < _children.size(); i++)
	_children[i]->trickleDown(_files);

      //todo-- should we sort the files here?
    }
}

void svlExtensionTreeNode::trickleDown(vector<string> & parentFiles)
{
  _files.clear();

  for (vector<string>::iterator i = parentFiles.begin(); i != parentFiles.end(); ++i)
    {
      if (endsWith(*i, _ext))
	{
	  _files.push_back(*i);
	  parentFiles.erase(i);
	}

      if (i == parentFiles.end())
	break;
    }

  for (unsigned i = 0; i < _children.size(); i++)
    _children[i]->trickleDown(_files);

  //todo-- should we sort the files here?
}

bool svlExtensionTreeNode::findChildExtEndsWith(const string & ext, unsigned & output) const //Returns true if successfully found a child that ext ends with, ie. the following would return true: endsWith(ext, _children[output]->_ext)
{
  //todo-- if we wanted to be purists we'd sort the children by the backwards
  //version of their extension and do a binary search here
  //but since we're not likely to have more than 2 or 3 strings yet I'm just
  //going to brute force itq

  for (unsigned i = 0; i < _children.size(); i++)
    if (endsWith(ext, _children[i]->_ext))
      {
	output = i; //note that this result is unique. if two
	//nodes end in the same subsequence they should either
	//be the same node or the longer one should be the child
	//of the shorter one
	return true;
	}

  return false;
}

bool svlExtensionTreeNode::findChildThatEndsWith(const string & ext, unsigned & output) const //Returns true if successfuly found a child such that the following would return true: endsWith(_children[output]->_ext, ext)
{
  //todo-- if we decide to sort _children, this section could benefit from the speedup too

  for (unsigned i = 0; i < _children.size(); i++)
    if (endsWith(_children[i]->_ext, ext))
      {
	output = i; //note that this result is unique. if two
	//nodes end in the same subsequence they should either
	//be the same node or the longer one should be the child
	//of the shorter one
	return true;
      }

  return false;
}

bool svlExtensionTreeNode::endsWith(const string & str, const string & other) //Returns true iff str ends with other
{
  if (str.size() < other.size())
    return false;

  size_t begin = str.size() - other.size();

  string lastPart = str.substr(begin);

  bool result = !strcmp( lastPart.c_str(), other.c_str());

  return result;
}

bool svlExtensionTreeNode::matches(const string & str, const string & other)
{
  return ! strcmp(str.c_str(), other.c_str());
}

bool svlExtensionTreeNode::contains(const vector<unsigned> & v, unsigned i) 
{
  return binary_search(v.begin(), v.end(), i);
}

void svlExtensionTreeNode:: insert(vector<unsigned> & v, unsigned i) 
{
  vector<unsigned>::iterator iter = lower_bound(v.begin(), v.end(), i);
  v.insert(iter, i);
}

void svlExtensionTree::ignore(const string & str)
{
  _ignore.push_back(str);
}

svlExtensionTreeNode::svlExtensionTreeNode(const svlExtensionTreeNode & o) : _ext(o._ext), _channels(o._channels), _files(o._files)
{
  for (unsigned i = 0; i < o._children.size(); i++)
    _children.push_back(new svlExtensionTreeNode(*(o._children[i])));
}
