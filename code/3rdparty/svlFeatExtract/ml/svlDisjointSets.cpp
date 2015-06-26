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
** FILENAME:    svlDisjointSets.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
**
*****************************************************************************/

#include <cassert>
#include <iostream>
#include <fstream>
#include <limits>
#include <cmath>

#include "svlBase.h"
#include "svlDisjointSets.h"

using namespace std;

// svlDisjointSets class ----------------------------------------------------

svlDisjointSets::svlDisjointSets(unsigned int count) :
    _nElements(0), _nSets(0)
{
    add(count);   
}
        
svlDisjointSets::svlDisjointSets(const svlDisjointSets &dset) :
    _nElements(dset._nElements), _nSets(dset._nSets)
{
    // copy nodes
    _nodes.resize(_nElements);
    for (int i = 0; i < _nElements; i++) {
        _nodes[i] = dset._nodes[i];
    }
    
    // update parent pointers
    for (int i = 0; i < _nElements; i++) {
        if (_nodes[i].parent != NULL) {
            _nodes[i].parent = &_nodes[dset._nodes[i].parent->index];
        }
    }
}

svlDisjointSets::~svlDisjointSets()
{
    //_nodes.clear();
}

int svlDisjointSets::size(int setId) const
{
    SVL_ASSERT(setId < _nElements);
    SVL_ASSERT(_nodes[setId].parent == NULL);
    
    return _nodes[setId].size;
}

set<int> svlDisjointSets::getMembers(int setId)
{
    set<int> members;
    for (int i = 0; i < _nElements; i++) {
        if (find(i) == setId)
            members.insert(i);
    }

    return members;
}

void svlDisjointSets::add(int count)
{
    _nodes.resize(_nElements + count);
    for (int i = 0; i < count; i++) {
        _nodes[_nElements + i].index = _nElements + i;
        _nodes[_nElements + i].size = 1;
        _nodes[_nElements + i].rank = 0;
        _nodes[_nElements + i].parent = NULL;
    }

    _nElements += count;
    _nSets += count;
}

int svlDisjointSets::find(int elementId)
{
    SVL_ASSERT(elementId < _nElements); 
    
    node_t *root;
    node_t *ptrNode, *nextNode;
    
    // find the root node for elementId
    root = &_nodes[elementId];
    while (root->parent != NULL) {
        root = root->parent;
    }
    
#if 1
    // optimize for future find() operations by making all descendents of
    // the root node direct children of the root node
    ptrNode = &_nodes[elementId];
    while (ptrNode != root) {
        nextNode = ptrNode->parent;
        ptrNode->parent = root;
        ptrNode = nextNode;
    }
#else
    // optimize for future find() operations by making search node a direct
    // child of the root node
    if (&_nodes[elementId] != root) {
        _nodes[elementId].parent = root;   
    }    
#endif

    return (root->index);
}

int svlDisjointSets::join(int setId1, int setId2)
{
    if (setId1 == setId2)
        return setId1;

    SVL_ASSERT((setId1 < _nElements) && (setId2 < _nElements));
    node_t *set1, *set2;
    
    set1 = &_nodes[setId1];
    set2 = &_nodes[setId2];
    SVL_ASSERT((set1->parent == NULL) && (set2->parent == NULL));

    // balance tree by joining based on rank
    int joinedSetId;
    if (set1->rank > set2->rank) {
        set2->parent = set1;
        set1->size += set2->size;
        joinedSetId = setId1;
    } else {
        set1->parent = set2;
        if (set1->rank == set2->rank) {
            set2->rank += 1;
        }
        set2->size += set1->size;
        joinedSetId = setId2;
    }
    
    _nSets -= 1;
    return joinedSetId;
}

vector<int> svlDisjointSets::getSetIds() const
{
    vector<int> setIds;
    setIds.reserve(_nSets);

    for (int i = 0; i < _nElements; i++) {
        if (_nodes[i].parent == NULL) {
            setIds.push_back(i);
        }
    }
    
    return setIds;       
}

svlDisjointSets& svlDisjointSets::operator=(const svlDisjointSets& dset)
{
    _nElements = dset._nElements;
    _nSets = dset._nSets;

    // copy nodes
    _nodes = dset._nodes;
    
    // update parent pointers
    for (int i = 0; i < _nElements; i++) {
        if (_nodes[i].parent != NULL) {
            _nodes[i].parent = &_nodes[dset._nodes[i].parent->index];
        }
    }

    return *this;
}
