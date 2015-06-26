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
** FILENAME:    svlDisjointSets.h
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
** DESCRIPTION:
**  Implements a forest of disjoint sets abstract data type. The elements are
**  numbered from 0 to (size() - 1). Each element belongs to exactly one set.
**  The sets have ID sparesly numbered in the range 0 to (size() - 1). To get
**  the number of elements of each set, call size(id) with a valid set ID.
**
*****************************************************************************/

#pragma once

#include <vector>

// svlDisjointSets class ----------------------------------------------------

class svlDisjointSets {
 protected:
    typedef struct _node_t {
        int rank;                   // depth of node
        int size;                   // size of set (for root nodes)
        int index;                  // index of the element represented by this node
        _node_t *parent;            // parent of this node
    } node_t;

    int _nElements;                 // number of elements in the forest
    int _nSets;                     // number of sets in the forest
    std::vector<node_t> _nodes;     // list of nodes

 public:
    // constructors and destructors
    svlDisjointSets(unsigned int count = 0);
    svlDisjointSets(const svlDisjointSets &dset);
    
    ~svlDisjointSets();
    
    // data type operations
    inline int size() const { return _nElements; }
    int size(int setId) const;        
    inline int sets() const { return _nSets; }
    std::set<int> getMembers(int setId);
    void add(int count);
    int find(int elementId);
    int join(int setId1, int setId2); // returns the joined set id (either setId1 or setId2)
    std::vector<int> getSetIds() const;

    svlDisjointSets& operator=(const svlDisjointSets& dset);
};
