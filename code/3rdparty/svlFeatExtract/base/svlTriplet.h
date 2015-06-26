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
** FILENAME:    svlTriplet.h
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
** DESCRIPTION:
**  Basic datatype for holding three objects of the same type. Similar to the
**  STL pair<> class.
**
*****************************************************************************/

#pragma once

#include <ctime>
#include <vector>
#include <map>
#include <string>
#include <iostream>

// svlTriplet class ---------------------------------------------------------

template<class T>
class svlTriplet {
 public:
    T first;
    T second;
    T third;

 public:
    inline svlTriplet() { }
    inline svlTriplet(const T& i, const T& j, const T& k) :
        first(i), second(j), third(k) { }
    inline svlTriplet(const svlTriplet<T>& t) :
        first(t.first), second(t.second), third(t.third) { }
    inline ~svlTriplet() { }

    // operators
    inline svlTriplet<T>& operator=(const svlTriplet<T>& t);
    inline bool operator==(const svlTriplet<T>& t);
    inline bool operator<(const svlTriplet<T>& t);
};

// implementation -----------------------------------------------------------

template<class T>
inline svlTriplet<T>& svlTriplet<T>::operator=(const svlTriplet<T>& t) {
    first = t.first;
    second = t.second;
    third = t.third;

    return *this;
}

template<class T>
inline bool svlTriplet<T>::operator==(const svlTriplet<T>& t) {
    return ((t.first == first) && (t.second == second) && (t.third == third));
}

template<class T>
inline bool svlTriplet<T>::operator<(const svlTriplet<T>& t) {
    return ((first < t.first) || (second < t.second) || (third < t.third));
}
