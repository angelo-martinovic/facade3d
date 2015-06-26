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
** FILENAME:    svlFileUtils.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
**
*****************************************************************************/

#include <cassert>
#include <string>
#include <algorithm>
#include <sys/types.h>
#include <sys/stat.h>

#if defined(_WIN32)||defined(WIN32)||defined(__WIN32__)
#include "win32/dirent.h"
#include <windows.h>
#else
#include <dirent.h>
#include <unistd.h>
#endif

#include "svlLogger.h"
#include "svlFileUtils.h"

using namespace std;

// build sorted list of filenames
vector<string> svlDirectoryListing(const char *directory,
    const char *extension, bool bIncludeDir, bool bIncludeExt)
{
    assert(directory != NULL);

    vector<string> filenames;
    DIR *dir = opendir(directory);
    if (dir == NULL) {
        SVL_LOG(SVL_LOG_FATAL, "could not open directory " << directory);
    }

    string prefix;
    if (bIncludeDir) {
        prefix = string(directory) + string("/");
    }
    
    int extLength = 0;
    if (extension != NULL) {
        extLength = strlen(extension);
    }

    struct dirent *e = readdir(dir);
    while (e != NULL) {
        // skip . and ..
        if (!strncmp(e->d_name, ".", 1) || !(strncmp(e->d_name, "..", 2))) {
            e = readdir(dir);
            continue;
        }
            
        bool bIncludeFile = false;
        if (extension == NULL) {
            bIncludeFile = true;
        } else {
            const char *p = strstr(e->d_name, extension);
            if ((p != NULL) && (*(p + strlen(extension)) == '\0')) {
                bIncludeFile = true;
            }
        }

        if (bIncludeFile) {
            string filename = string(e->d_name);
            if (!bIncludeExt) {
                filename.erase(filename.length() - extLength);
            }
            filenames.push_back(prefix + filename);
        }
            
        e = readdir(dir);
    }    
    closedir(dir);

    sort(filenames.begin(), filenames.end());

    return filenames;
}

// create directory
bool svlCreateDirectory(const char *directoryPath)
{
#if defined(_WIN32)||defined(WIN32)||defined(__WIN32__)
    bool success = CreateDirectory(directoryPath, NULL);
#else
    bool success = (mkdir(directoryPath, 0777) == 0);
#endif

    if (!success) {
        SVL_LOG(SVL_LOG_WARNING, "failed to create directory " << directoryPath);
    }

    return success;
}

// count fields
int svlCountFields(ifstream *ifs, char delimiter, bool bSkipRepeated)
{
    SVL_ASSERT(ifs != NULL);

    int numFields = 1;

    unsigned p = ifs->tellg();
    char ch;
    bool lastCharDelimiter = true;
    while (!ifs->eof() && (ifs->peek() != '\n')) {
        ifs->read(&ch, 1);
        if (ch == delimiter) {
            if (!bSkipRepeated || !lastCharDelimiter)
                numFields += 1;
            lastCharDelimiter = true;
        } else {
            lastCharDelimiter = false;
        }
    }
    ifs->seekg(p, ios::beg);

    return numFields;
}

// read strings from a file
vector<string> svlReadFile(const char *filename)
{
    ifstream ifs(filename);
    SVL_ASSERT(!ifs.fail());

    vector<string> fileLines;
    while (!ifs.eof()) {
        string str;
        ifs >> str;
        if (ifs.fail()) break;
        if (str.empty()) continue;
        fileLines.push_back(str);
    }    
    ifs.close();

    return fileLines;
}

// check for file existance
bool svlFileExists(const char *filename)
{
#if defined(_WIN32)||defined(WIN32)||defined(__WIN32__)
    FILE *f = fopen(filename, "r");
    if (f == NULL) return false;

    fclose(f);
    return true;
#else
    return (access(filename, R_OK) == 0);
#endif
}
