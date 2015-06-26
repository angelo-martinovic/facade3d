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
** FILENAME:    segImageExtractFeatures.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
** DESCRIPTION:
**  Code for extracting appearance features from image segments.
**
*****************************************************************************/

#include <cstdlib>
#include <cassert>
#include <iostream>
#include <fstream>
#include <iomanip>

#include "svlBase.h"
#include "svlVision.h"

using namespace std;

#define WINDOW_NAME "segImageExtractFeatures"

void usage()
{
    cerr << SVL_USAGE_HEADER << "\n";
    cerr << "USAGE: ./segImageExtractFeatures [OPTIONS] <imageDir>\n";
    cerr << "OPTIONS:\n"
         << "  -o <directory>    :: output directory (default: current)\n"
         << "  -imgExt <ext>     :: image file extension (default: .jpg)\n"
         << "  -segExt <ext>     :: over-segmentation file extension (default: .seg)\n"
         << "  -lblExt <ext>     :: pixel labels file extension (default: .txt)\n"
         << "  -segDir <dir>     :: over-segmentation file directory (default: <imageDir>)\n"
         << "  -lblDir <dir>     :: pixel labels file directory (default: <imageDir>)\n"
         << "  -featuresExt <ext>:: output feature file extension (default: .features.txt)\n"
         << "  -segLblExt <ext>  :: segment labels file extension (default: .labels.txt)\n"
         << "  -noColor          :: exclude color features\n"
         << "  -noTexture        :: exclude texture features\n"
         << "  -noGeometry       :: exclude geometry features\n"
         << "  -noLocation       :: exclude image location features\n"
         << "  -noHighOrderStats :: exclude skewness and kurtosis statistics\n"
         << "  -includeNeighbors :: include statistics from neighborhood\n"
         << "  -includeBias      :: include bias feature (default: false)\n"
         << "  -skipLabels       :: skip extraction of labels\n"
         << "  -x                :: visualize feature extraction\n"
         << SVL_STANDARD_OPTIONS_USAGE;
    cerr << "<imageDir> must contain images <base>.jpg and <regionsDir> must contain\n"
         << "corresponding over-segmentations <base>.seg.\n"
         << endl;
}

int main(int argc, char *argv[])
{
    // read commandline parameters
    const char *outputDir = ".";
    const char *imgDir = NULL;
    const char *segDir = NULL;
    const char *lblDir = NULL;
    const char *imgExt = ".jpg";
    const char *segExt = ".seg";
    const char *lblExt = ".txt";
    const char *featuresExt = ".features.txt";
    const char *segLblExt = ".labels.txt";
    bool bIncludeNeighbors = false;
    bool bIncludeBias = false;
    bool bSkipLabels = false;
    bool bVisualize = false;

    svlRegionFeatures featureCalculator;

    SVL_BEGIN_CMDLINE_PROCESSING(argc, argv)
        SVL_CMDLINE_STR_OPTION("-o", outputDir)
        SVL_CMDLINE_STR_OPTION("-imgExt", imgExt)
        SVL_CMDLINE_STR_OPTION("-segExt", segExt)
        SVL_CMDLINE_STR_OPTION("-lblExt", lblExt)
        SVL_CMDLINE_STR_OPTION("-segDir", segDir)
        SVL_CMDLINE_STR_OPTION("-lblDir", lblDir)
        SVL_CMDLINE_STR_OPTION("-featuresExt", featuresExt)
        SVL_CMDLINE_STR_OPTION("-segLblExt", segLblExt)
        SVL_CMDLINE_FLAG_BEGIN("-noColor")
            featureCalculator.setOption("noColor", true);
        SVL_CMDLINE_FLAG_END
        SVL_CMDLINE_FLAG_BEGIN("-noTexture")
            featureCalculator.setOption("noTexture", true);
        SVL_CMDLINE_FLAG_END
        SVL_CMDLINE_FLAG_BEGIN("-noGeometry")
            featureCalculator.setOption("noGeometry", true);
        SVL_CMDLINE_FLAG_END
        SVL_CMDLINE_FLAG_BEGIN("-noLocation")
            featureCalculator.setOption("noLocation", true);
        SVL_CMDLINE_FLAG_END
        SVL_CMDLINE_FLAG_BEGIN("-noHighOrderStats")
            featureCalculator.setOption("includeSkewness", false);
            featureCalculator.setOption("includeKurtosis", false);
        SVL_CMDLINE_FLAG_END
        SVL_CMDLINE_BOOL_OPTION("-includeBias", bIncludeBias)
        SVL_CMDLINE_BOOL_OPTION("-x", bVisualize)
    SVL_END_CMDLINE_PROCESSING(usage());

    if (SVL_CMDLINE_ARGC != 1) {
        usage();
        return -1;
    }

    svlCodeProfiler::tic(svlCodeProfiler::getHandle("main"));

    imgDir = SVL_CMDLINE_ARGV[0];
    if (segDir == NULL) segDir = imgDir;
    if (lblDir == NULL) lblDir = imgDir;

    if (bVisualize) {
        cvNamedWindow(WINDOW_NAME, 1);
    }

    // load images
    SVL_LOG(SVL_LOG_MESSAGE, "Reading images from " << imgDir << "...");
    vector<string> baseNames = svlDirectoryListing(imgDir, imgExt, false, false);
    SVL_LOG(SVL_LOG_MESSAGE, "...read " << baseNames.size() << " images");

    // extract features
    int hImageLoad = svlCodeProfiler::getHandle("imageLoad");
    int hComputeFeatures = svlCodeProfiler::getHandle("computeImageFeatures");
    int hFeatureSave = svlCodeProfiler::getHandle("featureSave");
    int hLabelSave = svlCodeProfiler::getHandle("labelSave");
    
    SVL_LOG(SVL_LOG_MESSAGE, "Processing images...");
    for (int i = 0; i < (int)baseNames.size(); i++) {
        // instantiate svlSegImage
        SVL_LOG(SVL_LOG_VERBOSE, "Loading image " << baseNames[i] 
            << " (" << i << " of " << baseNames.size() << ")");
        string imgName = string(imgDir) + string("/") + baseNames[i] + string(imgExt);
        string segName = string(segDir) + string("/") + baseNames[i] + string(segExt);
        string lblName = string(lblDir) + string("/") + baseNames[i] + string(lblExt);

        svlCodeProfiler::tic(hImageLoad);
        svlSegImage img(imgName.c_str(), segName.c_str(),
            bSkipLabels ? NULL : lblName.c_str());
        svlCodeProfiler::toc(hImageLoad);
        SVL_LOG(SVL_LOG_VERBOSE, "..." << toString(*img.image()));

        // show segmentation
        if (bVisualize) {
            IplImage *debugImage = img.visualize();
            img.colorBoundaries(debugImage, 0xff, 0xff, 0xff);
            cvShowImage(WINDOW_NAME, debugImage);
            cvWaitKey(50);
            cvReleaseImage(&debugImage);
        }
        
        // compute features
        SVL_LOG(SVL_LOG_VERBOSE, "...computing features");
        svlCodeProfiler::tic(hComputeFeatures);
        vector<vector<double> > x = featureCalculator.computeFeatures(img);
        svlCodeProfiler::toc(hComputeFeatures);
        SVL_LOG(SVL_LOG_VERBOSE, "..." << x.size() << " feature vectors computed");
        
        // append neighborhood
        if (bIncludeNeighbors) {
            vector<pair<int, int> > adjList = img.getAdjacencyList();
            vector<double> counts(x.size(), 0.0);
            int numFeatures = (int)x[0].size();
            for (int i = 0; i < (int)x.size(); i++) {
                x[i].resize(2 * numFeatures, 0.0);
            }
            for (int i = 0; i < (int)adjList.size(); i++) {
                int segA = adjList[i].first;
                int segB = adjList[i].second;
                counts[segA] += 1.0;
                counts[segB] += 1.0;
                for (int j = 0; j < numFeatures; j++) {
                    x[segA][j + numFeatures] += x[segB][j];
                    x[segB][j + numFeatures] += x[segA][j];
                }
            }
            for (int i = 0; i < (int)x.size(); i++) {
                if (counts[i] == 0.0) continue;
                for (int j = 0; j < numFeatures; j++) {
                    x[i][j + numFeatures] /= counts[i];
                }
            }
        }

        // save features
        svlCodeProfiler::tic(hFeatureSave);
        string outputName = string(outputDir) + string("/") + baseNames[i] + string(featuresExt);
        ofstream ofs(outputName.c_str());
        assert(!ofs.fail());
        for (unsigned j = 0; j < x.size(); j++) {
            if (bIncludeBias) ofs << "1 ";
            for (unsigned k = 0; k < x[j].size(); k++) {
                if (k != 0) ofs << " ";
                ofs << x[j][k];
            }
            ofs << "\n";
        }
        ofs.close();
        svlCodeProfiler::toc(hFeatureSave);

        // save labels
        if (!bSkipLabels) {
            svlCodeProfiler::tic(hLabelSave);
            string outputName = string(outputDir) + string("/") + baseNames[i] + string(segLblExt);
            ofstream ofs(outputName.c_str());
            assert(!ofs.fail());
            for (int i = 0; i < img.numSegments(); i++) {
                ofs << img.getLabel(i) << "\n";
            }
            ofs.close();
            svlCodeProfiler::toc(hLabelSave);
        }        
    }
    SVL_LOG(SVL_LOG_MESSAGE, "...done");
    
    // free memory
    cvDestroyAllWindows();
    svlCodeProfiler::toc(svlCodeProfiler::getHandle("main"));
    svlCodeProfiler::print(cerr);
    return 0;
}
