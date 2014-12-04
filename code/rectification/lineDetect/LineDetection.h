#include <math.h>
#include <stdio.h>

#include <cv.h>
#include <highgui.h>
#include <vector>

#include "LineUtils.h"

using namespace std;

//
vector<LineHough> DetectLines(IplImage *edgeImage, double C, double minLambda, int maxLines, const char *saveFirst = NULL);

//
vector<LineHough> DetectLinesWithGradient(IplImage *edgeImage, IplImage *dxImage,  IplImage *dyImage,
                                          float in_fLineBGBias, float in_fLineHypPenalty, 
                                          float in_fEdgelToLineDistCoef, float in_fAngleBetweenGradientAndNormalCoef,
                                          int maxLines, const char *saveFirst);

//
void DumpToFile(vector<LineHough> lines, const char *filename);