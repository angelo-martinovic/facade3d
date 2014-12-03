#pragma once

#include "cxcore.h"
#include "vector"
//#include "mex.h"


#define uint8 unsigned char

typedef CvPoint2D64f Point;

//
struct LineHough{
  double angle;
  double R;
  double confidence;
};

struct SortByAngle
{
    bool operator()(const LineHough & c1, const LineHough & c2)
    { return c2.angle < c1.angle; }
};


//
template<class T> class Image
{
  private:
  IplImage* imgp;
  public:
  Image(IplImage* img=0) {imgp=img;}
  ~Image(){imgp=0;}
  void operator=(IplImage* img) {imgp=img;}
  inline T* operator[](const int rowIndx) {
    return ((T *)(imgp->imageData + rowIndx*imgp->widthStep));}
};


//
void saveLines(CvSeq* lines, IplImage* canny_im, const char *in_cLinesFileName);

//
void printLines(CvSeq* lines, const char *in_cLinesTxtFileName);

//
void drawLines(CvSeq* lines, IplImage* canny_im, float R, float G, float B);

//
void drawLines(const std::vector<LineHough> & lines, IplImage* canny_im, float R, float G, float B);

//
void loadLines(const char *in_cLinesTxtFileName, CvSeq** lines, CvMemStorage* storage);

//
void loadLinesHough(const char *in_cLinesTxtFileName, std::vector<LineHough> &lines);

//
//void getLinesFromMxMatrix(const mxArray *in_mxLines, std::vector<LineHough> &lines);

//
void loadPoints(const char *in_cLinesTxtFileName, std::vector<Point> &points);

//
//void getPointsFromMxMatrix(const mxArray *in_mxPoints, std::vector<Point> &points);

//
void printPoints(std::vector<Point> &points, const char *in_cLinesTxtFileName);

//
void printVector(std::vector<int> &in_Vector, const char *in_cLinesTxtFileName);

//
double Determinant(double **a,int n);

//
void PlaneFrom3Points(std::vector <std::vector <double> > &in_vecPointCoords, double &out_dA, double &out_dB, double &out_dC);

//
double AngleBetween2Planes(double A1, double B1, double C1, double A2, double B2, double C2);

