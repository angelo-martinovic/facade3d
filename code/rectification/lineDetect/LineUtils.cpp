#include "LineUtils.h"

#include "cv.h"
#include "highgui.h"
#include "fstream"
#include "vector"
#include "algorithm"
#include <iostream>
//#include "mex.h"



/*************************************************************************************************************/
void drawLines(CvSeq* lines, IplImage* canny_im, float R, float G, float B)
{
  if (lines == 0)
  {
    return;
  }

  for (int i = 0; i < lines->total; i++ )
  {
    float* line = (float*)cvGetSeqElem(lines,i);
    float rho = line[0];
    float theta = line[1];

    CvPoint pt1, pt2;

    float a = cos(theta), b = sin(theta);
    float x0 = a*rho, y0 = b*rho;

    pt1.x = cvRound(x0 + 1000*(-b));
    pt1.y = cvRound(y0 + 1000*(a));
    pt2.x = cvRound(x0 - 1000*(-b));
    pt2.y = cvRound(y0 - 1000*(a));

    cvLine( canny_im, pt1, pt2, CV_RGB(R,G,B), 1, 8 );
  }
}

void drawLines(const std::vector<LineHough> & lines, IplImage* canny_im, float R, float G, float B)
{
  if (lines.size() == 0)
  {
    return;
  }
  for (size_t i = 0; i < lines.size(); i++ )
  {
    float rho = lines[i].R;
    float theta = lines[i].angle;

    CvPoint pt1, pt2;

    float a = cos(theta), b = sin(theta);
    float x0 = a*rho, y0 = b*rho;

    pt1.x = cvRound(x0 + 1000*(-b));
    pt1.y = cvRound(y0 + 1000*(a));
    pt2.x = cvRound(x0 - 1000*(-b));
    pt2.y = cvRound(y0 - 1000*(a));

    cvLine( canny_im, pt1, pt2, CV_RGB(R,G,B), 1, 8 );
  }
}

/*************************************************************************************************************/
void saveLines(CvSeq* lines, IplImage* canny_im, const char *in_cLinesFileName)
{
  IplImage* color_dst = cvCreateImage( cvGetSize(canny_im), 8, 3 );

  if (canny_im->nChannels == 1)
  {
    cvCvtColor( canny_im, color_dst, CV_GRAY2BGR );
  }
  else
  {
    color_dst = cvCloneImage(canny_im);
  }

  drawLines(lines, color_dst, 255, 0, 0);

  cvSaveImage(in_cLinesFileName, color_dst);

  cvReleaseImage(&color_dst);
}

/*************************************************************************************************************/
void printLines(CvSeq* lines, const char *in_cLinesTxtFileName)
{

  std::ofstream out_file(in_cLinesTxtFileName);
  for (int i = 0; i < lines->total; i++ )
  {
    float* line = (float*)cvGetSeqElem(lines,i);
    float rho = line[0];
    float theta = line[1];

    out_file << rho << '\t' << theta << '\n';
  }
  out_file.close();
}


/*************************************************************************************************************/
void loadLines(const char *in_cLinesTxtFileName, CvSeq** lines, CvMemStorage* storage)
{
  *lines = cvCreateSeq( CV_32FC2, /* sequence of integer elements */
    sizeof(CvSeq), /* header size - no extra fields */
    sizeof(float)*2, /* element size */
    storage /* the container storage */ );
  std::ifstream in_file(in_cLinesTxtFileName);

  while (!in_file.eof())
  {
    float rho, theta;

    in_file >> rho;
    in_file >> theta;

    float line [2];
    line[0] = rho;
    line[1] = theta;

    cvSeqPush( *lines, line );

  }
  in_file.close();
}


/*************************************************************************************************************/
void loadLinesHough(const char *in_cLinesTxtFileName, std::vector<LineHough> &lines)
{
  std::ifstream in_file(in_cLinesTxtFileName);

  int counter = 0;

  //while (!in_file.eof() && counter < 10)// DEBUG
  while (!in_file.eof() && counter < 10000)
  {
    LineHough curLine;
    in_file >> curLine.angle;
    in_file >> curLine.R;
    in_file >> curLine.confidence;

    lines.push_back(curLine);

    counter ++;
  }
  in_file.close();
}



/*************************************************************************************************************/
//void getLinesFromMxMatrix(const  mxArray *in_mxLines, std::vector<LineHough> &lines)
//{
//  double* mat;
//  mat = mxGetPr(in_mxLines);
//
//  size_t nLines = mxGetM(in_mxLines);
//  lines.resize(nLines);
//
//  for (size_t iLine = 0; iLine < nLines; iLine++)
//  {	 
//    lines[iLine].angle = mat[nLines*0 + iLine];
//    lines[iLine].R = mat[nLines*1 + iLine];
//    lines[iLine].confidence = mat[nLines*2 + iLine];
//  }    
//}



/*************************************************************************************************************/
void loadPoints(const char *in_cLinesTxtFileName, std::vector<Point> &points)
{
  std::ifstream in_file(in_cLinesTxtFileName);

  while (!in_file.eof())
  {
    Point curPoint;
    in_file >> curPoint.x;
    in_file >> curPoint.y;

    points.push_back(curPoint);
  }
  in_file.close();
}


/*************************************************************************************************************/
//void getPointsFromMxMatrix(const mxArray *in_mxPoints, std::vector<Point> &points)
//{
//  double* mat;
//  mat = mxGetPr(in_mxPoints);
//
//  size_t nPoints = mxGetM(in_mxPoints);
//  points.resize(nPoints);
//
//  for (size_t iPoint = 0; iPoint < nPoints; iPoint++)
//  {	 
//    points[iPoint].x = mat[nPoints*0 + iPoint];
//    points[iPoint].y = mat[nPoints*1 + iPoint];
//  }  
//}


/*************************************************************************************************************/
void printPoints(std::vector<Point> &points, const char *in_cLinesTxtFileName)
{
  std::ofstream out_file(in_cLinesTxtFileName);
  for (size_t i = 0; i < points.size(); i++ )
  {
    out_file << points[i].x << '\t' << points[i].y << '\n';
  }
  out_file.close();
}


/*************************************************************************************************************/
void printVector(std::vector<int> &in_Vector, const char *in_cLinesTxtFileName)
{
  std::ofstream out_file(in_cLinesTxtFileName);
  for (size_t i = 0; i < in_Vector.size(); i++ )
  {
    out_file << in_Vector[i] << '\n';
  }
  out_file.close();
}


/*************************************************************************************************************/
// Recursive definition of determinate using expansion by minors.
double Determinant(double **a,int n)
{
  int i,j,j1,j2;
  double det = 0;
  double **m = NULL;

  if (n < 1) { /* Error */

  } else if (n == 1) { /* Shouldn't get used */
    det = a[0][0];
  } else if (n == 2) {
    det = a[0][0] * a[1][1] - a[1][0] * a[0][1];
  } else {
    det = 0;
    for (j1=0;j1<n;j1++) {
      m = (double **)malloc((n-1)*sizeof(double *));
      for (i=0;i<n-1;i++)
        m[i] = (double *)malloc((n-1)*sizeof(double));
      for (i=1;i<n;i++) {
        j2 = 0;
        for (j=0;j<n;j++) {
          if (j == j1)
            continue;
          m[i-1][j2] = a[i][j];
          j2++;
        }
      }
      det += pow(-1.0,1.0+j1+1.0) * a[0][j1] * Determinant(m,n-1);
      for (i=0;i<n-1;i++)
        free(m[i]);
      free(m);
    }
  }
  return(det);
}


/*************************************************************************************************************/
void PlaneFrom3Points(std::vector <std::vector <double> > &in_vecPointCoords, double &out_dA, double &out_dB, double &out_dC)
{
  double **matrix;
  matrix = new double*[3];
  for (int i = 0; i < 3; i ++)
  {
    matrix[i] = new double[3];
  }
  for (int i = 0; i < 3; i ++)
  {
    for (int j = 0; j < 3; j ++)
    {
      matrix[i][j] = in_vecPointCoords[i][j];
    }
  }
  double D = Determinant(matrix, 3);

  for (int i = 0; i < 3; i ++)
  {
    for (int j = 0; j < 3; j ++)
    {
      matrix[i][j] = in_vecPointCoords[i][j];
    }
    matrix[i][0] = 1.0;
  }
  double A = Determinant(matrix, 3);
  out_dA = -A/D;


  for (int i = 0; i < 3; i ++)
  {    
    for (int j = 0; j < 3; j ++)
    {
      matrix[i][j] = in_vecPointCoords[i][j];
    }
    matrix[i][1] = 1.0;
  }
  double B = Determinant(matrix, 3);
  out_dB = -B/D;


  for (int i = 0; i < 3; i ++)
  {    
    for (int j = 0; j < 3; j ++)
    {
      matrix[i][j] = in_vecPointCoords[i][j];
    }
    matrix[i][2] = 1.0;
  }
  double C = Determinant(matrix, 3);
  out_dC = -C/D;

  for (int i = 0; i < 3; i ++)
  {
    delete[] matrix[i];
  }
  delete[] matrix;
}


/*************************************************************************************************************/
double AngleBetween2Planes(double A1, double B1, double C1, double A2, double B2, double C2)
{
  double top = A1*A2 + B1*B2 + C1*C2;

  double L1 = sqrt(A1*A1 + B1*B1 + C1*C1);
  double L2 = sqrt(A2*A2 + B2*B2 + C2*C2);

  double angle = acos(top/(L1*L2));

  return angle;
}
