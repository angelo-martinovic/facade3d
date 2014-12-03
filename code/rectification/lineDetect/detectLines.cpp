#include<stdlib.h>
#include<string.h>
#include<math.h>
#include<assert.h>
#include "LineDetection.h"

#ifdef _OPENMP
#include <omp.h>
#else
//#warning Not using OpenMP
#endif

#include <iostream>
#include <math.h>
#include <stdlib.h>

#include <unistd.h>

//#include <io.h>

#include <vector>
#include <list>
#include <sstream>

#include <cv.h>
#include <highgui.h>
#include <vector>

#define uint8 unsigned char
#define WHITE 255
#define BLACK 0

#define CANNY_THRESH 700
#define CANNY_MASK   5




/*************************************************************************************************************/
inline uint8 getColor(const uint8* img, int imgHeight, int x, int y)
{
  return img[x*imgHeight+y]; 
}


/*************************************************************************************************************/
inline void setColor(uint8* img, int imgHeight, int x, int y, uint8 color)
{
  img[x*imgHeight+y] = color; 
}


/*************************************************************************************************************/
int main(int argc, char *argv[]){
//void mexFunction(int nout, mxArray *out[], 
 //           int nin, const mxArray *in[])
  
  if (argc < 3) {
	  std::cout << "usage: ./detectLines <image> <linesfile> [<drawImage>]\n";
	  return 0;
  } 

  enum {img_i=0, LineBGBias_i, LineHypPenalty_i, EdgelToLineDistCoef_i, AngleBetweenGradientAndNormalCoef_i, maxLines_i} ;
  enum {lines_i=0, imgEdge_i, imgEdgeMap_i} ;

  /* ------------------------------------------------------------------
  **                                                Check the arguments
  ** --------------------------------------------------------------- */ 
  float maxLines = 300; // maximum number of lines
  float distThresh = 5;
  if (maxLines <= 0) {
	  std::cerr << "Maximum number of lines should be positive\n";
  } 

  float fLineBGBias = -log(1e-2); 
  if (fLineBGBias <= 0) {
	  std::cerr << "Background bias parameter should be positive\n";
  } 
	float fLineHypPenalty = 42 * fLineBGBias;
	float fEdgelToLineDistCoef = 0.6 * fLineBGBias;
	float fAngleBetweenGradientAndNormalCoef = 3 * fLineBGBias;


  IplImage * image = cvLoadImage(argv[1]);
  IplImage * iplImg= cvCreateImage (cvGetSize(image), 8, 1);
  cvCvtColor(image, iplImg, CV_BGR2GRAY);


  //get edge map
  IplImage* iplEdge=cvCreateImage(cvSize(iplImg->width, iplImg->height),IPL_DEPTH_8U,1);
  cvCanny(iplImg,iplEdge,CANNY_THRESH,CANNY_THRESH*2,CANNY_MASK); 

  //get smoothed image
  IplImage *imSmooth = cvCloneImage(iplImg);
  cvSmooth(iplImg, imSmooth, CV_GAUSSIAN, 3);

  //get derivatives of the image along horizontal and vertical axes
  IplImage *xder16S = cvCreateImage(cvSize(iplImg->width, iplImg->height), IPL_DEPTH_16S, 1);
  IplImage *yder16S = cvCreateImage(cvSize(iplImg->width, iplImg->height), IPL_DEPTH_16S, 1);

  cvSobel( imSmooth, xder16S, 1, 0); // dI/dx
  cvSobel( imSmooth, yder16S, 0, 1); // dI/dy

  // find lines
  std::vector<LineHough> lines = DetectLinesWithGradient(iplEdge, xder16S, yder16S, fLineBGBias, fLineHypPenalty, 
                                                          fEdgelToLineDistCoef, fAngleBetweenGradientAndNormalCoef, maxLines, NULL);

//remove duplicates
	std::sort(lines.begin(), lines.end(), SortByAngle());
	std::vector<LineHough> newlines;
	LineHough last = lines[0];
	for (size_t i =1; i< lines.size(); ++i){
		if (lines[i].angle != last.angle || lines[i].R != last.R){
			if (std::abs((std::abs(lines[i].angle)- std::abs(last.angle))) < 1e-10 && std::abs(lines[i].R - last.R) < distThresh){
				//which has the better score?
				if (last.confidence < lines[i].confidence){
					//replace
					if (newlines.size() == 0)
						newlines.push_back(lines[i]);
					else
						newlines[newlines.size() -1] = lines[i];
				}
			}
			else 
				newlines.push_back(lines[i]);
		}
		last = lines[i];
	}

  if (argc ==4){
	  drawLines(newlines, image, 255, 0,0);
	  cvSaveImage(argv[3], image);
  }
  DumpToFile(newlines, argv[2]);
  
  //// output lines to MATLAB

  //uint nLines = lines.size();
  //mxArray* mxMat = (mxArray*) mxCreateDoubleMatrix(nLines,3,mxREAL) ; //arrange un columns
  //double* mat = (double*) mxGetPr(mxMat);

  //for (uint iLine = 0; iLine < nLines; iLine++)
  //{	 
  //  mat[nLines*0 + iLine] = lines[iLine].angle;
  //  mat[nLines*1 + iLine] = lines[iLine].R;
  //  mat[nLines*2 + iLine] = lines[iLine].confidence;  

  //}
  //out[lines_i] = mxMat;

  //if (nout >= 2) //return edge image, mostly for debugging
  //  {
  //    mxArray* mxImgEdge = (mxArray*) mxCreateNumericMatrix(imgHeight,imgWidth,mxUINT8_CLASS,mxREAL);
  //    uint8* imgEdge = (uint8*) mxGetPr(mxImgEdge); 
  //    
  //    Image<uint8> iplEdgeT(iplEdge);
  //    for (int x = 0; x < imgWidth; x++)
  //  	  for (int y = 0; y < imgHeight; y++)
  //  		 setColor(imgEdge,imgHeight, x,y, iplEdgeT[y][x]);
  //    
  //    out[imgEdge_i] = mxImgEdge;
  //  }

  cvReleaseImage(&image);
  cvReleaseImage(&iplEdge);
  cvReleaseImage(&iplImg);


  /////////////////////////////////////////////////////////
  cvReleaseImage(&imSmooth);
  
  cvReleaseImage(&xder16S);
  cvReleaseImage(&yder16S);
  /////////////////////////////////////////////////////////
}


