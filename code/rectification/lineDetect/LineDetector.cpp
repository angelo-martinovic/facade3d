

/*************************************************************************************************************/
// This is an implementation of lines detection method described in 
// O. Barinova, V. Lempitsky, P. Kohli "On detection of multiple object instances using Hough Transforms", CVPR 2010 
// 30/10/2010
//
// Two types of cost fuctions for attaching an edgel to a line are supported
//  C(edgel, line) = in_fDistCoef * distance^k1(edgel, line) + in_fAngleCoef * angle^k2(edgel, line),
//
// in_fDistCoef and in_fAngleCoef      are parameters of the method
// distance(edgel, line)               is a euclidean distance between edgel and line
// angle(edgel, line)                  is an angle between direction of image gradient at the edgel and the line
// k1 can be either 1 (in case UseQuadraticCostForDist is set to false) or 2 (in case UseQuadraticCostForDist is set to true)
// k2 can be either 1 (in case UseQuadraticCostForAngle is set to false) or 2 (in case UseQuadraticCostForAngle is set to true)
/*************************************************************************************************************/

#include "LineDetection.h"

const double PI = 3.141592653589793238462643383279502884197;

const double angleStep = PI/1000.0;	// resolution of Hough images in angles
const double distStep = 0.5; // resolution of Hough images in distance

const int nAngles = PI/angleStep;	// number of bins in Hough image
const double minAngle = -PI/2; // minimum angle

const bool UseQuadraticCostForDist = false; // use quadratic cost for distance penalty
const bool UseQuadraticCostForAngle = false;  // use quadratic cost for angle penalty


/*************************************************************************************************************/
double round(double d)
{
  return floor(d + 0.5);
};

/*************************************************************************************************************/
void DumpToFile(vector<LineHough> lines, const char *filename)
{
	FILE *out = fopen(filename, "w");
	for(int i = 0; i < lines.size(); i++)
		fprintf(out, "%lf %lf %lf\n", lines[i].confidence, lines[i].angle, lines[i].R);
	fclose(out);
}


/*************************************************************************************************************/
std::vector<LineHough> DetectLinesWithGradient(IplImage *edgeImage, //image with edge map 
                                          IplImage *dxImage, //gradient along horizontal axis
                                          IplImage *dyImage, // gradient along vertical axis
                                          float in_fLineBGBias, // background bias
                                          float in_fLineHypPenalty, // penalty for switching on one line
                                          float in_fDistCoef, // coefficient for distance penalty
                                          float in_fAngleCoef, // coefficient for angle penalty
                                          int maxLines, // maximum number of lines 
                                          const char *saveFirst) // save lines from Hough image
{
	int imWidth = edgeImage->width;
	int imHeight = edgeImage->height;

  vector<LineHough> lines;

	int nEdges = cvCountNonZero(edgeImage);
	
	int *edgeX = new int[nEdges]; // x-coordinates of edgels
	int *edgeY = new int[nEdges]; // y-coordinates of edgels
	double *edgeAlpha = new double[nEdges]; // angles of edgels
	int *edgeAlphaIndex = new int[nEdges]; // angles indexes of edgels
	double *maxVote = new double[nEdges]; // maximum given vote for each edgel

  int iMaxDistToLine; // maximum distance from edgel to line
  int iMaxdAngleToGrad; // maximum angle between an edgel and a line

  if (in_fDistCoef == 0)
  {
    return lines;
  }
  

  if (UseQuadraticCostForDist)
      iMaxDistToLine = int(floor( sqrt(in_fLineBGBias / in_fDistCoef) / distStep) + 1);
  else
    iMaxDistToLine = int(floor(in_fLineBGBias / in_fDistCoef / distStep) + 1);

  if (in_fAngleCoef == 0)
  {
      iMaxdAngleToGrad = int(floor(nAngles/2.0));
  }
  else
  {
    if (UseQuadraticCostForAngle)
      iMaxdAngleToGrad = int(min(floor( sqrt(in_fLineBGBias / in_fAngleCoef) / angleStep) + 1, 
                                floor(nAngles/2.0)));
    else
      iMaxdAngleToGrad = int(min(floor(in_fLineBGBias / in_fAngleCoef / angleStep) + 1, 
                                floor(nAngles/2.0)));  
  }


  // fill the array of penalties for distances between edgel and line
  double *distPenalties = new double[iMaxDistToLine];
  for (int iDist = 0; iDist < iMaxDistToLine; iDist ++)
  {
    if (UseQuadraticCostForDist)
      distPenalties[iDist] = in_fDistCoef*(iDist*distStep)*(iDist*distStep);
    else
      distPenalties[iDist] = in_fDistCoef*iDist*distStep;
  }
  
  // fill the array of penalties for angles between edgel and line
  double *alphaPenalties = new double[iMaxdAngleToGrad];
  for (int iAlpha = 0; iAlpha < iMaxdAngleToGrad; iAlpha ++)
  {
    if (in_fAngleCoef == 0)
    {
        alphaPenalties[iAlpha] = 0.0;
    }
    else
    {
      if (UseQuadraticCostForAngle)
        alphaPenalties[iAlpha] = in_fAngleCoef*(iAlpha*angleStep)*(iAlpha*angleStep);
      else
        alphaPenalties[iAlpha] = in_fAngleCoef*(iAlpha*angleStep);
    }
  }
    
  int offsetR = iMaxDistToLine;

  
  // fill the edgel coordinates and angles 
	int iEdge = 0;
	for(int y = 0; y < imHeight; y++)
		for(int x = 0; x < imWidth; x++)
		{
			if(((uchar*)(edgeImage->imageData + edgeImage->widthStep*y))[x])
			{
				edgeX[iEdge] = x;
				edgeY[iEdge] = y;

        double dx = ((short int*)(dxImage->imageData + y * dxImage->widthStep))[dxImage->nChannels * x ];
        double dy = ((short int*)(dyImage->imageData + y * dyImage->widthStep))[dyImage->nChannels * x ];

        if (abs(dx) < 1e-6)
          edgeAlpha[iEdge] = PI / 2.0;
        else
          edgeAlpha[iEdge] = atan (dy/dx);//alpha is between -pi/2 and pi/2

        edgeAlphaIndex[iEdge] = int(round((edgeAlpha[iEdge]-minAngle)/angleStep));       
        
				maxVote[iEdge++] = in_fLineBGBias; // initialize maximum given vote with in_fLineBGBias
			}
		}
	assert(iEdge == nEdges);

  double *coss;
  double *sins;
  
  double *cos_divided;
  double *sin_divided;

  coss = new double[nAngles];
  sins = new double[nAngles];
  
  cos_divided = new double[nAngles];
  sin_divided = new double[nAngles];
  
  double *angles;
  angles = new double[nAngles];

  // precalulate sin and cos of angles to speedup computation 
	for(int iAngle = 0; iAngle < nAngles; iAngle++)
	{
		angles[iAngle] = -PI/2+iAngle*angleStep;

    coss[iAngle] = cos(angles[iAngle]);
		sins[iAngle] = sin(angles[iAngle]);    

		cos_divided[iAngle] = coss[iAngle] / distStep;
		sin_divided[iAngle] = sins[iAngle] / distStep;    
	}
	double maxR = ceil(sqrt(double(imWidth)*imWidth+imHeight*imHeight) / distStep);

  int nDists = int(maxR*2+offsetR);
  
  // fill array of line parameters for Hough 
  double *distValues;
  distValues = new double[nDists];

  for (int iDist = 0; iDist < nDists; iDist++)
  {
    distValues[iDist] = (iDist-maxR)*distStep;
  }

  // precalculate maximum distance from line for given angle to speedup computation 
  int *maxRforGivenAngle;
  maxRforGivenAngle = new int[iMaxdAngleToGrad];
  for (int iAngle = 0; iAngle < iMaxdAngleToGrad; iAngle++)
  {    
    double alphaPenalty = alphaPenalties[iAngle];
  
    double defaultPenalty = in_fLineBGBias - alphaPenalty; 

    if (UseQuadraticCostForDist)
      maxRforGivenAngle[iAngle] = floor(sqrt(defaultPenalty / in_fDistCoef) /distStep) + 1;
    else
      maxRforGivenAngle[iAngle] = floor(defaultPenalty / (in_fDistCoef*distStep)) + 1;
  } 
    

	IplImage *houghImage = cvCreateImage(cvSize(nDists, nAngles), IPL_DEPTH_64F, 1);
	cvSetZero(houghImage);

	// perform Hough voting for the first time
	for(int i = 0; i < nEdges; i++)
	{
		double X = edgeX[i];
		double Y = edgeY[i];
		for(int a_ = edgeAlphaIndex[i]-iMaxdAngleToGrad+1; a_ <= edgeAlphaIndex[i]+iMaxdAngleToGrad-1; a_++)
		{
      int a = a_;
      if (a < 0) a += nAngles;
      if (a >= nAngles) a -= nAngles;

			double r = X*cos_divided[a]+Y*sin_divided[a] + maxR;

      int dAlphaIdx = abs(a_-edgeAlphaIndex[i]);
      double alphaPenalty = alphaPenalties[dAlphaIdx];
              
      double *pHoughImage = (double*)(&((double*)(houghImage->imageData + houghImage->widthStep*a))[int(r+offsetR)]);
      double vote = max(0.0, in_fLineBGBias - alphaPenalties[dAlphaIdx]);

      (*pHoughImage) += vote;

      double *pHoughImagePos = pHoughImage;  
      pHoughImagePos++;
      double *pHoughImageNeg = pHoughImage;
      pHoughImageNeg --;
           
      double defaultPenalty = in_fLineBGBias - alphaPenalty; 
      int maxRforCurAngle = maxRforGivenAngle[dAlphaIdx];
      for(int dR = 1; dR < maxRforCurAngle; dR++, pHoughImagePos++, pHoughImageNeg--)
      {
          double vote = defaultPenalty - distPenalties[dR];
          
				  *pHoughImagePos += vote;
				  *pHoughImageNeg += vote;  
      }
		}
	}

	// save Hough lines if required
  if(saveFirst) 
  {
		FILE *out = fopen(saveFirst, "w");
		IplImage *tmp1 = cvCreateImage(cvSize(houghImage->width, houghImage->height), IPL_DEPTH_32F, 1);
		cvConvert(houghImage, tmp1);
		IplImage *tmp = cvCloneImage(tmp1);
		cvDilate(tmp1, tmp);
		IplImage *localMax = cvCreateImage(cvSize(houghImage->width, houghImage->height), IPL_DEPTH_8U, 1);
		cvCmp(tmp1, tmp, localMax, CV_CMP_EQ);

		for(int y = 0; y < localMax->height; y++)
    {
			for(int x = 0; x < localMax->width; x++)
			{
				if(((uchar*)(localMax->imageData + localMax->widthStep*y))[x])
				{
					int aMax = y;
					double rMax = x;
					double vMax = ((double*)(houghImage->imageData + houghImage->widthStep*y))[x];
					if (vMax < 2*in_fLineHypPenalty) continue;

					rMax -= offsetR;
					LineHough newLine;
					newLine.angle = angles[aMax];
					newLine.R = rMax-maxR;
					newLine.confidence = vMax;

					fprintf(out, "%lf %lf %lf\n", newLine.confidence, newLine.angle, newLine.R);
				}
			}
    }
		fclose(out);
		cvReleaseImage(&tmp);
		cvReleaseImage(&tmp1);
		cvReleaseImage(&localMax);
	}

 // iterate Hough voting with changing votes

	for(int iter = 0; iter < maxLines; iter++)
	{
		int aMax, rMax;
		double vMax, dummy;
		CvPoint maxLoc;
		cvMinMaxLoc(houghImage, &dummy, &vMax, NULL, &maxLoc);
		aMax = maxLoc.y;
		rMax = maxLoc.x;
		if(vMax < in_fLineHypPenalty) break;

		rMax -= offsetR;
		LineHough newLine;
		newLine.angle = angles[aMax];
    newLine.R = distValues[rMax];
		newLine.confidence = vMax;

		lines.push_back(newLine);

    //updating the hough map
		for(int i = 0; i < nEdges; i++)
		{
			double prevVote = maxVote[i];
			double X = edgeX[i];
			double Y = edgeY[i];
			
      double rUpd = X*cos_divided[aMax]+Y*sin_divided[aMax] + maxR;
			int dR = abs(rUpd-rMax)/distStep;
      if (dR >= iMaxDistToLine) continue;
       
      int dAlphaIdx = abs(aMax-edgeAlphaIndex[i]);
      if (dAlphaIdx > nAngles/2) dAlphaIdx = nAngles - dAlphaIdx;
      if (dAlphaIdx >= iMaxdAngleToGrad) continue;

      double newVote = min(double(in_fLineBGBias), distPenalties[dR] + alphaPenalties[dAlphaIdx]);
			if(newVote >= prevVote) continue; //this node would stay with the previous hypothesis
			
			for(int a_ = edgeAlphaIndex[i]-iMaxdAngleToGrad+1; a_ <= edgeAlphaIndex[i]+iMaxdAngleToGrad-1; a_++)
			{
        int a = a_;
        if (a < 0) a += nAngles;
        if (a >= nAngles) a -= nAngles;

				double r = X*cos_divided[a]+Y*sin_divided[a] + maxR;
                          
        int dAlphaIdx = abs(a_-edgeAlphaIndex[i]);
        double alphaPenalty = alphaPenalties[dAlphaIdx];
              
        double *pHoughImage = (double*)(&((double*)(houghImage->imageData + houghImage->widthStep*a))[int(r+offsetR)]);
        double vote = min(double(in_fLineBGBias), alphaPenalty);
        (*pHoughImage) += max(newVote-vote, 0.0) - max(prevVote-vote, 0.0);

        double *pHoughImagePos = pHoughImage;  
        pHoughImagePos++;
        double *pHoughImageNeg = pHoughImage;
        pHoughImageNeg --;

        int maxRforCurAngle = maxRforGivenAngle[dAlphaIdx];
             
        for(int dR = 1; dR < maxRforCurAngle; dR++, pHoughImagePos++, pHoughImageNeg--)
        {
            double vote = distPenalties[dR] + alphaPenalty;
            if (prevVote <= vote) continue;
                
            double deltaVote = max(newVote, vote) - prevVote;

				    *pHoughImagePos += deltaVote;
				    *pHoughImageNeg += deltaVote;   

        }
			}
			maxVote[i] = newVote;
		}
	}

	cvReleaseImage(&houghImage);
	delete[] edgeX;
	delete[] edgeY;
	delete[] maxVote;

  delete [] edgeAlpha;  
	delete [] edgeAlphaIndex;

  delete [] distPenalties;
  delete [] alphaPenalties;

  delete [] distValues;

  delete [] maxRforGivenAngle;

  delete [] cos_divided;
  delete [] sin_divided;

  delete [] coss;
  delete [] sins;
  
  delete [] angles;

	return lines;
}
