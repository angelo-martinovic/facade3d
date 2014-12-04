#!/usr/bin/env python
# author:   Markus Mathias <markus.mathias@esat.kuleuven.be>
# date:     25.02.2011
# version:  0.1

#needs numpy
#needs the modified version of ransac.py
#needs the modified version of line detector from http://graphics.cs.msu.ru/en/science/research/machinelearning/hough

#common call:
#for i in <path>/*.<fileType>; do python rectifyImage.py $i ${i%.png}rect.png & NPROC=$(($NPROC+1));if [ "$NPROC" -ge 8 ]; then wait; NPROC=0;fi; done


import sys
import os
import math
import shutil
import numpy as np
import ransac
import random
from optparse import OptionParser

#define epsilon
eps = np.finfo(type(float(1))).eps

#######################################adjust parameters here

#add path to the local pyopencv egg
#sys.path.append('/users/visics/mmathias/sw/pyopencv/lib64/python2.7/site-packages/pyopencv-2.1.0.wr1.2.0-py2.7-linux-x86_64.egg')
#import pyopencv as cv

#sys.path.append("/users/visics/mmathias/sw/opencv2.3.1/lib/python2.7/site-packages/")
#sys.path.append("/users/visics/rbenenso/no_backup/usr/local/lib/python2.7/site-packages/")
import cv2 as cv
#import cv


lineDetect = os.path.dirname(os.path.realpath(__file__)) + "/lineDetect/detectLines" 

writeFile = False 
debug = False 
writeErrfiles = True 

#commandline parameters, do not change here
cropImage = True


#ransac parameters
ransacIterations = 8000
mindist = 0.005 
minNoOfInliers = 1 


def test():
	
	im = cv.imread("test.png")
	img = im.copy() 
	phi = 0.3
	R=np.zeros((3,3),float)#cv.CreateMat(3,3,cv.CV_64FC1)
	R[0,0] = np.cos(phi) 
	R[0,1] = -np.sin(phi)
	R[0,2] = 0

	R[1,0] = np.sin(phi) 
	R[1,1] = np.cos(phi) 
	R[1,2] = 0

	R[2,0] = 0
	R[2,1] = 0
	R[2,2] = 1 
	while True:
		img = cv.warpPerspective(im,R, (img.shape[1],img.shape[0]) );



#######################################

def normalizePt(pt):
	"""normalizes the homogenious point"""
	if abs(pt[2]) > eps:
		pt[0] = pt[0]/ pt[2]
		pt[1] = pt[1]/ pt[2]
		pt[2] = 1.
	else:
		m = max(abs(pt[0]), abs(pt[1]))
		pt[0] = pt[0]/ m
		pt[1] = pt[1]/ m
	return pt

def normalizeLine(line):
	"""normalizes the line to unit length"""
	n = math.sqrt(line[0] * line[0] + line[1] * line[1])
	line[0] = line[0] / n
	line[1] = line[1] / n
	line[2] = line[2] / n
	return line


def readliness(filename):
	"""Read lines from file in format: \n
	   conf rho teta\n
	        ...\n
			...\n
		returns tuple of the list of lines and the list of confidences"""
	file = open(filename, 'r')
	lines = np.array([])
	confidences= np.array([])
	count = 0
	for l in file:
		v = l.split()
		rho = 	float(v[2])
		theta = float(v[1])
		conf =  float(v[0])
		confidences = np.append(confidences, conf)

		a = np.cos(theta)
		b = np.sin(theta)

		x0 = a*rho
		y0 = b*rho

		#get two points on the line
		p1x = (x0 + 1000*(-b))
		p1y = (y0 + 1000*(a))
		p2x = (x0 - 1000*(-b))
		p2y = (y0 - 1000*(a))

		#get the line representation in homogenious coordinates
		crossp = np.cross(np.array([p1x, p1y, 1.]).T, np.array([p2x, p2y, 1.]).T)
		crossp = normalizeLine(crossp)

		if lines.size >0:
			lines = np.vstack( (lines,crossp) )
		else:
			lines = crossp

	#confidences += confidences.max()
	confidences /= confidences.max() #normalize confidence values
	return (lines, confidences)

def null(A, e=eps):
	""" returns the right Nullspace of Matrix A"""
	u, s, vh = np.linalg.svd(A) # u*s*v = A
	erg = vh[2,:] # take last row
	return erg

def getBestLine(lines, model):
	"""returns the line that fits the model best"""
	tmpmodel = normalizeLine(model)
	return lines[np.argmin(np.dot(lines,tmpmodel))]


def getAngle(l1, l2):
	l1 = normalizeLine(l1)
	l2 = normalizeLine(l2)
	cosPhi = l1[0]*l2[0] + l1[1]* l2[1]
	cosPhi = cosPhi - eps
	if cosPhi >=1:
		cosPhi = 1
	if cosPhi <=-1:
		cosPhi = -1
	angle = np.arccos(cosPhi)
	if angle > np.pi/2.:
		angle = np.pi-angle
	return angle


def getVanishingPoints(lines, confidences, image):
	"""finds the vanishing points of the dominant directions of a list of lines"""


	#initialize model
	model = ransac.crossProductModel(mindist,image.shape[1], image.shape[0], True, True)
	lines = np.array([normalizeLine(i) for i in lines])
        print "lines total: ", len(lines)
	#run ransac
	print "ransacIterations: " ,ransacIterations
	model1, ransac_data1 , ransac_error1= ransac.ransac(lines ,model, 2, ransacIterations, mindist, minNoOfInliers, \
															debug=debug,return_all=True)
	#get the inlier set of lines
	lines1 = [lines[i] for i in ransac_data1] 
	#get the line closest to the model
	bestLine1 = getBestLine(lines1, model1)
	linesn = np.array([])
	#print 'lines1:' , lines1 
	print "lines taken by vp1: ", len(lines1)
	#take only lines with angle > pi/8
	for line in lines:
		angle = getAngle(line, bestLine1)
		#select lines outside pi/8 range
		if  abs(angle) > np.pi/8:
			if linesn.size > 0: 
				linesn= np.vstack((linesn, line))
			else:
				linesn= np.array(line)

	#img = image.copy()
	#superImpressLines(img, lines1, 0,0,255)
	#cv.imwrite('vanishingLines.jpg', img)
      
	print "lines left for second direction: ", len(linesn)
		
	if len(linesn)<=4:
	  linesn = np.array([])
	  for c in range(1,5):
	    if abs(bestLine1[1])>eps:
	      line = np.array([1., -bestLine1[0]/bestLine1[1] ,c])
	    else:
	      line = np.array([0., 1. ,c])

	    if linesn.size > 0: 
		    linesn= np.vstack((linesn, line))
	    else:
		    linesn= np.array(line)
	  
	
	#second round of ransac for the remaining lines
	model2, ransac_data2 , ransac_error2= ransac.ransac(linesn ,model, 2, ransacIterations, mindist, minNoOfInliers, \
															debug=debug,return_all=True)
	lines2 = [linesn[i] for i in ransac_data2] 
	bestLine2 = getBestLine(lines2, model2)
	  
	  
	if writeFile:
		img = image.copy()
		superImpressLines(img, lines1, 0,0,255)
		superImpressLines(img, lines2, 0,255, 0)
		cv.imwrite('vanishingLines.jpg', img)

	if debug:
		print 'model1:', model1 
		print 'No of inliers:', len(ransac_data1)
		print 'model error:' , ransac_error1
		print 'bestline1:' , bestLine1 
		print 'model2:', model2 
		print 'No of inliers:', len(ransac_data2)
		print 'model error:' , ransac_error2
		print 'bestline2:' , bestLine2 


	models= [model1, model2] 
	inlierLines = [lines1, lines2]
	bestLines = [bestLine1, bestLine2]
	return (models, bestLines, inlierLines)

def getClosestPoint(a, pts):
	aa = np.array(a)
	minIdx = -1
	mindist = np.inf
	for i,p in enumerate(pts):
		pp = np.array(p)
		d = pp -aa
		dist = np.sqrt((d*d).sum())
		if dist < mindist:
			mindist = dist
			minIdx = i
	return (minIdx,mindist)
		
def getRotationMatrix(phi):

	R=np.zeros((3,3),float)
	R[0,0] = np.cos(phi) 
	R[0,1] = -np.sin(phi)
	R[0,2] = 0

	R[1,0] = np.sin(phi) 
	R[1,1] = np.cos(phi) 
	R[1,2] = 0

	R[2,0] = 0
	R[2,1] = 0
	R[2,2] = 1 

	return R


def updateHomography(poss, hom):
	"""translates the image, returns (updated homography, translation vector, target image size, cropbox)"""
	global cropImage

	ptsx = []
	ptsy = []
	p = np.array([0.,0.,0.])
	# determine max extent of warped image
	for i,pos in enumerate(poss):
		p[0] = (pos[0])
		p[1] = (pos[1])
		p[2] = (1)
		erg = np.dot(hom,p)

		x = np.round(erg[0]/erg[2])
		y = np.round(erg[1]/erg[2])

		ptsx.append(x)
		ptsy.append(y)


	ptsx.sort()
	ptsy.sort()
	# compute new homography with added translation
	sx = float(poss[2][0])/float(ptsx[2] - ptsx[1])
	sy = float(poss[2][1])/float(ptsy[2] - ptsy[1])
	T = np.zeros((3,3),float);
	T[0,0] = sx
	T[0,1] = 0. 
	T[0,2] = -ptsx[1]* sx
	T[1,0] = 0. 
	T[1,1] = sy
	T[1,2] = -ptsy[1]* sy
	T[2,0] = 0. 
	T[2,1] = 0. 
	T[2,2] = 1.0
	if not cropImage:
		T[0,2] = -ptsx[0]* sx
		T[1,2] = -ptsy[0]* sy
		poss[2][0] = int(sx*(ptsx[3] - ptsx[0]+1 +0.5))
		poss[2][1] = int(sy*(ptsy[3] - ptsy[0]+1 +0.5))

	newhom = np.dot(T,hom)


	return newhom, poss[2] 


def isDegenerate(line):
	if abs(line[0]) < 1e-10:
		return True
	if abs(line[1]) < 1e-10:
		return True


def getAffine(l1, l2,theta):
	"""get affine part of the rectification: returns 3x3 Matrix A"""
	l1 = normalizeLine(l1)
	l2 = normalizeLine(l2)
	rot = False
	R= []
	if isDegenerate(l1) or isDegenerate(l2):
	#	rot = True
		print "rotated"
	#	R = getRotationMatrix(np.pi / 4.)
	#	l1 =  normalizeLine(np.dot(R,l1))
	#	l2 =  normalizeLine(np.dot(R,l2))
		
		

	#constraint circle for a and b
	ca = (-l1[1]/l1[0] - l2[1]/l2[0]) / 2.0
	if theta == np.pi/2.:
		cb = 0
	else:
		cb = (-l1[1]/l1[0] + l2[1]/l2[0]) / (2 * math.tan(theta))
	r =  abs((-l1[1]/l1[0] + l2[1]/l2[0]) / (2.0 * math.sin(theta)))

	#find the point on the circle closest to (0,1)
	v = np.array([-ca, 1-cb])
	n = math.sqrt(np.dot(v,v.conj())) # norm
	v /=n

	alpha = ca + r * v[0]
	beta  = cb + r * v[1]


	A = np.zeros((3,3), float)
	A[0,0] =1./beta
	A[0,1] =-alpha/beta
	A[0,2] =0.

	A[1,0] =0.
	A[1,1] =1.
	A[1,2] =0.

	A[2,0] =0.
	A[2,1] =0.
	A[2,2] =1.
	if rot:
		return np.dot(A,R) 

	return A
	

def getIntersections(w,h,line):
	"""returns intersection points of a homogenious line with the image border (0,0,w,h)"""
	pt = [np.array([]), np.array([])]
	line = normalizeLine(line)
	ix = normalizePt(np.cross(line, np.array([1,0,0 ])))    #intersect x axis
	iy = normalizePt(np.cross(line, np.array([0,1,0 ])))	#intersect y axis
	ir = normalizePt(np.cross(line, np.array([1,0,-w+1])))	#intersect right border of image
	ib = normalizePt(np.cross(line, np.array([0,1,-h+1])))	#intersect bottom of the image
	if abs(line[1]) < eps:
		pt[0] = iy
		pt[1] = ib
		return pt
	if abs(line[0]) < eps:
		pt[0] = ix
		pt[1] = ir
		return pt

	count = 0
	# find the two valid intersections
	if (ix[1] >= 0 and ix[1]<h and count <2):
		pt[count] = ix
		count +=1
	if (iy[0] >= 0 and iy[0]<w and count <2):
		pt[count] = iy
		count +=1
	if (ir[1] >= 0 and ir[1]<h and count <2):
		pt[count] = ir
		count +=1
	if (ib[0] >= 0 and ib[0]<w and count <2):
		pt[count] = ib
		count +=1
	return pt
	
def showLines(fname,img, lines):
	image = img.copy()
	for line in lines:
		pts = getIntersections(img.shape[1], img.shape[0], line)

		a = pts[0]
		b = pts[1]
		if len(a) == 0 or len(b) == 0:
			print "line not in image"
			continue
		cv.line(image, (int(a[0]), int(a[1])), (int(b[0]), int(b[1])), (255,0,0), 4)
		
	cv.imwrite(fname,image)
	return image
def superImpressLines(image, lines, red,green,blue):

	for line in lines:
		pts = getIntersections(image.shape[1], image.shape[0], line)

		a = pts[0]
		b = pts[1]
		if len(a) == 0 or len(b) == 0:
			print "line not in image"
			continue
		cv.line(image, (int(a[0]), int(a[1])), (int(b[0]), int(b[1])), (red,green,blue), 2)


def getProjective(vpts):
	"""get projective transformation matrix (3,3) P"""

	#getLineAtInfinity
	vp1 = normalizePt(vpts[0])
	vp2 = normalizePt(vpts[1])
	vinf = np.cross(vp1,vp2)
	vinf = normalizePt(vinf)

	
	P = np.zeros((3,3), float)
	P[0,0] =1.
	P[0,1] =0.
	P[0,2] =0.
	P[1,1] =1.
	P[1,0] =0.
	P[1,2] =0.
	P[2,0] =vinf[0]
	P[2,1] =vinf[1]
	P[2,2] =vinf[2]

	return P

def warpLines(lines, hom,size):
	"""use homography hom and image boarders to warp the lines"""
	linesP = np.array([])
	#np.array([np.linalg.inv(hom.T) * line for line in lines])
	#hom = hom.ndarray
	for line in lines:
		#line = np.array(line)
		#pt = getIntersections(size[1], size[0], line)
		#a = pt[0]
		#b = pt[1]
		#if len(a) == 0 or len(b) == 0:
		#	print "line not in image"
		#	continue
		#a = np.dot(hom, a.T)
		#b = np.dot(hom, b.T)
		if linesP.size == 0:
			linesP= np.dot(np.linalg.inv(hom).T,line)
		else:
			linesP = np.vstack((linesP, np.dot(np.linalg.inv(hom).T , line) ))
	return linesP

def getRotation(bestLine1, bestLine2):
	"""calculates the 3x3 rotation matrix to align the input lines with the axis"""

	#which line is closest to x axis?
	bestLine1 = normalizeLine(bestLine1)
	line = bestLine1
	
	#get angle to x-axis
	phi = np.arctan2(line[1], line[0]) 
	
	#adjust range of phi
	if phi < 0:
		if -phi > np.pi/2:
			phi = np.pi + phi
	else:
		if phi > np.pi/2:
			phi = phi - np.pi 

	#apply clockwise or counterclockwise rotation
	if phi >=0:
		R = np.zeros((3,3), float)
		R[0,0] = np.cos(phi)
		R[0,1] = np.sin(phi)
		R[1,0] =-np.sin(phi)
		R[1,1] = np.cos(phi)

		R[0,2] = 0
		R[1,2] = 0

		R[2,0] = 0
		R[2,1] = 0
		R[2,2] = 1
	else:
		phi = -phi
		R = np.zeros((3,3), float)
		R[0,0] = np.cos(phi)
		R[0,1] =-np.sin(phi)
		R[1,0] = np.sin(phi)
		R[1,1] = np.cos(phi)

		R[0,2] = 0
		R[1,2] = 0

		R[2,0] = 0
		R[2,1] = 0
		R[2,2] = 1

	return R
	

def getScale(sw, sh):
	"""return  3x3 scaling matrix S"""

	S=np.zeros((3,3), float)
	S[0,0] = sw 
	S[0,1] = 0
	S[0,2] = 0

	S[1,0] = 0 
	S[1,1] = sh 
	S[1,2] = 0

	S[2,0] = 0
	S[2,1] = 0
	S[2,2] = 1 

	return S

def saveInlierLinesHom(name,image,inlierLines,H, cropbox =None):
	img = image.copy() 
	img = cv.warpPerspective(image,H, (img.shape[1],img.shape[0]) );
	H,(neww, newh) = updateHomography([ [0,0] , [0,image.shape[0]-1] ,\
		[image.shape[1]-1, image.shape[0]-1] , [image.shape[1]-1,0] ], H)
	#img = np.zeros((image.shape[0], image.shape[1], image.shape[2]), dtype=np.uint8)

	img = cv.warpPerspective(image,H, (neww,newh) );

	superImpressLines( img, warpLines(inlierLines[0], H, image.shape),0,0, 255)
	superImpressLines( img, warpLines(inlierLines[1], H, image.shape),0,255, 0)
	print 'saving image: ' , name
	cv.imwrite(name, img)

def saveBestLinesHom(name,image,lines,H):
	img = image.copy()
	img = cv.warpPerspective(image,H, (image.shape[1],image.shape[0]) );

	#H = np.array(H)
	#img = cv.warpPerspective(image, H,(image.shape[0], image.shape[1]));
	H, (neww,newh)= updateHomography([ [0,0] , [0,image.shape[0]-1] ,\
		[image.shape[1]-1, image.shape[0]-1] , [image.shape[1]-1,0] ], H)
	img = cv.warpPerspective(image,H, (neww,newh) );
	print 'saving image: ' , name
	#cv.imwrite(name, img)
	showLines(name, img, warpLines(lines, H, image.shape))

def calcRectification(image, models, bestLines, inlierLines):

	#the horizontal line corresponds to bestLine1
	bestLine1 = normalizeLine(bestLines[0])
	bestLine2 = normalizeLine(bestLines[1])
#check if already rectified
	if (bestLine1[0] == 0 and bestLine2[1] == 0) or (bestLine1[1] == 0 and bestLine2[0] == 0):
		return image, np.eye(3,3)
	a = np.abs(np.dot(bestLine1, np.array([1,0,0])))
	b = np.abs(np.dot(bestLine2, np.array([1,0,0])))
	if b < a:
		bestLines = [bestLine1, bestLine2]
	else:
		bestLines = [bestLine2, bestLine1]

	allinlierLines = inlierLines[0] + inlierLines[1]
	if writeFile:
		showLines('inlierLines.jpg', image,allinlierLines)

	#dummy = cv.CreateMat(3,3,cv.CV_64FC1)
	#Ht = cv.CreateMat(3,3,cv.CV_64FC1)
	#Htt = cv.CreateMat(3,3,cv.CV_64FC1)
	
	if len(bestLines) !=2:
		raise Exception('the two dominant directions have not been found! Abort...')

	 
	if writeFile:
		bestLines_0 = bestLines

		saveBestLinesHom('initBestlines.jpg', image,bestLines_0,getScale(1.,1.))
		saveInlierLinesHom('initInliers.jpg', image,inlierLines,getScale(1.,1.))

		
		#projective transform
		P = getProjective(models)
		saveBestLinesHom('PBestlines.jpg', image,bestLines_0,P)
		saveInlierLinesHom('PInliers.jpg', image,inlierLines,P)
		bestLines =  warpLines(bestLines, P, image.shape)

		#affine transform
		A = getAffine(bestLines[0], bestLines[1], np.pi/2)
		Ht = np.dot(A,P)
		#cv.GEMM(A,P,1.0,dummy,0.0,Ht)
		saveBestLinesHom('ABestlines.jpg', image,bestLines_0,Ht)
		saveInlierLinesHom('AInliers.jpg', image,inlierLines,Ht)
		bestLines =  warpLines(bestLines, A, image.shape)

		#rotation transform
		R = getRotation(bestLines[0], bestLines[1])
		Htt = np.dot(R,A)
		Ht = np.dot(Htt,P)
		saveBestLinesHom('RBestlines.jpg', image,bestLines_0,Ht)
		saveInlierLinesHom('RInliers.jpg', image,inlierLines,Ht)

	else:
		#projective transform
		P = getProjective(models)

		bestLines =  warpLines(bestLines, P, image.shape)
		#affine transform
		A = getAffine(bestLines[0], bestLines[1], np.pi/2)

		bestLines =  warpLines(bestLines, A, image.shape)
		#rotation transform
		R = getRotation(bestLines[0], bestLines[1])

	# all homograhies after each other
	Ht = np.dot(R,A)
	H = np.dot(Ht,P)

	#determine translation and cropbox
	H,(neww, newh) = updateHomography([ [0,0] , [0,image.shape[0]-1] ,\
		[image.shape[1]-1, image.shape[0]-1] , [image.shape[1]-1,0] ], H)

	#imgH = np.zeros((image.shape[0], image.shape[1], image.shape[2]), dtype=np.uint8)

	imgH = cv.warpPerspective(image,H, (neww,newh) );
	#cv.NamedWindow("test")
	#cv.imshow("test", imgH)
	#cv.WaitKey()
	return imgH, H


def rectifyImage(image,lines, confidences):
	if writeFile:
		showLines('allLines.jpg', image,lines)

	print 'get vanishing points using ransac'
	models, bestLines, inlierLines = getVanishingPoints(lines, confidences, image)
	return calcRectification(image, models, bestLines, inlierLines)
	


	
def rectify(imageName, saveName, cropImg):
	global cropImage
	cropImage = cropImg
	"""main rectification funciton"""
	random.seed()
	linesname= '../tmp/' + str(random.random())[2:] + '.txt'	
	command = lineDetect + " " + "'" + imageName + "'" + " " + linesname
	print command
	os.system(command)
	if 1:
		image = cv.imread(imageName)
		lines, confidences = readliness(linesname)
		img, hom = rectifyImage(image ,lines, confidences)
		print saveName
		cv.imwrite(saveName, img)
		saveNameHom = os.path.splitext(saveName)[0] + "_rect.dat"
		hom.tofile(saveNameHom, " ")
		imageDims = np.array([img.shape[0], img.shape[1]])
		saveNameDims = os.path.splitext(saveName)[0] + "_scale.dat"
		imageDims.tofile(saveNameDims, " ")	

	else:
		try:
			image = cv.imread(imageName)
			lines, confidences = readliness(linesname)
			img, hom = rectifyImage(image ,lines, confidences)
			print saveName
			cv.imwrite(saveName, img)
			saveNameHom = os.path.splitext(saveName)[0] + "_rect.dat"
			hom.tofile(saveNameHom, " ")
		except Exception as e:
			print 'ERROR: ', e
			if os.path.exists(linesname):
				shutil.os.remove(linesname)
			if writeErrfiles: 
				errname= linesname[:-3] + "err"
				print errname
				f = open(errname, 'w')
				lines = "python rectifyImage.py " +  imageName + " " + saveName
				f.writelines(lines)
				print f
				f.close()
	if os.path.exists(linesname):
		shutil.os.remove(linesname)
def rectifyUsingGivenHomography(imageName, homographyname, saveName, crop, nn):
	global cropImage
	cropImage = crop
	image = cv.imread(imageName)
	H = np.fromfile(homographyname, float,-1,  " ")
	H = H.reshape(3,3)
	H,(neww, newh) = updateHomography([ [0,0] , [0,image.shape[0]-1] ,\
		[image.shape[1]-1, image.shape[0]-1] , [image.shape[1]-1,0] ], H)
	if nn:
		imgH = cv.warpPerspective(image,H, (neww,newh), flags=cv.INTER_NEAREST );

	else:
		imgH = cv.warpPerspective(image,H, (neww,newh) );
	cv.imwrite(saveName, imgH)

def parse_arguments():
		
	parser = OptionParser()
	parser.description = \
		"This program automatically rectifies input images\n\
		common call>:\n\
		for i in <path>/*.<fileType>; do python rectifyImage.py auto $i ${i%.<fileType>}rect.png & NPROC=$(($NPROC+1));if [ \"$NPROC\" -ge 8 ]; then wait; NPROC=0;fi; done"

	parser.add_option("-t", "--task", dest="task",
					   metavar="TASK", type="string", default="auto",
					   help="give the task: auto for automatic rectification | hom for rectification using a given homograpy (saved sequentially in a file with spaces as seperators)")

	parser.add_option("-i", "--imageName", dest="imageName",
					   metavar="IMAGENAME", type="string",
					   help="input image name")

	parser.add_option("-o", "--output", dest="saveName",
					   metavar="SAVENAME", type="string",
					   help="output image name")

	parser.add_option("-f", "--homograpyFile", dest="homographyName",
						metavar="HOMOGRAPYFILE", type="string",
						help="filenme of the homography")
	
	parser.add_option("-n", "--nearestNeighbor", dest="nn",
						action="store_true", default=False, 
						help="use nearest neighbor interpolation for warping using homographyFile")

	parser.add_option("-c", "--cropImage", dest="cropImage",
						action="store_true", default=False, 
						help="set to crop the output image")

												  
	(options, args) = parser.parse_args()
#print (options, args)

	if options.task:
		if options.task == "hom":
			if not (options.homographyName):
				parser.error("Task 'hom' requires the homography file specified: -h")

	if (not options.imageName) or (not options.saveName):
		parser.error("specify input and output files")


	return options 


def main():
	options = parse_arguments()
	if options.task=="auto":
		rectify(options.imageName, options.saveName, options.cropImage)
	if options.task=="hom":
		rectifyUsingGivenHomography(options.imageName, options.homographyName, options.saveName, options.cropImage, options.nn)





if __name__ == "__main__":
	main()


