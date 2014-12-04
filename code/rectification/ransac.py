import numpy
import scipy # use numpy if scipy unavailable
import scipy.linalg # use numpy if scipy unavailable

## Copyright (c) 2004-2007, Andrew D. Straw. All rights reserved.

## Redistribution and use in source and binary forms, with or without
## modification, are permitted provided that the following conditions are
## met:

##     * Redistributions of source code must retain the above copyright
##       notice, this list of conditions and the following disclaimer.

##     * Redistributions in binary form must reproduce the above
##       copyright notice, this list of conditions and the following
##       disclaimer in the documentation and/or other materials provided
##       with the distribution.

##     * Neither the name of the Andrew D. Straw nor the names of its
##       contributors may be used to endorse or promote products derived
##       from this software without specific prior written permission.

## THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
## "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
## LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
## A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
## OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
## SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
## LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
## DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
## THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
## (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
## OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# modified by:   Markus Mathias <markus.mathias@esat.kuleuven.be>
# date:     	 01.03.2011
# version:  	 0.1

eps = numpy.finfo(type(float(1))).eps

def ransac(data,model,n,k,t,d,debug=False,return_all=False):
    """fit model parameters to data using the RANSAC algorithm
    
This implementation written from pseudocode found at
http://en.wikipedia.org/w/index.php?title=RANSAC&oldid=116358182

{{{
Given:
    data - a set of observed data points
    model - a model that can be fitted to data points
    n - the minimum number of data values required to fit the model
    k - the maximum number of iterations allowed in the algorithm
    t - a threshold value for determining when a data point fits a model
    d - the number of close data values required to assert that a model fits well to data
Return:
    bestfit - model parameters which best fit the data (or nil if no good model is found)
iterations = 0
bestfit = nil
besterr = something really large
while iterations < k {
    maybeinliers = n randomly selected values from data
    maybemodel = model parameters fitted to maybeinliers
    alsoinliers = empty set
    for every point in data not in maybeinliers {
        if point fits maybemodel with an error smaller than t
             add point to alsoinliers
    }
    if the number of elements in alsoinliers is > d {
        % this implies that we may have found a good model
        % now test how good it is
        bettermodel = model parameters fitted to all points in maybeinliers and alsoinliers
        thiserr = a measure of how well model fits these points
        if thiserr < besterr {
            bestfit = bettermodel
            besterr = thiserr
        }
    }
    increment iterations
}
return bestfit
}}}
"""
    iterations = 0
    bestfit = None
    besterr = -numpy.inf
    best_inlier_idxs = None
    tt = 0
    while iterations < k:
        printit = False
        maybe_idxs, test_idxs = random_partition(n,data.shape[0])
        if len(maybe_idxs) == 0:
	  continue
	
        #maybe_idxs, test_idxs = random_partition(n,len(data))
        maybeinliers = data[maybe_idxs,:]
        test_points = data[test_idxs]
        #print "maybeinliers: " , maybeinliers
        if len(maybeinliers) ==0:
			iterations+=1
			continue
        maybemodel = model.fit(maybeinliers)
        test_err = model.get_error( test_points, maybemodel)
        also_idxs = test_idxs[test_err < t] # select indices of rows with accepted points

        if debug:
			if (len(also_idxs) > tt):
				print 'noOfModelFit:', len(also_idxs)
				print data[also_idxs[0]]
				print 'maybemodel: ' , maybemodel
				#print 'error: ', numpy.mean( [test_err < t] )
				tt = len(also_idxs)
				printit = True
			
        alsoinliers = data[also_idxs,:]
        if len(alsoinliers) > d:
            betterdata = numpy.concatenate( (maybeinliers, alsoinliers) )
            bettermodel = model.fit(betterdata)
            better_errs = model.get_error( betterdata, bettermodel)
            thiserr = numpy.mean( better_errs )

            if printit:
					print 'bettererror so far ', thiserr
					print 'bestmodel so far', bettermodel 
			
            if len(better_errs) > besterr:
                #print 'really best moedel', bettermodel
                #print 'thiserror', thiserr
                #print 'maybemodel:', maybemodel
                #print 'bm', bettermodel
                bestfit = bettermodel
                besterr = len(better_errs)
                best_inlier_idxs = numpy.concatenate( (maybe_idxs, also_idxs) )
                if printit:
					print 'besterror so far ', besterr
					print 'bestmodel so far', bestfit
        iterations+=1
    if bestfit is None:
		raise Exception("Ransac: did not meet fit acceptance criteria")
    if return_all:
        return normalizePt(bestfit), best_inlier_idxs, besterr
    else:
        return normalizePt(bestfit)

def null(A, e=eps):
	u, s, vh = numpy.linalg.svd(A) # u*s*v = A
	erg = vh[2,:] # take last row
	return erg

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




def random_partition(n,n_data):
    """return n random rows of data (and also the other len(data)-n rows)"""
    all_idxs = numpy.arange( n_data )
    numpy.random.shuffle(all_idxs)
    idxs1 = all_idxs[:n]
    idxs2 = all_idxs[n:]
    return idxs1, idxs2


def normalizeLine(line):
	"""normalizes the line to unit length"""
	n = numpy.sqrt(line[0] * line[0] + line[1] * line[1])
	line[0] = line[0] / n
	line[1] = line[1] / n
	line[2] = line[2] / n
	return line


class crossProductModel:
    """ calculates the crossproduct as a model for the estimated vanishing point of two lines

    This class serves as an example that fulfills the model interface
    needed by the ransac() function.
    
    """
    def __init__(self,dist, imwidth, imheight, vanishingPointShouldBeOutsideTheImage, debug=False):
		self.debug = debug
		self.dist = dist
		self.parallel = False
		self.invalid = False 
		self.w = imwidth
		self.h = imheight
		self.vanishingPointShouldBeOutsideTheImage = vanishingPointShouldBeOutsideTheImage
		
    def fit(self, data):
		t = scipy.cross(data[0], data[1])
		k = abs(data[0] - data[1])
		#these lines where added due to a bug in the line detector, which produced
		#many times the same line
		if (str(k) == "[ 0.  0.  0.]"):
			self.invalid = True
		else:
			self.invalid = False
		#check for parrallel lines (intersection at infinity)
		norm = numpy.sqrt(t[0]* t[0] + t[1]*t[1] +t[2]*t[2])
		t[0] /= norm
		t[1] /= norm
		t[2] /= norm 

		return t

    def get_dist(self, model):
		d = numpy.sqrt(model[0]* model[0] + model[1]* model[1])
		print "newDist: ", self.dist + int(d/1000)
		return self.dist + int(d/1000)


    def get_error( self, data, model):
				
		if (self.vanishingPointShouldBeOutsideTheImage):
			if (model[0]/model[2] >=0 and model[0]/model[2] <= self.w and model[1]/model[2] >=0 and model[1]/model[2] <=self.h):
				errors = numpy.array(len(data)*[float(numpy.inf)])
				return errors

		if (self.invalid):
			#set the error to max
			errors = numpy.array(len(data)*[float(numpy.inf)])
			return errors

		
		errors = scipy.dot(data, model.T)
		errors = numpy.array([abs(e) for e in errors])
		return errors 

    
