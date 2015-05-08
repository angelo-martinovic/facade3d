# 3D all the way! #

![pipeline.jpg](http://homes.esat.kuleuven.be/~amartino/2015_cvpr_www/pics/pipeline04.jpg)

### Running the application
First, open the ```setup.m``` file and change the paths to the necessary libraries on your system. 

#### Required libraries

* ```STAIR``` vision library, http://ai.stanford.edu/~sgould/svl/ OR
```MatConvNet``` for CNN-based region features, http://www.vlfeat.org/matconvnet/
* ```libSVM``` library for Support Vector Machines, http://www.csie.ntu.edu.tw/~cjlin/libsvm/ OR
```liblinear``` library for large linear classification (recommended if using CNN features), http://www.csie.ntu.edu.tw/~cjlin/liblinear/
* ```KD tree``` helper library for spin image calculation, http://www.mathworks.com/matlabcentral/fileexchange/4586-k-d-tree
* ```GCMex``` MATLAB wrapper for graph cuts multi-label energy minimization, http://vision.ucla.edu/~brian/gcmex.html
* ```CVX``` MATLAB software for disciplined convex programming, http://cvxr.com/cvx/

#### Optional libraries

* ```Doppia``` object detector, https://bitbucket.org/rodrigob/doppia
* ```ExportFig``` library for exporting MATLAB figures, http://www.mathworks.com/matlabcentral/fileexchange/23629-export-fig


Next, set up your dataset. Our code comes with several example parameter setups provided in ```DatasetConfig.m```. Modify this file as required.

Finally, run the main script ```facade_run.m```.

### Creating 3D facade models

After running all three layers in 3D, the code will result in a set of labeled facades, each facade represented as a set of elements, such as windows, balconies, doors... All of these elements are represented as bounding boxes. To create a realistic-looking model from the labeling, we provide additional code that exports the result in 3dsMax and renders the result. 

In short, elements such as windows are modeled as intrusions in the wall. This boolean operation for ,,digging'' window holes is performed directly in 3ds Max. The facade texture is then orthographically projected on the final mesh. The environment also allows us to set lights, cast shadows, use shaders or other kind of post-processing techniques. Now, we will show how to run this code. Please go to https://bitbucket.org/honza00/pclvisul to obtain this code.

![rendering.jpg](https://bitbucket.org/repo/deay7R/images/402738020-rendering.jpg)

