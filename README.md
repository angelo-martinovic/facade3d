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

In short, elements such as windows are modeled as intrusions in the wall. This boolean operation for ,,digging'' window holes is performed directly in 3ds Max. The facade texture is then orthographically projected on the final mesh. The environment also allows us to set lights, cast shadows, use shaders or other kind of post-processing techniques. Now, we will show how to run this code. 

### Rendering estimated Facades in 3ds max

First, Matlab exports our result to ```output/export/3D_3L_facades_3ds/*``` as several files for each facade,

* ```facadeID.obj``` contains sky, roof, shop and door
* ```facadeID_balc.obj``` containes balconies
* ```facadeID_wallWin.boxesbin``` contains wall and windows
* ```facadeID.png``` containes texture


Second, run 3ds max by simple opening scene file ```code/3ds_max/textured_facades.max```.

Third, open the script using MaxScript->RunScript as shown at the following figure,

![2run_script.jpg](https://bitbucket.org/repo/deay7R/images/4177371873-2run_script.jpg)

choose ```code/3ds_max/show_facades.ms```. You should get the following simple gui,

![3gui.jpg](https://bitbucket.org/repo/deay7R/images/2092428085-3gui.jpg)

The input path should correspond to yours ```C:\...\output\export\3D_3L_facades_3ds\``` 

![43ds.jpg](https://bitbucket.org/repo/deay7R/images/912925717-43ds.jpg)

Once you click on **RUN**, it reads each exported facade and render it from the window is now selected (be sure it Camera01 which is normalized for the facade).

![5render.jpg](https://bitbucket.org/repo/deay7R/images/671782614-5render.jpg)

### Possible problems
Code runs without problems on 3ds max 2009.

In some versions of 3dsmax, read binary file may return error because *ReadFloat* shoud be changed to *ReadLong*.

![rendering.jpg](https://bitbucket.org/repo/deay7R/images/402738020-rendering.jpg)