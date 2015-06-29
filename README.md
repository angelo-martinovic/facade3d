# 3D all the way! #

![pipeline.jpg](http://homes.esat.kuleuven.be/~amartino/2015_cvpr_www/pics/pipeline04.jpg)

This code implements the approach detailed in the paper ```3D All The Way: Semantic Segmentation of Urban Scenes From Start to End in 3D```, check the project page: http://homes.esat.kuleuven.be/~amartino/2015_cvpr_www/index.html.

#### Required libraries

* ```OpenCV``` (recommended version: 2.4), http://opencv.org/downloads.html for region-based features from STAIR vision library (integrated) OR
```MatConvNet``` for CNN-based region features, http://www.vlfeat.org/matconvnet/
* ```libSVM``` library for Support Vector Machines, http://www.csie.ntu.edu.tw/~cjlin/libsvm/ OR
```liblinear``` library for large linear classification (recommended if using CNN features), http://www.csie.ntu.edu.tw/~cjlin/liblinear/
* ```CVX``` MATLAB software for disciplined convex programming, http://cvxr.com/cvx/  (needed only if running the third layer)

#### Optional libraries

* ```Doppia``` object detector, https://bitbucket.org/rodrigob/doppia

### Running the application
First, open the ```code/setup.m``` file and either change the paths or redirect the symlinks in the ```code/external/``` subfolder to the installed libraries on your system. Second, compile the dependencies in the ```code/3rdparty/``` subfolder.

In the next step you need to set up your dataset. A toy dataset with 4 images and a low-resolution point cloud is provided in the ```dataToy/``` subfolder. Use this as a template for your new dataset. The data folder should have the following structure:

* ```images/``` folder with jpg images
* ```labels/``` folder with ground truth labels in the form of png images (colormap maps pixel colors into classes)
* ```cameras.txt``` camera file, VisualSFM output
* ```listtrain.txt``` training set (list of filenames without extensions)
* ```listeval.txt``` evaluation (test) set (list of filenames without extensions)
* ```listall.txt``` all filenames in images/
* ```pcl.ply``` the point cloud (e.g. from SfM, PMVS, CMP MVS)
* ```pcl_depth.mat``` depth of each point from the estimated facade plane
* ```pcl_gt_train.ply``` ground truth point cloud of the training set (colormap maps point colors to classes)
* ```pcl_gt_test.ply``` ground truth point cloud of the test set (colormap maps point colors to classes)
* ```pcl_split.mat``` output of facade splitting: list of integers, each 3D point assigned to one facade ID

The parameters are set-up in ```code/DatasetConfig.m```. Modify this file to reflect the particularities of your dataset.

Finally, run the main script ```code/facade_run.m```.

TODO: Facade splitting in 3D has not yet been integrated. A precalculated split file is provided in ```dataToy/pcl_split.mat```.

### Creating 3D facade models

After running all three layers in 3D, the result will be a set of labeled facades, each facade represented as a set of elements, such as windows, balconies, doors... All of these elements are represented as bounding boxes. To create a realistic-looking model from the labeling, we provide additional code that exports the result in 3dsMax and renders the result. 

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