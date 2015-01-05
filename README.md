# 3D all the way! #

![rendering.jpg](https://bitbucket.org/repo/deay7R/images/402738020-rendering.jpg)

### Running the application
First, open the ```setup.m``` file and change the paths to the necessary libraries on your system. You will require:

* ```libSVM``` SVM library, http://www.csie.ntu.edu.tw/~cjlin/libsvm/, for the first layer
* ```GC_optimization``` software for energy minimization with graph cuts, http://www.csd.uwo.ca/faculty/olga/software.html, for the second layer
* ```CVX``` Matlab software for disciplined convex programming, http://cvxr.com/cvx/, for the third layer

Next, set up your dataset. Our code comes with an example dataset, with parameters provided in ```InitializeDataset.m```. Modify this file if your dataset requires it.

Finally, run the main script ```facade_run.m```.

### Creating 3D facade from WA

Result of Matlab 3DFacade is a set of hypothesized windows, balconies, walls and other objects on the facade. All of them are represented in a form of boxes. To make facades looking realistic, one need to consider windows create holes in the wall and the texture should be projected on the final mesh. This boolean operation for ,,digging'' holes window holes in the mesh of the wall is not an easy problem. For that, we use 3ds max to perform these operations and to visualize facade in a nice way as one can set lights, cast shadows, use shaders or any kind of post-processing technique you can imagine very easily. Now, we will show how to run this code

### How to run it

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

