% Various stuff
% addpath(genpath('/esat/sadr/amartino/Code/matlabtools/'));
clear all classes;

%% Local folder structure
addpath condor/                 % Calculation on the cluster
addpath config/
addpath logger/
addpath tools/                  % Helper functions
addpath mesh/                   
addpath planes/
addpath utils_3d/
addpath(genpath('thirdLayer/'));

%% External software
    %% Layer 1
        % CNN 
        % Type: 2D feature extraction, recommended
        % URL: http://www.vlfeat.org/matconvnet/
        addpath external/matconvnet/matlab/
        addpath external/matconvnet/models/ % Put the path to your CNN models here

        % STAIR vision library v2.4 
        % Type: 2D feature extraction, optional
        % http://ai.stanford.edu/~sgould/svl/
        addpath external/lasik/

        % BICLOP/DOPPIA 
        % Type: 2D object detector, optional
        addpath external/biclop/

        % libLINEAR
        % Type: classifier, recommended
        % URL: http://www.csie.ntu.edu.tw/~cjlin/liblinear/
        addpath external/liblinear/matlab/

        % libSVM
        % Type: classifier, optional
        % URL: http://www.csie.ntu.edu.tw/~cjlin/libsvm/
        addpath external/libsvm/matlab/

        % KD-tree 
        % type: library for 3D feature extraction (spinImages), required
        % URL: http://www.mathworks.com/matlabcentral/fileexchange/4586-k-d-tree
        addpath external/mat_kdtree/lib/ 
        addpath external/mat_kdtree/src/ 

    %% Layer 2
        % Graph cuts
        % Type: CRF solver, required
        % URL: http://vision.ucla.edu/~brian/gcmex.html
        addpath external/GCMex2.3/

    %% Layer 3
        % CVX
        % Type: library for disciplined convex programming, required
        % URL: http://cvxr.com/cvx/
        addpath external/cvx/ 
        % cvx_setup % Uncomment if your cvx is not set-up at MATLAB startup.

    %% MISC
        % ECCV 2014 
        % Type: evaluation protocol from eccv14, required
        % URL: http://www.vision.ee.ethz.ch/~rhayko/paper/eccv2014_riemenschneider_multiviewsemseg/
        addpath external/eccv2014_eval/

        % export_fig
        % Type: library for exporting matlab figures
        % URL: http://www.mathworks.com/matlabcentral/fileexchange/23629-export-fig
        addpath external/export_fig/ 

