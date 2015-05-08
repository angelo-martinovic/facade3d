% clear all classes;

%% Local folder structure
addpath batch/
addpath condor/                 % Calculation on the cluster
addpath config/
addpath eccv2014_eval/          % URL: http://www.vision.ee.ethz.ch/~rhayko/paper/eccv2014_riemenschneider_multiviewsemseg/
addpath logger/    
addpath mesh/                   
addpath planes/
addpath tools/    
addpath utils_3d/
addpath(genpath('thirdLayer/'));

%% External software
    %% Layer 1   
        % CNN 
        % Type: 2D feature extraction, recommended
        % URL: 
        addpath external/matconvnet/matlab/
        addpath external/matconvnet/models/ % Put the path to your CNN models here

        % STAIR vision library v2.4 
        % Type: 2D feature extraction
        % http://ai.stanford.edu/~sgould/svl/
        addpath external/lasik/

        % libLINEAR
        % Type: classifier, recommended
        % URL: http://www.csie.ntu.edu.tw/~cjlin/liblinear/
        addpath external/liblinear/matlab/

        % libSVM
        % Type: classifier, optional
        % URL: http://www.csie.ntu.edu.tw/~cjlin/libsvm/
        addpath external/libsvm/matlab/
        
        % BICLOP/DOPPIA 
        % Type: 2D object detector, optional
        % https://bitbucket.org/rodrigob/doppia
        addpath external/biclop/

        % KD-tree 
        % type: helper library for spin image calculation, required
        % URL: http://www.mathworks.com/matlabcentral/fileexchange/4586-k-d-tree
        addpath external/mat_kdtree/lib/ 
        addpath external/mat_kdtree/src/ 
        
    %% Layer 2
        % MEX Graph cuts
        % Type: CRF solver, required
        % URL: http://vision.ucla.edu/~brian/gcmex.html
        addpath external/GCMex2.3/

    %% Layer 3
        % CVX
        % Type: library for disciplined convex programming, required
        % URL: http://cvxr.com/cvx/
        addpath external/cvx/ 
        
        % Note: we are using the MOSEK solver in CVX. An academic license
        % is available at http://cvxr.com/cvx/academic/
        % Run the following 3 lines to initialize CVX with your new license
        % cvx_setup /path/to/license/cvx_license.dat % Point this to your license file
        % cvx_solver mosek
        % cvx_save_prefs

    %% MISC
        % export_fig
        % Type: library for exporting MATLAB figures, optional
        % URL: http://www.mathworks.com/matlabcentral/fileexchange/23629-export-fig
        addpath external/export_fig/ 
