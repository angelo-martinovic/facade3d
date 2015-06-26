% clear all classes;

%% Local folder structure
addpath batch/
%addpath condor/                 % Calculation on the cluster
addpath config/
addpath eccv2014_eval/          % URL: http://www.vision.ee.ethz.ch/~rhayko/paper/eccv2014_riemenschneider_multiviewsemseg/
addpath logger/    
addpath mesh/                   
addpath planes/
addpath tools/    
addpath utils_3d/
addpath(genpath('thirdLayer/'));


%% External software
%
% Make sure that the symlinks in the external/ subfolder point to your
% local installations, or simply edit this file!
%
    %% Layer 1   
        % STAIR vision library v2.4, recommended
        % Type: 2D feature extraction
        % http://ai.stanford.edu/~sgould/svl/
        % Integrated in the codebase, under 3rdparty/svlFeatExtract
        % Requires OpenCV (2.4) during compilation and execution
        addpath external/opencv/

        % CNN 
        % Type: 2D feature extraction, optional
        % URL: http://www.vlfeat.org/matconvnet/
        addpath external/matconvnet/matlab/
        addpath external/matconvnet/models/ % Put the path to your CNN models here

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
    % Integrated in the codebase
        % MEX Graph cuts
        % Type: CRF solver, required
        % URL: http://vision.ucla.edu/~brian/gcmex.html
        addpath 3rdparty/GCMex2.3/
        
        % KD-tree 
        % type: helper library for spin image calculation
        % URL: http://www.mathworks.com/matlabcentral/fileexchange/4586-k-d-tree
        addpath 3rdparty/mat_kdtree/lib/ 
        addpath 3rdparty/mat_kdtree/src/ 
        
        % export_fig
        % Type: library for exporting MATLAB figures
        % URL: http://www.mathworks.com/matlabcentral/fileexchange/23629-export-fig
        addpath 3rdparty/export_fig/ 
