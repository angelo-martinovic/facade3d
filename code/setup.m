% Various stuff
% addpath(genpath('/esat/sadr/amartino/Code/matlabtools/'));
clear all classes;


% Graph cuts
addpath /esat/sadr/amartino/Code/GCMex2.3/

% SVM training
addpath /esat/sadr/amartino/Code/libsvm-3.14/matlab/

% Hayko's code
addpath('/esat/sadr/amartino/monge428New/eccv2014_dataset/');

% Local folders
addpath condor/
addpath mesh/
addpath planes/
addpath firstLayer/
addpath libs/
addpath(genpath('thirdLayer/'));


% 3d


%  addpath /users/visics/jknopp/libraries/mat_general/  %%% for compSpinImages
 addpath /users/visics/jknopp/libraries/mat_kdtree/lib/  %%% library needed by  spinIm needs  
 addpath /users/visics/jknopp/libraries/mat_kdtree/src/

addpath utils_3d
addpath ~jknopp/libraries/cvx/  %%% 3rd layer needs cvx...

addpath /users/visics/jknopp/libraries/export_fig/ %%% library for exporting figures




cvx_setup