%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Learning Where To Classify In Multi-View Semantic Segmentation
% ETHZ RueMonge 2014 dataset + load/ evaluation code
%
% Learning Where To Classify In Multi-View Semantic Segmentation
% ECCV 2014, Zurich, Switzerland. (PDF, poster, project)
% H. Riemenschneider, A. Bodis-Szomoru, J. Weissenberg, L. Van Gool
%
% http://www.varcity.eu
% http://www.vision.ee.ethz.ch/~rhayko/paper/eccv2014_riemenschneider_multiviewsemseg/
%
% Please cite the above work if you use any of the code or dataset.
%

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% OVERVIEW
%
% EVALUATION
% TASK 1 - Image Labelling - vanilla 2d img labelling task (THIS CODE)
% TASK 2 - Mesh Labelling - collect or reason in 3d to label mesh 
% TASK 3 - Pointcloud Labelling - collect or reason in 3d to label point cloud
% TASK 4 - View selection - reason which images to skip for features & classification via view reduction (ECCV paper)
% TASK 5 - Scene Coverage - reason which images to skip for features & classification via scene coverage (ECCV paper)
%
% this code shows ONLY TASK 1 for pure 2D image labelling and provides data and protocol to:
% thiss code loads 119 test 2D predictions (raw, png) and evaluates directly on 2D images (vanilla style)
%
% PROTOCOL:  train set 113 images, test set 119 images, evaluate on 2d images directly (no 3D)
%
% DATA FORMAT:
% data/images       - raw rgb images as jpg
% data/labels       - GT labels for training / testing images
%
% data/probraw/     - eccv raw probabilities (very large, not included)
% data/probpng/     - eccv MAP label maps
% data/probgco/     - eccv GCO label maps
%
% listall.txt               - all 428 images
% listeval_new_full.txt     - 202 test evaluation
% listtrain_new.txt         - 113 train 
%

% RESULTS OVERVIEW          TASK 1 - Image Labelling - vanilla 2d img labelling task (THIS CODE)
%
% eccv14 feat MAP           38.72% pascal IOU accuracy (only  LAB, MR8, etc)
% eccv14 feat GCO           40.92% pascal IOU accuracy (only  LAB, MR8, etc)
% recode feat MAP           50.73% pascal IOU accuracy (slower more features than only LAB, MR8, etc)
% recode feat GCO           52.96% pascal IOU accuracy (slower more features than only LAB, MR8, etc)
%
% CODE OVERVIEW
%  
% EVALUATION    - evaluation_multilabel.m, evaluation_multilabel_print.m
% GRAPHCUT 2D   - ICG_Graphcut_GCO.m
% MISC          - progressbartime.m, rgb2labelmap.m
%
% REQUIREMENTS: GCO graphcut library from http://vision.csd.uwo.ca/code/gco-v3.0.zip
%
% CHANGES
% v1.00 - reimplementation of evaluation codes for public
%
% AUTHOR: Hayko Riemenschneider, 2014
%
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all, clear all

%% SETUP PATHS
dataLoc = '/esat/sadr/amartino/monge428New/';
cm = HaussmannColormap();
% load([dataLoc 'data/' mesh.mat,'cm') % load color map

filelist_all = 'listall.txt';
filelist_train = 'listtrain_new.txt';
filelist_eval = 'listeval_new_full.txt';
filelist_str = filelist_eval;

% LOAD DATA FILE NAMES & INDEX
fid = fopen([dataLoc 'data/' filelist_str]);
file_str_idx = textscan(fid, '%s'); fclose(fid);
numViews = length(file_str_idx{1});
display(['found ' num2str(numViews) ' files.'])

path.images = [dataLoc 'data/images/'];
path.train = [dataLoc 'data/labels/'];
path.test = [dataLoc 'data/labels/'];

path.predictprob = [dataLoc 'data/probraw/']; % raw input
path.predictmap = [dataLoc 'data/work/probmap_l1/']; % can be generated
path.predictgco = [dataLoc 'data/work/probmap_l2/']; % can be generared

       
%% EVALUATION TASK 1 - Image Labelling - vanilla 2d img labelling task (THIS CODE)

% DATA COLLECTION 2D MAP
% [gt, res] = evaluation_load_folder (path.test, path.predictmap, file_str_idx, numViews, cm);
% score_map = evaluation_multilabel(gt,res,[1 9])


% DATA COLLECTION 2D GCO
% [gt, res] = evaluation_load_folder (path.test, path.predictgco, file_str_idx, numViews, cm);
% score_gco = evaluation_multilabel(gt,res,[1 9])


% DATA COLLECTIOn 2D PROB (generate MAP and GCO)
[gt, res] = evaluation_load_folder (path.test, path.predictmap, file_str_idx, numViews, cm);
score_gco = evaluation_multilabel(gt,res,[1 9])

[gt, res] = evaluation_load_folder (path.test, path.predictgco, file_str_idx, numViews, cm);
score_gco = evaluation_multilabel(gt,res,[1 9])

