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
% TASK 1 - Image Labelling - vanilla 2d img labelling task
% TASK 2 - Mesh Labelling - collect or reason in 3d to label mesh (THIS CODE)
% TASK 3 - Pointcloud Labelling - collect or reason in 3d to label point cloud
% TASK 4 - View selection - reason which images to skip for features & classification via view reduction (ECCV paper)
% TASK 5 - Scene Coverage - reason which images to skip for features & classification via scene coverage (ECCV paper)
%
% eccv14 paper collects all 2d features into all 3d mesh faces and trains on 3d triangles (lab, mr8, height, normal, dominant depth)
% eccv14 paper classifies each triangle, and then selects all or best for view reduction / scene coverage
% eccv14 paper reports TASK 2 (37.33%), TASK 4 (11.9x faster) and TASK 5 (7.1x faster)
%
% this code shows ONLY TASK 2 for pure 3D mesh labelling and provides data and protocol to:
% thos code loads 202 test 2D predictions (raw, png) into mesh and evaluates baseline GC SUMALL directly in 3d mesh
%
% PROTOCOL ECCV PAPER   :  train set 119 images, test set 196 images, evaluate on 3d mesh directly (15.5 buildings)
% PROTOCOL NEW THIS CODE:  train set 113 images, test set 202 images, evaluate on 3d mesh directly (16 full buildings)
%                          test set uses all 202 images (not just 106 labelled) (IMG_5504 till IMG_5705, all images additional to folder test)
%
% DATA FORMAT:
% data/images       - raw rgb images as jpg
% data/index        - triangle index from image to mesh
% data/labels       - GT labels for training / testing images
% data/mesh.ply     - surface mesh used for 3d representation
%
% data/probraw/     - eccv raw probabilities
% data/probpng/     - eccv MAP label maps
% data/probgco/     - eccv GCO label maps
%
% listall.txt               - all 428 images
% listeval_new_full.txt     - 202 test evaluation
% listtrain_new.txt         - 113 train 
%
% data/mesh_gt_new.ply      - 16 buildings (index bug fixed, +6 testing images more)
% data/mesh_gt_old.ply      - 15.5 buildings (index bug fixed)
% data/mesh_gt_eccv14.ply   - 15.5 buildings (index bug)
%
% RESULTS OVERVIEW          TASK 2 - Mesh Labelling - collect or reason in 3d to label mesh (THIS CODE)
%                           NOTE:  THIS ARE THE BASELINES ONLY (GC SUMALL), NO CODE OR RESULTS FOR THE ECCV PAPER TASK 4 and 5!
%
% GT eccv14 paper           37.33% pascal IOU accuracy (baselines, GC SUMALL)
% GT eccv14 recode          41.92% pascal IOU accuracy (fixed index bug)
% GT new train/test split   todo% pascal IOU accuracy (6 more testing images something ~42.33% (old train set))
% GT eccv14 recode feat     53.16% pascal IOU accuracy (slower more features than only LAB, MR8, etc)
%
% CODE OVERVIEW
%  
% EVALUATION    - evaluation_multilabel.m, evaluation_multilabel_print.m
% COLLECTION 3D - eccv2014_mesh_collect_files.m, mesh_collect_raw.m, mesh_collect_labelmap.m
% GRAPHCUT 3D   - mesh_adjacency_onehit.m, conversion_likelihood2cost.m, ICG_Graphcut_GCO.m
% MISC          - progressbartime.m, rgb2labelmap.m, export_ply_simple.m
%
% REQUIREMENTS: GCO graphcut library from http://vision.csd.uwo.ca/code/gco-v3.0.zip
%
% CHANGES
% v1.00 - reimplementation of loading and evaluation codes for public
%       - changes in training / testing split (old covered 15.5 vs. new 16 buildings).
%       - bug fix in gt (smoothing, missed some triangles)
% 
% AUTHOR: Hayko Riemenschneider, 2014
%
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% SUMMARY
% OWN MESH LABELING?
%   load('../data/mesh_gt_test_new.mat'); 
%   score_new = evaluation_multilabel(gt_test-1,predictprob_sum_labels-1,[1 9])
% OWN FUSION?
%   load('eccv14_recode_predictprob.mat','data')
% OWN TRAINING?
%   produce .mat or .png files and use code to load
%   produce data.predictprob [202 x 8 x numFaces]
 

close all, clear all

%% LOAD MESH and ground truth

load('../data/mesh.mat','face','vertex','cm')
numFaces =  size(face,2);
numVertices =  size(vertex,2);
numLabels = size(cm,1);


%% SETUP PATHS

filelist_all = 'listall.txt';
filelist_train = 'listtrain_new.txt';
filelist_eval = 'listeval_new_full.txt';
filelist_str = filelist_eval;

% LOAD DATA FILE NAMES & INDEX
fid = fopen(['../data/' filelist_str]);
file_str_idx = textscan(fid, '%s'); fclose(fid);
numViews = length(file_str_idx{1});
display(['found ' num2str(numViews) ' files.'])

path.images = '../data/images/';
path.index = '../data/index/';
path.train = '../data/labels/';
path.test = '../data/labels/';

path.predictprob = '../data/probraw/';
path.predicttest = '../data/probmap/';
path.predicttest = '../data/probgco/';

path.predictprob = '/scratch_net/biwisrv01_second/varcity/dataout/semseg/paris_ruemonge/ruemonge428all/new_out/';

%% DATA COLLECTION

data = eccv2014_mesh_collect_files (path, file_str_idx, numViews, numLabels, numFaces, 'predicttest');
%save('eccv14_recode_predictprob.mat','data','-v7.3')
  
%% DATA FUSION (BASELINE ONLY)

% sum over all probabilities from all views (eccv14 paper: SUM ALL)
data.predictprob_sum = squeeze(sum(data.predictprob,1))';


%% GENERATE 3D GRAPH RESULT

load('../data/mesh_gt_test_new.mat'); 
addpath(genpath('gco'))

alpha = 0.1;
pairwise = mesh_adjacency_onehit(face, vertex);
unary = conversion_likelihood2cost(data.predictprob_sum, false)';
predictprob_sum_labels = ICG_Graphcut_GCO(unary, pairwise, alpha, [], [], true, 100000)';


%% EVALUATION TASK 2 - Mesh Labelling - collect or reason in 3d to label mesh (THIS CODE)

score_new = evaluation_multilabel(gt_test-1,predictprob_sum_labels-1,[1 9])
export_ply_simple( '../data/mesh_predict.ply', vertex, face, cm(predictprob_sum_labels,:)'*255)



