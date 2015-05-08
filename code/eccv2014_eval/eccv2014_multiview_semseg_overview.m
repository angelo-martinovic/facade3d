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
% TASK 1 - Image Labelling - vanilla 2d img labelling task (NEW TASK && INCLUDED CODE)
% TASK 2 - Mesh Labelling - collect or reason in 3d to label mesh (ECCV paper INCLUDED CODE)
% TASK 3 - Pointcloud Labelling - collect or reason in 3d to label point cloud (NEW TASK)
% TASK 4 - View selection - reason which images to skip for features & classification via view reduction (ECCV paper)
% TASK 5 - Scene Coverage - reason which images to skip for features & classification via scene coverage (ECCV paper)
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


% TASK 1 - Image Labelling - vanilla 2d img labelling task
eccv2014_run_task1

% TASK 2 - Mesh Labelling - collect or reason in 3d to label mesh
eccv2014_run_task2b


