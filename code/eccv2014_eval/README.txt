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
%
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
%
% CHANGES
% v1.00 - reimplementation of loading and evaluation codes for public
%       - changes in training / testing split (old covered 15.5 vs. new 16 buildings).
%       - bug fix in gt (smoothing, missed some triangles)


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% TASK 1 - Image Labelling - vanilla 2d img labelling task
eccv2014_run_task1

% TASK 2 - Mesh Labelling - collect or reason in 3d to label mesh
eccv2014_run_task2b
