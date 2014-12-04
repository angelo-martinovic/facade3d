% Set paths
setup;

% Set up the dataset: file locations, file names
datasetConfig = InitializeDataset('monge428');

%% angelo
% Input: images and labels
% Output: classified images of the test set and the projected classification on the
% point cloud
FirstLayer2D(datasetConfig);

%% honza
% Input: point cloud
% Output: classified point cloud
FirstLayer3D(datasetConfig);

%% angelo
% Input: classified point clouds
% Output: combined classified point cloud 
inputName = CombineLabelings(datasetConfig);


%% angelo
% Input: classified point cloud
% Output: improved classification of point cloud
ThirdLayer2D(datasetConfig,inputName);

%% honza
% Input: classified point cloud
% Output: improved classification of point cloud
ThirdLayer3D(datasetConfig,inputName);

%%
EvaluateAll(datasetConfig);














