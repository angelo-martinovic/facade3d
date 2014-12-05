function datasetConfig = InitializeDataset(datasetName)

    % General
    datasetConfig.name              = datasetName;
    datasetConfig.nClasses          = 7;
    datasetConfig.ignoreClasses     = [0 8];
    datasetConfig.cm                = HaussmannColormap()/255;  
    
    datasetConfig.parallel.enabled  = true;
    datasetConfig.parallel.nWorkers = 2;
    
    
    datasetConfig.dataLocation      = '../data/'; 
    datasetConfig.outputLocation    = '../output/';
    
    % 2D
    datasetConfig.imageLocation     = 'images/';    % Relative to dataLocation
    datasetConfig.labelLocation     = 'labels/';
    
        % Train-test split
        datasetConfig.trainList         = 'listtrain.txt';
        datasetConfig.evalList          = 'listeval.txt';
        datasetConfig.fullList          = 'listall.txt';
        
    
    
    % 3D
    datasetConfig.pointCloud        = 'pcl.ply';
    datasetConfig.cameras           = 'cameras.txt';
    datasetConfig.groundTruthTrain  = 'pcl_gt_train.ply';
    datasetConfig.groundTruthTest   = 'pcl_gt_test.ply';
    datasetConfig.splitData         = 'pcl_split.mat';  % Generated
    datasetConfig.depth             = 'pcl_depth.mat';  % Generated 
    
    
    % Parameters
                                 %[3D 2D det pairwise]
    datasetConfig.CRF3D.weights = [0 1 0 1];
    
    
%     path_mat_data_dir         = '/esat/sadr/amartino/monge428New/data/';
%     postfix_path_data_orig{1} = 'pcloud_gt_train_new_GCO_old.ply';   %%% train
%     postfix_path_data_orig{2} = 'pcloud_gt_test_new_GCO_old.ply';    %%% test
%     postfix_path_data_orig{3} = 'splitData.mat';                     %%% full pcl
%     postfix_path_data_orig{4} = 'mesh_colors_normals.ply';%pcloud_colors.ply';           
%     postfix_path_data_orig{5} = 'mesh_depth.mat';                    %%% depth
    %postfix_path_data_orig{6} = '/esat/sadr/amartino/monge428/reconstruction.nvm.cmvs/00/cameras_v2.txt';

  


end

