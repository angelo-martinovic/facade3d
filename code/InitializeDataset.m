function datasetConfig = InitializeDataset(datasetName)

    % General
    datasetConfig.name              = datasetName;
    datasetConfig.nClasses          = 7;
    datasetConfig.ignoreClasses     = [0 8];
    datasetConfig.cm                = HaussmannColormap()/255;  
    
    datasetConfig.nWorkers = 4;    % Set to 0 to run everything in serial on 1 CPU

    % Logging
    sl = ScreenLogger(VerbosityLevel.Verbose);
    fl = FileLogger(VerbosityLevel.Info,['log_' datasetConfig.name '.txt']);
    
    dl = DispatchingLogger.getInstance();
    dl.Clear();
    dl.Subscribe(sl);
    dl.Subscribe(fl);
   
    % Paths
    datasetConfig.useCache          = true;
    datasetConfig.rectificationNeeded  = true;
    datasetConfig.dataLocation      = '../data/'; 
    datasetConfig.outputLocation    = '../output/';
%     datasetConfig.dataLocation      = '../dataFullRes/'; 
%     datasetConfig.outputLocation    = '../outputFullRes/';
    
    
    datasetConfig.imageLocation     = 'images/';    % Relative to dataLocation
    datasetConfig.labelLocation     = 'labels/';
    
    % Train-test split
    datasetConfig.trainList         = 'listtrain.txt';
    datasetConfig.evalList          = 'listeval.txt';
    datasetConfig.fullList          = 'listall.txt';
        
    datasetConfig.skipAll3D         = false; % If true, will perform only image segmentation
   
    %% 2D
    datasetConfig.c2D = c2D_SVM_gould_winDet(datasetConfig); % CVPR15 submission version
%     datasetConfig.c2D = c2D_SVM_gould(datasetConfig);
%     datasetConfig.c2D = c2D_SVM_cnn_winDet(datasetConfig);
%     datasetConfig.c2D = c2D_SVM_cnn(datasetConfig);     
            
    %% 3D
    datasetConfig.pointCloud        = 'pcl.ply';
    datasetConfig.cameras           = 'cameras.txt';
    datasetConfig.groundTruthTrain  = 'pcl_gt_train.ply';
    datasetConfig.groundTruthTest   = 'pcl_gt_test.ply';
    datasetConfig.splitData         = 'pcl_split.mat';  % Generated
    datasetConfig.depth             = 'pcl_depth.mat';  % Generated 
    
    
    % Parameters
    datasetConfig.c3D = c3D_3D_2D(datasetConfig);
%     datasetConfig.c3D = c3D_2D();
    
    

end

