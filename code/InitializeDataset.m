function datasetConfig = InitializeDataset(datasetName)

    % Logging
    sl = ScreenLogger(VerbosityLevel.Verbose);
    fl = FileLogger(VerbosityLevel.Info,['log_' datasetName '.txt']);
    
    dl = DispatchingLogger.getInstance();
    dl.Clear();
    dl.Subscribe(sl);
    dl.Subscribe(fl);
    
    datasetConfig.nWorkers = 0;    % Set to 0 to run everything in serial on 1 CPU
   
    
    %% General
    datasetConfig.name              = datasetName;
    datasetConfig.useCache          = false;
    
    if strcmp(datasetName,'monge428')
        datasetConfig.nClasses          = 7;
        datasetConfig.ignoreClasses     = [0 8];
        datasetConfig.cm                = HaussmannColormap()/255;  
        datasetConfig.dataLocation      = '../data/'; 
        datasetConfig.outputLocation    = '../output/';
        datasetConfig.rectificationNeeded  = true;
        datasetConfig.resizeImages      = true;
        
    elseif strcmp(datasetName,'monge428FullRes')
        datasetConfig.nClasses          = 7;
        datasetConfig.ignoreClasses     = [0 8];
        datasetConfig.cm                = HaussmannColormap()/255;  
        datasetConfig.dataLocation      = '../dataFullRes/'; 
        datasetConfig.outputLocation    = '../outputFullRes/';
        datasetConfig.rectificationNeeded  = true;
        datasetConfig.resizeImages      = true;
        
    elseif strcmp(datasetName,'monge428SfM')
        datasetConfig.nClasses          = 7;
        datasetConfig.ignoreClasses     = [0 8];
        datasetConfig.cm                = HaussmannColormap()/255;  
        datasetConfig.dataLocation      = '../dataSfM/'; 
        datasetConfig.outputLocation    = '../outputSfM/';
        datasetConfig.rectificationNeeded  = true;
        datasetConfig.resizeImages      = true;
        
    elseif strcmp(datasetName,'monge428PMVS')
        datasetConfig.nClasses          = 7;
        datasetConfig.ignoreClasses     = [0 8];
        datasetConfig.cm                = HaussmannColormap()/255;  
        datasetConfig.dataLocation      = '../dataPMVS/'; 
        datasetConfig.outputLocation    = '../outputPMVS/';
        datasetConfig.rectificationNeeded  = true;
        datasetConfig.resizeImages      = true;
        
    elseif strcmp(datasetName,'nanZubud')
        datasetConfig.nClasses          = 2;
        datasetConfig.ignoreClasses     = 0;
        datasetConfig.cm                = NanColormap()/255;
        datasetConfig.dataLocation      = '../dataZubud/'; 
        datasetConfig.outputLocation    = '../outputZubud/';
        datasetConfig.rectificationNeeded  = false;
        datasetConfig.resizeImages      = false;
        
    elseif strcmp(datasetName,'nanGraz')
        datasetConfig.nClasses          = 2;
        datasetConfig.ignoreClasses     = 0;
        datasetConfig.cm                = NanColormap()/255;
        datasetConfig.dataLocation      = '../dataGraz/'; 
        datasetConfig.outputLocation    = '../outputGraz/';
        datasetConfig.rectificationNeeded  = false;
        datasetConfig.resizeImages      = false;
        
    elseif strcmp(datasetName,'nanStrasbourg')
        datasetConfig.nClasses          = 2;
        datasetConfig.ignoreClasses     = 0;
        datasetConfig.cm                = NanColormap()/255;
        datasetConfig.dataLocation      = '../dataStrasbourg/'; 
        datasetConfig.outputLocation    = '../outputStrasbourg/';
        datasetConfig.rectificationNeeded  = false;
        datasetConfig.resizeImages      = false;
        
    elseif strcmp(datasetName,'nanLondon')
        datasetConfig.nClasses          = 2;
        datasetConfig.ignoreClasses     = 0;
        datasetConfig.cm                = NanColormap()/255;
        datasetConfig.dataLocation      = '../dataLondon/'; 
        datasetConfig.outputLocation    = '../outputLondon/';
        datasetConfig.rectificationNeeded  = false;
        datasetConfig.resizeImages      = false;
    end
  
    datasetConfig.imageLocation     = 'images/';    % Relative to dataLocation
    datasetConfig.labelLocation     = 'labels/';
    
    % Train-test split
    datasetConfig.trainList         = 'listtrain.txt';
    datasetConfig.evalList          = 'listeval.txt';
    datasetConfig.fullList          = 'listall.txt';
    
    datasetConfig.pointCloud        = 'pcl.ply';
    datasetConfig.cameras           = 'cameras.txt';
    datasetConfig.groundTruthTrain  = 'pcl_gt_train.ply';
    datasetConfig.groundTruthTest   = 'pcl_gt_test.ply';
    datasetConfig.splitData         = 'pcl_split.mat';  % Generated - TODO!
    datasetConfig.depth             = 'pcl_depth.mat';  % Generated - TODO!
    
    workFolder = [datasetConfig.outputLocation 'work/'];
    nClasses = datasetConfig.nClasses;
    
    %% Semantic image segmentation - configuration
    selector = 1;
    
    if selector==1
        % ---- CVPR15 submission version ----
        % Gould region features, SVM classification, window detector
        datasetConfig.c2D = ConfigFactory.Make2DConfig(...
            workFolder,nClasses,'nonlinearSVM','gould','windowDetector',[0.245 0.4029 0.3725]);
    
    elseif selector==2
        % Gould region features, SVM classification, no detector
        % datasetConfig.c2D = c2D_SVM_gould(datasetConfig);
        datasetConfig.c2D = ConfigFactory.Make2DConfig(...
            workFolder,nClasses,'nonlinearSVM','gould','none',[1 0.1]);
    
    elseif selector==3
        % CNN region features, SVM classification, window detector
        % datasetConfig.c2D = c2D_SVM_cnn_winDet(datasetConfig);
        datasetConfig.c2D = ConfigFactory.Make2DConfig(...
            workFolder,nClasses,'linearSVM','CNN','windowDetector',[1 0.4029 0.1]);
    
    elseif selector==4
        % CNN region features, SVM classification, no detector
        %  datasetConfig.c2D = c2D_SVM_cnn(datasetConfig);     
        datasetConfig.c2D = ConfigFactory.Make2DConfig(...
            workFolder,nClasses,'linearSVM','CNN','none',[1 0.1]);
    end
            
    %% Semantic point cloud segmentation - configuration
    selector = 1;
    
    if selector==1
        % Only 3D classification
         datasetConfig.c3D = ConfigFactory.Make3DConfig(...
            'rf','spinImage','none',[1 0 0.3725]);
        
    elseif selector==2
        % Only 2D classification projected
         datasetConfig.c3D = ConfigFactory.Make3DConfig(...
            'rf','2D','none',[0 1 0.3725]);
        
    elseif selector==3
        % Only 2D classification + 2D window detector projected
        datasetConfig.c3D = ConfigFactory.Make3DConfig(...
            'rf','2D','windowDetector',[0 1 1 0.3725]);
        
    elseif selector==4
        % 3D + 2D classification
        datasetConfig.c3D = ConfigFactory.Make3DConfig(...
            'rf','spinImage+2D','none',[1 1 0.3725]);
        
    elseif selector==5
         % 3D + 2D classification + 2D window detector
         datasetConfig.c3D = ConfigFactory.Make3DConfig(...
            'rf','spinImage+2D','windowDetector',[1 1 1 0.3725]);
         
    end


end

