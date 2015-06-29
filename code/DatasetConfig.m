classdef (Sealed) DatasetConfig < handle
    %   Singleton class.
    
    properties 
        nWorkers = 8;    % Set to 0 to run everything in serial on 1 CPU

        name                = [];
        useCache            = [];
        
        nClasses            = [];
        ignoreClasses       = [];
        cm                  = [];
        dataLocation        = [];
        outputLocation      = [];
        rectificationNeeded = [];
        resizeImages        = [];
        
        imageLocation       = 'images/';    % Relative to dataLocation
        labelLocation       = 'labels/';
        
        % Train-test split
        trainList           = 'listtrain.txt';
        evalList            = 'listeval.txt';
        fullList            = 'listall.txt';
        
        pointCloud          = 'pcl.ply';
        cameras             = 'cameras.txt';
        groundTruthTrain    = 'pcl_gt_train.ply';
        groundTruthTest     = 'pcl_gt_test.ply';
        splitData           = 'pcl_split.mat';  % Generated - TODO!
        depth               = 'pcl_depth.mat';  % Generated
        
        c2D = [];
        c3D = [];
    end
     
        
    methods (Access = private)
      function datasetConfig = DatasetConfig(datasetName)

        datasetConfig.name              = datasetName;
        datasetConfig.useCache          = true;

        if strcmp(datasetName,'mongeToy')
            datasetConfig.nWorkers          = 0;     % Run in serial for the toy dataset
            datasetConfig.nClasses          = 7;
            datasetConfig.ignoreClasses     = [0 8];
            datasetConfig.cm                = HaussmannColormap()/255;  
            datasetConfig.dataLocation      = '../dataToy/'; 
            datasetConfig.outputLocation    = '../outputToy/';
            datasetConfig.rectificationNeeded  = true;
            datasetConfig.resizeImages      = true;
            
        elseif strcmp(datasetName,'monge428')
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

        workFolder = [datasetConfig.outputLocation 'work/'];
        
        %% Semantic image segmentation - configuration
        selector = 1;

        if selector==1
            % ---- CVPR15 submission version ----
            % Gould region features, RBF SVM classification, window detector
            datasetConfig.c2D = ConfigFactory.Make2DConfig(...
                workFolder,datasetConfig.nClasses,'nonlinearSVM','gould','windowDetector',[0.245 0.4029 0.3725]);

        elseif selector==2
            % Gould region features, RBF SVM classification, no detector
            datasetConfig.c2D = ConfigFactory.Make2DConfig(...
                workFolder,datasetConfig.nClasses,'nonlinearSVM','gould','none',[1 0.1]);

        elseif selector==3
            % CNN region features, linear SVM classification, window detector
            datasetConfig.c2D = ConfigFactory.Make2DConfig(...
                workFolder,datasetConfig.nClasses,'linearSVM','CNN','windowDetector',[1 0.4029 0.1]);

        elseif selector==4
            % CNN region features, linear SVM classification, no detector
            datasetConfig.c2D = ConfigFactory.Make2DConfig(...
                workFolder,datasetConfig.nClasses,'linearSVM','CNN','none',[1 0.1]);
            
        elseif selector==5
            % CNN+Gould region features, linear SVM classification, no detector
            datasetConfig.c2D = ConfigFactory.Make2DConfig(...
                workFolder,datasetConfig.nClasses,'linearSVM','CNN+gould','none',[1 0.1]);
            
        elseif selector==6
            % CNN+Gould region features, linear SVM classification, window detector
            datasetConfig.c2D = ConfigFactory.Make2DConfig(...
                workFolder,datasetConfig.nClasses,'linearSVM','CNN+gould','windowDetector',[1 0.4029 0.3725]);
        end

        %% Semantic point cloud segmentation - configuration
        selector = 4;

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
    end
    
    methods (Access = public)

    end
    
   methods (Static)
      function singleObj = getInstance(datasetName)
         persistent localObj
         if nargin<1
             if isempty(localObj) || ~isvalid(localObj)
                 error('DatasetConfig not initialized! Call getInstance(datasetName) once!');
             else
                 singleObj = localObj;
             end
             
         elseif nargin==1
                localObj = DatasetConfig(datasetName);
                singleObj = localObj;
         else
             error('Usage: getInstance([datasetName])');
         end
         
         
      end
      
      
   end
    
end