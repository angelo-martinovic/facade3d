classdef FirstLayer2DLabeler < handle
    %FirstLayer2DLabeler Base class for semantic classification of 2D
    %facade imagery.
    %   Detailed explanation goes here
    
    properties
            config = [];   
    end
    %%
    methods (Access = public)
        % Constructor
        function fl = FirstLayer2DLabeler(datasetConfig)
            fl.config = datasetConfig;
        end
        
        PrepareData(obj);
        
        filenames = LoadFilenames(obj,subset);
        
        [t,x,segsPerImage,imageNames] = LoadData(obj,subset);
        
        TrainClassifier(obj);
        
        EvaluateClassifier(obj);
        
        LabelImagesATLAS(obj);
        
        Project2DOntoPointCloud(obj);
       
        % Runs ECCV 2014 evaluation on 2D image labeling
        function [scoreL1, scoreL2] = EvaluateLabeling(obj)
           
            outputFolder1 = [get_adr('2D_classification',obj.config,obj.config.c2D.classifier.name) 'layer1/'];
            scoreL1 = obj.EvaluateImageLabeling(outputFolder1);

            outputFolder2 = [get_adr('2D_classification',obj.config,obj.config.c2D.classifier.name) 'layer2/'];
            scoreL2 = obj.EvaluateImageLabeling(outputFolder2);
            
        end
    end
    %%
    methods(Access = private)
        function score = EvaluateImageLabeling(obj,outputFolder)
            dl = DispatchingLogger.getInstance();
            
            imageNames = obj.LoadFilenames('eval');
            numViews = length(imageNames);
            dl.Log(VerbosityLevel.Info,sprintf('Evaluating on %d images.\n',numViews));
            
            % EVALUATION TASK 1 - Image labeling - vanilla 2d img labeling task
            dl.Log(VerbosityLevel.Info,sprintf('Loading data...\n'));
            
            [gt, res] = evaluation_load_folder (get_adr('2D_labels',obj.config), outputFolder,...
                {imageNames}, numViews, obj.config.cm);
            
            dl.Log(VerbosityLevel.Info,sprintf('Done! Evaluating...\n'));
            score = evaluation_multilabel(gt,res,obj.config.ignoreClasses + 1);
            dl.Log(VerbosityLevel.Info,sprintf('Done!\n'));
            
            % Save the output
            save([outputFolder 'results.mat'],'score');
            
            dl.Log(VerbosityLevel.Info,...
                sprintf('Evaluated the 2D labeling of %d images in %s. Pascal IoU score: %.2f.\n',...
                numViews,outputFolder,score.mean_pascal));
  
        end
        
    end
    
end
