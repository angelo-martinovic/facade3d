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

        TrainClassifier(obj);
        
        EvaluateClassifier(obj);
        
        LabelImagesATLAS(obj);
        
        [f1,f2] = Project2DOntoPointCloud(obj);
       
        
        function outputFolder = GetOutputFolderLayer1(obj)
            outputFolder = [get_adr('2D_classification',obj.config,obj.config.c2D.classifier.name) 'layer1/'];
        end
        
        function outputFolder = GetOutputFolderLayer2(obj)
            outputFolder = [get_adr('2D_classification',obj.config,obj.config.c2D.classifier.name) 'layer2/'];
        end
        
        function outputFile = GetOutputProjectedLayer1(obj)
            outputFile = get_adr('pcl_labeling',obj.config,'2D_layer1_majorityVote');
        end
        
        function outputFile = GetOutputProjectedLayer2(obj)
            outputFile = get_adr('pcl_labeling',obj.config,'2D_layer2_majorityVote');
        end
        
    end
    %%
    methods(Access = private)
        [t,x,segsPerImage,imageNames] = LoadData(obj,subset);
        
    end
    
end
