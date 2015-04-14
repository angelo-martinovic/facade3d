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
        
    end
    %%
    methods(Access = private)
        [t,x,segsPerImage,imageNames] = LoadData(obj,subset);
        
    end
    
end
