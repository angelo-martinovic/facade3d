classdef FirstLayer2DLabeler < handle
    %FirstLayer2DLabeler Base class for semantic classification of 2D
    %facade imagery.
    %   Detailed explanation goes here
    
    properties

    end
    %%
    methods (Static)

        
        PrepareData();

        TrainClassifier();
        
        RunClassifier();
        
        LabelImagesATLAS();
        
        [f1,f2] = Project2DOntoPointCloud();
       
        
        function outputFolder = GetOutputFolderLayer1()
            cf = DatasetConfig.getInstance();
            outputFolder = [get_adr('2D_classification',cf.c2D.classifier.name) 'layer1/'];
        end
        
        function outputFolder = GetOutputFolderLayer2()
            cf = DatasetConfig.getInstance();
            outputFolder = [get_adr('2D_classification',cf.c2D.classifier.name) 'layer2/'];
        end
        
        function outputFile = GetOutputProjectedLayer1()
            outputFile = get_adr('pcl_labeling','2D_layer1_majorityVote');
        end
        
        function outputFile = GetOutputProjectedLayer2()
            outputFile = get_adr('pcl_labeling','2D_layer2_majorityVote');
        end
        
    end
    %%
    methods(Access = private)
        [t,x,segsPerImage,imageNames] = LoadData(subset);
        
    end
   
    
end
