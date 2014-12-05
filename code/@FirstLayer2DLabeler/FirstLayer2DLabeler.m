classdef FirstLayer2DLabeler < handle
    %FirstLayer2DLabeler Base class for semantic classification of 2D
    %facade imagery.
    %   Detailed explanation goes here
    
    properties
            config = [];   
            baseName = [];
            
            classifierName = 'SVM';
            
            detectors = { };
            
            % List of images
            evalListFilename = [];
            trainListFilename = []; 
            allListFilename = []; 
            
            % Paths
            edisonLoc = 'edison/';
            lasicLoc = '/esat/sadr/amartino/atlas/lasik-2.4/';
            biclopLoc = '/esat/sadr/amartino/atlas/biclop/';
            
            % Parameters
            nClasses = [];
            ignoreClasses = [];
            nFeats = 225;
            resizedImagesHeight = 800;
            minRegionArea = 150;
            
            svm = struct(...
                'log2c',-1,...
                'log2g',-3,...
                'maxTrainExamples',1e6,...
                'maxTestExamples',1e3...
                );
            
            crf = struct(...
                'weightSegmentationUnary', [],...
                'weightPairwise', [],...
                'labelcost',[]);
            
            pclName = [];
    end
    
    methods
        %% Constructor
        function fl = FirstLayer2DLabeler(datasetConfig)
            
            fl.config = datasetConfig;
            fl.baseName = datasetConfig.name;
            fl.nClasses = datasetConfig.nClasses;
            
            fl.Configure();
        end
        
        
        %% Methods
        PrepareData(obj);
        
        filenames = LoadFilenames(obj,subset);
        
        [t,x,segsPerImage,imageNames] = LoadData(obj,subset);
        
        TrainSVM(obj);
        
        ClassifyWithSVM(obj);
        
        LabelImagesATLAS(obj);
        
        Project2DOntoPointCloud(obj);
        

        %% Builtin methods
        function Configure(obj)
            
            % Detectors
            obj.detectors{1}.name = 'window-generic';
            obj.detectors{1}.class = 1;
            obj.detectors{1}.configFile = '/esat/sadr/amartino/atlas/detector/configs/window-generic.ini';
            obj.detectors{1}.modelFile = '/esat/sadr/amartino/atlas/detector/models/window-generic.bin';
            obj.detectors{1}.crfWeight = 0.4029 ;
            
            % CRF
            obj.crf.weightSegmentationUnary = 0.2450;
            obj.crf.weightPairwise = 0.3725;
            
            l = load('config/haussmannLabelCost.mat');
            obj.crf.labelCost = l.labelCost(1:obj.config.nClasses,1:obj.config.nClasses);
           
        end
        
       
        % Runs Hayko's evaluation on 2D image labeling
        function [scoreL1, scoreL2] = EvaluateLabeling(obj)
           
            outputFolder1 = [get_adr('work',obj.config) 'classifier/' obj.classifierName '/layer1/'];
            scoreL1 = obj.EvaluateImageLabeling(outputFolder1);

            outputFolder2 = [get_adr('work',obj.config) 'classifier/' obj.classifierName '/layer2/'];
            scoreL2 = obj.EvaluateImageLabeling(outputFolder2);
            
        end
        
        function score = EvaluateImageLabeling(obj,outputFolder)
            file_str_idx = obj.LoadFilenames('eval');
            numViews = length(file_str_idx);
            display(['Evaluating on ' num2str(numViews) ' images.'])
            

            %% EVALUATION TASK 1 - Image labeling - vanilla 2d img labeling task
            fprintf('Loading data...\n');
            [gt, res] = evaluation_load_folder (get_adr('2D_labels',obj.config), outputFolder, {file_str_idx}, numViews, obj.config.cm);
            fprintf('Done!\n');
            fprintf('Evaluating...\n');
            score = evaluation_multilabel(gt,res,obj.config.ignoreClasses + 1);
            disp(score);

        end
        

    end
    
end

