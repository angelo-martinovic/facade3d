classdef c2D_SVM_cnn_gould < c2D
    %C2D_SVM_CNN Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
    end
    
    methods
        function c2D = c2D_SVM_cnn_gould(config)
            c2D.name = 'SVM_cnn_gould';
            
            f1 = featureExtractorCNN(config);
            f2 = featureExtractorGould(config);
            c2D.featureExtractors = { f1 f2 };
            
            c2D.resizedImagesHeight = 800; 
            c2D.minRegionArea = 150;

            c2D.classifier = classifier_linearSVM();
            
            % Detectors
            c2D.detectors = { };

            % CRF
            c2D.crf = crf2D();
            c2D.crf.weightUnarySegmentation = 1;%0.2450;
            c2D.crf.weightPairwise = 0.1;%0.3725;
            
            l = load('config/haussmannLabelCost.mat');
            c2D.crf.labelCost = l.labelCost(1:config.nClasses,1:config.nClasses); 
        end
    end
    
end

