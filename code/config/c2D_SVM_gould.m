classdef c2D_SVM_gould < c2D
    %C2D_SVM_CNN Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
    end
    
    methods
        function c2D = c2D_SVM_gould(config)
            c2D.name = 'SVM_gould';
            
            f1 = featureExtractorGould(config);
            c2D.featureExtractors = { f1 };
            
            c2D.resizedImagesHeight = 800; 
            c2D.minRegionArea = 150;

%             c2D.classifier = classifier_linearSVM();
            c2D.classifier = classifier_nonlinearSVM();
            
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

