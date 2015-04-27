classdef c2D_SVM_gould_winDet < c2D
    %C2D_SVM_CNN Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
    end
    
    methods
        function c2D = c2D_SVM_gould_winDet(config)
            c2D.name = 'SVM_gould';
            
            f1 = featureExtractorGould(config);
            c2D.featureExtractors = { f1 };

            c2D.classifier = classifier_nonlinearSVM();
            
            % Detectors
            d1 = detector_window_generic(config);
            c2D.detectors = {d1 };

            % CRF
            c2D.crf = crf2D();
            c2D.crf.weightUnarySegmentation = 0.2450;
            c2D.crf.weightsUnaryDetectors = 0.4029;
            c2D.crf.weightPairwise = 0.3725;
            
            l = load('config/haussmannLabelCost.mat');
            c2D.crf.labelCost = l.labelCost(1:config.nClasses,1:config.nClasses); 
        end
    end
    
end

