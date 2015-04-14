classdef c3D_3D_2D < c3D
    %C2D_SVM_CNN Summary of this class goes here
    %   Detailed explanation goes here
    
    properties

    end
    
    methods
        function c3D = c3D_3D_2D(config)
            c3D.name = '3D_2D';
            
            f1 = featureExtractorSpinImage(config);
            c3D.featureExtractors = { f1 };
            
            c3D.classifier = classifier3D();
            
            % CRF
            c3D.crf = crf3D();
            c3D.crf.weight2DClassification = 1;
            c3D.crf.weight3DClassification = 1;
            c3D.crf.weightPairwise = 0.3725;% 0.1;
            
           
        
        end
    end
    
end

