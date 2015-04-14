classdef c3D_2D_winDet< c3D
    %C2D_SVM_CNN Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
    end
    
    methods
        function c3D = c3D_2D_winDet()
            c3D.name = '2D_winDet';
            
            % CRF
            c3D.crf = crf3D();
            c3D.crf.weight2DClassification = 1;
            c3D.crf.weights2DDetectors = 1;
            c3D.crf.weight3DClassification = 0;
            c3D.crf.weightPairwise = 0.3725;%0.1;
            
        
        end
    end
    
end

