classdef c3D_2D< c3D
    %C2D_SVM_CNN Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
    end
    
    methods
        function c3D = c3D_2D()
            c3D.name = '2D';
            
            % CRF
            c3D.crf = crf3D();
            c3D.crf.weight2DClassification = 1;
            c3D.crf.weights2DDetectors = [];
            c3D.crf.weight3DClassification = 0;
            c3D.crf.weightPairwise = 1;
            
        
        end
    end
    
end

