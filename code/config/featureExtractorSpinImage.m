classdef featureExtractorSpinImage < featureExtractor3D
    %featureExtractorCNN Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        si_dimensions = [.15 .3 .45];
        binSize = 8;
        imgW = 10;
    end
    
    methods
        function F = featureExtractorSpinImage(config)
            
            F@featureExtractor3D(config); 
            F.name = 'spinImage';
        end
        
        function ExtractFeatures(obj)
            error('Not yet implemented through this interface.');
        end
    end
    
   
    
end

