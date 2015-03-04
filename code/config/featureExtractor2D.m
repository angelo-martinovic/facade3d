classdef featureExtractor2D < handle
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = protected)
        config
        name
        nFeats
        scaleFeats
    end
    
    methods
        function F = featureExtractor2D(config)
            F.config = config;
        end
    end
    
    methods (Abstract)
        ExtractFeatures(obj);
    end
    
end

