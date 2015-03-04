classdef featureExtractor3D < handle
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = protected)
        config
        name
    end
    
    methods
        function F = featureExtractor3D(config)
            F.config = config;
        end
    end
    
    methods (Abstract)
        ExtractFeatures(obj);
    end
    
end