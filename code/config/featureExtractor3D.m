classdef featureExtractor3D < handle
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = protected)
        name
    end
    
    methods

    end
    
    methods (Abstract)
        ExtractFeatures(obj);
    end
    
end