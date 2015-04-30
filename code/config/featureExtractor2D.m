classdef featureExtractor2D < handle
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = protected)
        workFolder
        name
        nFeats
        scaleFeats
    end
    
    methods
        function F = featureExtractor2D(workFolder)
            F.workFolder = workFolder;
        end
    end
    
    methods (Abstract)
        ExtractFeatures(obj);
    end
    
end

