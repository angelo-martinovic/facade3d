classdef c2D < handle
    %C2D Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = protected)
        name
        
        featureExtractors
        
        resizedImagesHeight
        minRegionArea 

        classifier

        crf

        detectors       
    end
    
    methods (Access = public)
        function SetClassifier(obj, classifier)
           obj.classifier = classifier; 
        end
        
    end
    
end

