classdef config < handle
    %CONFIG Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = protected)
        name = '';
        
        featureExtractors = {};
        
        classifier = {};

        crf = {};

    end
    
    methods
        function SetName(obj, name)
           obj.name = name; 
        end
        
        function SetClassifier(obj, classifier)
           obj.classifier = classifier; 
        end
        
        function AddFeatureExtractor(obj, featureExtractor)
           obj.featureExtractors{end+1} = featureExtractor;
        end
        
        function AddDetector(obj, detector)
           obj.detectors{end+1} = detector;
        end
        
        function SetCRF(obj, crf)
           obj.crf = crf;
        end
    end
    
end

