classdef c2D < config
    %C2D Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = protected)
        
        resizedImagesHeight = 800;
        minRegionArea = 150;

        detectors = {};    
    end
    
    methods (Access = public)

        
        function AddDetector(obj, detector)
           obj.detectors{end+1} = detector;
        end
        

        
    end
    
end

