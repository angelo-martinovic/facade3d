classdef detector_window_generic < detector
    %DETECTOR_WINDOW_GENERIC Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
    end
    
    methods
        function D = detector_window_generic()
            D.name = 'window-generic';
            D.class = 1;
            D.configFile = 'detector/configs/window-generic.ini';
            D.modelFile = 'detector/models/window-generic.bin';
        end
    end
    
end

