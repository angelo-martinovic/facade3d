classdef ILogger < handle
    %ILogger Interface for logger classes
    %   Detailed explanation goes here
    
    properties
        
    end
    
    methods (Abstract)
        Log(severity,message);
    end
    
end

