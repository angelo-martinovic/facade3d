classdef scaling < handle
    %UNTITLED3 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        rangeAll=[];
        sumAll=[];
    end
    
    methods
        function obj = scaling(x)
            maxAll = max(x,[],1);
            minAll = min(x,[],1);
            obj.rangeAll = maxAll-minAll;
            obj.sumAll = maxAll+minAll; 
        end
        
        function xScaled = Scale(obj,x)
            xScaled = x;
            
            if isempty(obj.rangeAll) || isempty(obj.sumAll)
                warning('Scaling not initialized!');
                return;
            end

            % Scale to [-1,1]
            xScaled = 2 * xScaled;
            xScaled = bsxfun(@minus, xScaled, obj.sumAll);
            xScaled = bsxfun(@rdivide, xScaled, obj.rangeAll);
        end

    end
    
    
    
end

