classdef classifier < handle
    %CLASSIFIER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = protected)
        model
        name   
        
        maxTrainExamples = 1e7;
        maxTestExamples = 1e7;
    end
    
    methods (Access = public)
        
    end
        
    
    methods (Abstract)
        
        Train(obj,t,x);
        [yPred,acc,yProb] = Evaluate(obj,t,x);
        
    end
    
end

