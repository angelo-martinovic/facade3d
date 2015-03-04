classdef classifier_linearSVM < classifier
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
    end
    
    methods
        function obj = classifier_linearSVM()
            obj.name = 'linSVM';
        end
        
        function Train(obj,y,x)
            n=length(y);
            trainExamples = min(obj.maxTrainExamples,n);
            cmd = '-s 2'; 
            obj.model = train(y(1:trainExamples), sparse(x(1:trainExamples,:)) ,cmd);
        end
        
        
        function [yPred,accuracy,yProb] = Evaluate(obj,y,x)
             n=length(y);
             testExamples =  min(obj.maxTestExamples,n);
             [yPred, accuracy, yProb] = predict(y(1:testExamples), sparse(x(1:testExamples,:)), obj.model, '-b 1 -q');
             
             % Re-order classes based on model order
             yProb(:,obj.model.Label) = yProb;
             accuracy = accuracy(1);
        end
        
        
    end
    
end

