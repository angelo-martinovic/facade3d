classdef classifier_rf < classifier
    %UNTITLED3 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        nTrees = 100;
        useParallel = true;
        minLeaf = 30;
    end
    
    methods
     
        function obj = classifier_rf()
            obj.name = 'RF';
        end
        
        function Train(obj,y,x)
            n=length(y);
            trainExamples = min(obj.maxTrainExamples,n);
            
            obj.model = TreeBagger(obj.nTrees,x(1:trainExamples,1:end),y(1:trainExamples),...
            'Method','classification',...
            'MinLeaf',obj.minLeaf,...
            'Options',statset('UseParallel',obj.useParallel));
        end
        
        
        function [yPredict,accuracy,scores] = Evaluate(obj,y,x)
             n=length(y);
             testExamples =  min(obj.maxTestExamples,n);
             
             [~,scores] = predict(model,x(n1+1:n1+testExamples,1:end));
             
             [~,yPredict] = max(scores,2);
             accuracy = 100*sum(cprb'==y(n1+1:n1+testExamples))/testExamples;
        end
    end
    
end

