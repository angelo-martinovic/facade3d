classdef classifier_nonlinearSVM < classifier
    %classifier_nonlinearSVM libSVM classification
    %   Detailed explanation goes here
    
    properties
        log2c = -1;
        log2g = -3;
        type = 2;   % 0: linear SVM, 2: Gaussian kernel
    end
    
    methods
        function obj = classifier_nonlinearSVM()
            obj.name = 'nonlinSVM';
        end
        
        function Train(obj,y,x)
            n=length(y);
            trainExamples = min(obj.maxTrainExamples,n);
            cmd = ['-c ', num2str(2.^obj.log2c), ' -g ', num2str(2.^obj.log2g),...
                ' -t ' num2str(obj.type) ' -m 2000 -h 0 -b 1 -q'];
            obj.model = svmtrain(y(1:trainExamples),x(1:trainExamples,1:end),cmd);
        end
        
        
        function [yPred,accuracy, yProb] = Evaluate(obj,y,x)
             n=length(y);
             testExamples =  min(obj.maxTestExamples,n);
             [yPred, accuracy, yProb] = svmpredict(y(1:testExamples), x(1:testExamples,:), obj.model, '-b 1 -q');
             
             % Re-order classes based on model order
             yProb(:,obj.model.Label) = yProb;
             accuracy = accuracy(1);
        end
        
    end
    
end

