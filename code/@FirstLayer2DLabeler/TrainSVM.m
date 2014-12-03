%     - Uses the output of PrepareData.m to train a superpixel classifier on the training set.
function TrainSVM(obj)

    cacheFolder = [obj.dirName 'work/cache/'];
    cacheFilename = [cacheFolder 'temp_' obj.baseName '.mat'];
    
    % Caching
    if ~exist(cacheFolder,'dir')
        mkdir(cacheFolder);
    end
    
    if ~exist(cacheFilename,'file')
        
        % -------------------------------Loading training data ---------------
        tic;
        disp('Loading training data...');
        [t1,x1] = obj.LoadData('train');
        trainingDataLoadTime = toc;
        fprintf('Training data loading: %d seconds.',trainingDataLoadTime);
        
        % -------------------------------Loading test data -------------------
        tic;
        disp('Loading testing data...');
        [t3,x3,segsPerImage,imageNames] = obj.LoadData('eval');
        evalDataLoadTime = toc;
        fprintf('Evaluation data loading: %d seconds.',evalDataLoadTime);
        save(cacheFilename,'t1','x1','t3','x3','segsPerImage','imageNames');
    else
        load(cacheFilename);
    end
    
    % --------- Data scaling -------
    disp('Scaling data...');
    maxAll = max([x1],[],2)';
    minAll = min([x1],[],2)';
    rangeAll = maxAll-minAll;
    sumAll = maxAll+minAll;

    % Scale to [-1,1]
    x = [x1 x3]';
    
    x = 2 * x;
    x = bsxfun(@minus, x, sumAll);
    x = bsxfun(@rdivide, x, rangeAll);
    
    % --------- Combining ---
    t = [t1 t3];
    n1=length(t1);
    n3=length(t3);

    % Target classes
    t = (vec2ind(t))';
   
    disp('Starting the training...');
    tic;
    trainExamples = min(obj.svm.maxTrainExamples,n1);
    cmd = ['-c ', num2str(2.^obj.svm.log2c), ' -g ', num2str(2.^obj.svm.log2g),' -t 2 -m 2000 -h 0 -b 1 '];
    model = svmtrain(t(1:trainExamples),x(1:trainExamples,1:end),cmd);
    
    svmTrainTime = toc;
    fprintf('Training time elapsed: %d.\n',svmTrainTime);
   
    % Prediction on the test set
    disp('Predicting performance on (a subset of) test...');
    testExamples =  min(obj.svm.maxTestExamples,n3);
    [~, accuracy, ~] = svmpredict(t(n1+1:n1+testExamples), x(n1+1:n1+testExamples,1:end), model);
    disp(['Accuracy on the test set: ' num2str(accuracy(1))]);
    
    disp('Saving the SVM model.');
    classifierFolder = [obj.dirName 'work/classifier/'];
    if ~exist(classifierFolder,'dir')
        mkdir(classifierFolder);
    end
    save([classifierFolder obj.classifierName '.mat'],'model','sumAll','rangeAll');   
end
