%     - Uses the pretrained classifier with the given name to classify the images in the test set.
function ClassifyWithSVM(obj)

    % -- Load the SVM
    disp('Loading the model.');
    classifierFolder = [obj.dirName 'work/classifier/'];
    load([classifierFolder obj.classifierName '.mat'],'model','sumAll','rangeAll');
    
    % Caching
    cacheFolder = [obj.dirName 'work/cache/'];
    cacheFilename = [cacheFolder 'temp_' obj.baseName '.mat'];
    
    if ~exist(cacheFilename,'file')
        % -------------------------------Loading test data -------------------
        disp('Loading testing data...');
        [t3,x3,segsPerImage,imageNames] = LoadData(obj,'eval');
    else
        load(cacheFilename);
    end
    
    % Testing data
    t = t3;
    x = x3;
    
    % --- Scaling test data with the parameters from training&validation
    disp('Scaling data...');

    if size(x,1)>obj.nFeats
        x=x(1:obj.nFeats,:);
    end
    x = 2 * x';
    x = bsxfun(@minus, x, sumAll);
    x = bsxfun(@rdivide, x, rangeAll);
    
    % Target classes
    t = (vec2ind(t))';
    
    % SVM output
    disp('Predicting output...');
    cacheFilename = [cacheFolder 'temp_' obj.baseName '_prediction_' obj.classifierName '.mat'];
    if ~exist(cacheFilename,'file')
        tic;
        [~, ~, yProb] = svmpredict(t,x,model,'-b 1');
        svmPredictTime = toc;
%         disp(svmPredictTime);
        save(cacheFilename,'yProb');
    else
        disp('Found prediction cache...');
        load(cacheFilename);
    end
    
    tic;
    % -- Transpose the probability matrix to have rows corresponding to
    % classes, and reorder the rows based on the model
    yProb = yProb';
    
    yProb2 = zeros(obj.nClasses,size(yProb,2)); 
    yProb2(model.Label,:) = yProb;
%     yProb = yProb(order,:);
    
    % Add a row vector of zeros at the top - class '0'
    yProb = [zeros(1,size(yProb2,2)); yProb2; zeros(1,size(yProb2,2))];
    
    % Output predictions for each image separately
    count=1;
    for i=1:length(segsPerImage)
        yProbSubset = yProb(1:obj.nClasses+1,count:count+segsPerImage{i}-1);
        count = count+segsPerImage{i};
        
        % Create the directory if it doesnt exist
        classificationDir = [obj.dirName '/work/classifier/' obj.classifierName '/'];
        if ~exist(classificationDir,'dir')
            mkdir(classificationDir);
        end
        
        % Write out the prediction file
        dlmwrite([classificationDir imageNames{i} '.marginal.txt'],yProbSubset',' ');
        
    end
    marginalPrepTime = toc;
%     disp(marginalPrepTime);
    
end