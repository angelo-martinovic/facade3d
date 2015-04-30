%     - Uses the pretrained classifier with the given name to classify the images in the test set.
function RunClassifier(obj)  
    dl = DispatchingLogger.getInstance();
    
    classifierName = obj.config.c2D.classifier.name;
    % -- Load the SVM and the scaling data
    dl.Log(VerbosityLevel.Info,sprintf('Classifying with %s.\n',classifierName));
    
    predictionFilename = get_adr('cache_classifier_prediction',obj.config, obj.config.name, classifierName);
    
    if obj.config.useCache && exist(predictionFilename,'file')
        t = dir(predictionFilename);
        dl.Log(VerbosityLevel.Info,sprintf(' - Found predictions in %s, created at %s.\n',predictionFilename,t.date));
        return;
    end
    
    classifierLoc = get_adr('2D_classifier',obj.config,classifierName);
    t = dir(classifierLoc);
    dl.Log(VerbosityLevel.Info,sprintf(' - Loading the model from %s, created at %s.\n',classifierLoc,t.date));

    load(classifierLoc,'classifier','sc');
    
    obj.config.c2D.SetClassifier(classifier); 
    
    dl.Log(VerbosityLevel.Info,sprintf(' - Loading the testing data...\n'));
    cacheFilename = get_adr('cache_classifier_test',obj.config, obj.config.name, classifierName); 
    if obj.config.useCache && exist(cacheFilename,'file')
        % Load cached data
        dl.Log(VerbosityLevel.Info,sprintf(' - Found cache! Loading the cache...\n'));
        load(cacheFilename);
    else
        % Reload data
        tic;
        [t3,x3,segsPerImage,imageNames] = obj.LoadData('eval');
        dl.Log(VerbosityLevel.Info,sprintf(' - Evaluation data loading: %.2f seconds.\n',toc));
        if obj.config.useCache
          cacheFolder = get_adr('cache',obj.config);
          mkdirIfNotExist(cacheFolder);
          save(cacheFilename,'t3','x3','segsPerImage','imageNames');
        end
    end
   
    % --- Scaling test data with the parameters from training&validation
    dl.Log(VerbosityLevel.Info,sprintf(' - Scaling data...\n'));
    x_test = sc.Scale(x3);
    y_test = vec2ind(t3')';

    % SVM output
    dl.Log(VerbosityLevel.Info,sprintf(' - Predicting output...\n'));

    tic;
    [~,~,yProb] = classifier.Evaluate(y_test,x_test);
    dl.Log(VerbosityLevel.Info,sprintf(' - Evaluation of %s done. Elapsed time: %.2f seconds.\n',classifierName,toc));
    if obj.config.useCache
        save(predictionFilename,'yProb');
    end
    
    tic;    
    % Transpose the probability matrix to have rows corresponding to classes
    % Add a row vector of zeros at the top - class '0'
    yProb = [zeros(1,size(yProb',2)); yProb'; zeros(1,size(yProb',2))];
    
    dl.Log(VerbosityLevel.Info,sprintf(' - Saving predictions...\n'));
    % Output predictions for each image separately
    count=1;
    for i=1:length(segsPerImage)
        yProbSubset = yProb(1:obj.config.nClasses+1,count:count+segsPerImage{i}-1);
        count = count+segsPerImage{i};
        
        % Create the directory if it doesnt exist
        classificationDir = get_adr('2D_classification',obj.config,classifierName);
        mkdirIfNotExist(classificationDir);
        
        % Write out the prediction file
        dlmwrite(get_adr('2D_marginals',obj.config,classifierName,imageNames{i}),yProbSubset',' ');   
    end
    dl.Log(VerbosityLevel.Info,sprintf(' - Marginal saving of %s done. Elapsed time: %.2f seconds.\n',classifierName,toc));
end