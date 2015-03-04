%     - Uses the output of PrepareData.m to train a superpixel classifier on the training set.
function TrainClassifier(obj)
    cf = obj.config;
    dl = DispatchingLogger.getInstance();
    classifier = cf.c2D.classifier; 
    
    dl.Log(VerbosityLevel.Info,...
        sprintf('Training the %s superpixel classifier...\n',classifier.name));
    
    dl.Log(VerbosityLevel.Info,sprintf(' - Loading training data...\n'));
    cacheFilename = get_adr('cache_classifier_train',cf, cf.name, cf.c2D.classifier.name); 
    
    if cf.useCache && exist(cacheFilename,'file')
        % Load cached data
        t= dir(cacheFilename);
        dl.Log(VerbosityLevel.Info,...
            sprintf(' - - Loading cache from %s, created on %s ...\n',cacheFilename,t.date));
        load(cacheFilename);
    else
        % Loading training data
        tic;
        [t1,x1] = obj.LoadData('train');
        dl.Log(VerbosityLevel.Info,sprintf(' - Training data loading: %.2f seconds.\n',toc));

        if obj.config.useCache
          cacheFolder = get_adr('cache',cf);
          mkdirIfNotExist(cacheFolder);
          save(cacheFilename,'t1','x1');%,'t3','x3');
        end
    end
    
    % Data scaling
    dl.Log(VerbosityLevel.Info,sprintf(' - Scaling data...\n'));
    sc = scaling(x1);
    x_train = sc.Scale(x1);
    y_train = vec2ind(t1')';

    % Training
    dl.Log(VerbosityLevel.Info,sprintf(' - Starting training...\n'));
    tic;
    classifier.Train(y_train,x_train);
    dl.Log(VerbosityLevel.Info,sprintf(' - Training time of %s elapsed: %.2f.\n',classifier.name,toc));
    
    dl.Log(VerbosityLevel.Info,sprintf(' - Saving the model.\n'));
    classifierFolder = get_adr('2D_classifier_folder',cf);
    mkdirIfNotExist(classifierFolder);

    % Save the classifier and scaling data from the training set
    save(get_adr('2D_classifier',cf,cf.c2D.classifier.name),'classifier','sc');   
end
