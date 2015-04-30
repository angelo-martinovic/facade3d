% Train the point cloud classifier
function TrainClassifier(obj)
    dl = DispatchingLogger.getInstance();
     
    c = obj.config.c3D.classifier;
    classifierLoc = get_adr('classifier3d',obj.config,c.name);
    
    % Check if cache exists
    if obj.config.useCache && exist(classifierLoc,'file')
        t = dir(classifierLoc);
        dl.Log(VerbosityLevel.Info,...
            sprintf(' - Loading the classifier from %s, created on %s.\n',classifierLoc,t.date)); 
        tic;
        load(classifierLoc,'model');
        dl.Log(VerbosityLevel.Info,sprintf(' - - done. Elapsed time: %.2f seconds.\n',toc));
    else

        dl.Log(VerbosityLevel.Info,...
            sprintf(' - Learning the 3D classifier %s with %i trees, %d min_leaf...\n',...
            c.name,c.nrf_tree,c.min_leaf));  
       
        tic;
        [model,error]     = obj.facade_unary_class('learn','X',obj.Xtrain,'Y',obj.Ytrain); %#ok<ASGLU>
        trainTime = toc;
        dl.Log(VerbosityLevel.Info,sprintf(' - - done. Elapsed time: %.2f seconds.\n',trainTime));
        
        % Save the model
        dl.Log(VerbosityLevel.Info,sprintf(' - Saving the model to %s...\n',classifierLoc));
        tic;
        save(classifierLoc,'model','-v7.3');
        dl.Log(VerbosityLevel.Info,sprintf(' - - done. Elapsed time: %.2f seconds.\n',toc));
        
        save(get_adr('classifier3d_error',obj.config,c.name),'error');
        save(get_adr('classifier3d_trainTime',obj.config,c.name),'trainTime'); 

    end
    
end