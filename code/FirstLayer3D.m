function scene = FirstLayer3D(datasetConfig)
    rng(1); % For reproducibility
    
    fl = FirstLayer3DLabeler(datasetConfig);

    dl = DispatchingLogger.getInstance();
    dl.Log(VerbosityLevel.Info,'Running the 1st layer in 3D ...\n');
    
    %% Initialize data, calculate descriptors
    fl.facade_init_all_data();
    
    %% Prepare training and test data
    dl.Log(VerbosityLevel.Info,' - Creating correct descriptor format\n');
    tic;
    [Xtrain,Ytrain,Xtest] = fl.facade_unary_class('get_desc');
    elapsedTime = toc;
    dl.Log(VerbosityLevel.Info,sprintf(' - - done. Elapsed time: %.2f seconds.\n',elapsedTime));
   
    %% Learn the classifier
    c = datasetConfig.c3D.classifier;
    classifierLoc = get_adr('classifier3d',datasetConfig,c.unary_classifier);
    
    if datasetConfig.useCache && exist(classifierLoc,'file')
        t = dir(classifierLoc);
        dl.Log(VerbosityLevel.Info,...
            sprintf(' - Loading the classifier from %s, created on %s.\n',classifierLoc,t.date)); 
        tic;
        load(classifierLoc,'model');
        dl.Log(VerbosityLevel.Info,sprintf(' - - done. Elapsed time: %.2f seconds.\n',toc));
    else

        dl.Log(VerbosityLevel.Info,...
            sprintf(' - Learning the 3D classifier %s with %i trees, %d min_leaf...\n',...
            c.unary_classifier,c.nrf_tree,c.min_leaf));  
       
        tic;
        model       = fl.facade_unary_class('learn','X',Xtrain,'Y',Ytrain);
        dl.Log(VerbosityLevel.Info,sprintf(' - - done. Elapsed time: %.2f seconds.\n',toc));
        
        % Save the model
        dl.Log(VerbosityLevel.Info,sprintf(' - Saving the model to %s...\n',classifierLoc));
        tic;
        save(classifierLoc,'model','-v7.3');
        dl.Log(VerbosityLevel.Info,sprintf(' - - done. Elapsed time: %.2f seconds.\n',toc));
        
        if strcmp(c.oob_pred,'On')
            dl.Log(VerbosityLevel.Info,sprintf(' - Estimating the classifier error.\n')); 
            error = oobError(model);  %#ok<NASGU>
            save([classifierLoc(1:end-4) '_error.mat'],'error');
        end
    end
                
    %% Classify test set  
    dl.Log(VerbosityLevel.Info,sprintf(' - Evaluating the 3D classifier on the test set...\n'));  
    tic;
    [prb,cprb]  = fl.facade_unary_class('test','model',model,'Xtest',Xtest); %#ok<ASGLU>
    elapsedTime = toc;    
    dl.Log(VerbosityLevel.Info,sprintf(' - - done. Elapsed time: %.2f seconds.\n',elapsedTime));

    %% Plot results in matlab, only the sub-sampled set as matlab is slow...
    if datasetConfig.c3D.plotResults,
        fl.test_data.plot_scene('sample',30,'Q',cprb');
        fl.test_data.plot_scene('sample',30,'Q',fl.scene.lindex');
    end

    %% Save results
    if 1,
        dl.Log(VerbosityLevel.Info,sprintf(' - Saving the results...\n')); 
        path_labeling = get_adr('pcl_labeling',datasetConfig,'3D');
        fl.test_data.export_as_full_pcl_data( datasetConfig , cprb , path_labeling ); %%% export labeling
        pclProbDir = get_adr('pclProbDir',datasetConfig);
        mkdirIfNotExist(pclProbDir);
        path_prb = get_adr('pcl_unaries',datasetConfig,'3D');
        %dt = [scene.pts ; prb];
        save( path_prb , 'prb' );
        dl.Log(VerbosityLevel.Info,sprintf(' - Results saved to: %s and %s\n',path_labeling,path_prb)); 
    end
    
    scene = fl.test_data;
    
    dl.Log(VerbosityLevel.Info,sprintf(' - Evaluating the 3D labeling....\n')); 
    gt = get_adr('pcl_gt_test',datasetConfig);
    score = EvaluateMeshLabeling(path_labeling,gt); 
    save([path_labeling '_score.mat'],'score');
    disp(score);
    dl.Log(VerbosityLevel.Info,sprintf(' - Results saved in %s.\n',[path_labeling '_score.mat'])); 
end