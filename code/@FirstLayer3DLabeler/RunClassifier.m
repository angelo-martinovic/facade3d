function RunClassifier(obj)
    dl = DispatchingLogger.getInstance();
    
    c3d = obj.config.c3D;
    c = c3d.classifier;
    classifierLoc = get_adr('classifier3d',obj.config,c.name);
    
    % Run classifier on test set
    dl.Log(VerbosityLevel.Info,sprintf(' - Running the 3D classifier on the test set...\n')); 
    clear model;
    
    tic;
    t = dir(classifierLoc);
    dl.Log(VerbosityLevel.Info,...
        sprintf(' - - Loading the classifier from %s, created on %s ...\n',classifierLoc,t.date));
    load(classifierLoc,'model');
    
    % Prepares the test matrix
    [~,~,Xtest] = obj.facade_unary_class('get_desc');
    
    dl.Log(VerbosityLevel.Info,sprintf(' - - Evaluating the classifier...\n'));
    [obj.prb,obj.cprb]  = obj.facade_unary_class('test','model',model,'Xtest',Xtest); 
    testTime = toc;    
    dl.Log(VerbosityLevel.Info,sprintf(' - - done. Elapsed time: %.2f seconds.\n',testTime));
    
    % Save timing
    save(get_adr('classifier3d_testTime',obj.config,c3d.name),'testTime');
    
    % Save resulting labeled point cloud
    path_labeling = get_adr('pcl_labeling',obj.config,c3d.name);
    dl.Log(VerbosityLevel.Info,sprintf(' - Saving the results...\n')); 
    obj.sceneTest.export_as_full_pcl_data( obj.config , obj.cprb , path_labeling ); %%% export labeling
    
    % Save resulting probability map per point
    pclProbDir = get_adr('pclProbDir',obj.config);
    mkdirIfNotExist(pclProbDir);
    
    path_prb = get_adr('pcl_unaries',obj.config,c3d.name);
    prb = obj.prb; %#ok<NASGU>
    save( path_prb , 'prb' );
    dl.Log(VerbosityLevel.Info,sprintf(' - Results saved to: %s and %s\n',path_labeling,path_prb)); 
    
    
end