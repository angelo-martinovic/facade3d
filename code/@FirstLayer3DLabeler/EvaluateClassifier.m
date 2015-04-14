function EvaluateClassifier(obj)
    dl = DispatchingLogger.getInstance();
    
    c = obj.config.c3D.classifier;
    classifierLoc = get_adr('classifier3d',obj.config,c.name);
    
    % Run classifier on test set
    dl.Log(VerbosityLevel.Info,sprintf(' - Running the 3D classifier on the test set...\n')); 
    clear model;
    
    tic;
    load(classifierLoc,'model');
    [obj.prb,obj.cprb]  = obj.facade_unary_class('test','model',model,'Xtest',obj.Xtest); 
    rfTestTime = toc;    
    dl.Log(VerbosityLevel.Info,sprintf(' - - done. Elapsed time: %.2f seconds.\n',rfTestTime));
    
    % Save timing
    save(get_adr('classifier3d_testTime',obj.config,c.name),'rfTestTime');
    
    % Save resulting labeled point cloud
    path_labeling = get_adr('pcl_labeling',obj.config,c.name);
    dl.Log(VerbosityLevel.Info,sprintf(' - Saving the results...\n')); 
    fl.test_data.export_as_full_pcl_data( obj.config , obj.cprb , path_labeling ); %%% export labeling
    
    % Save resulting probability map per point
    pclProbDir = get_adr('pclProbDir',obj.config);
    mkdirIfNotExist(pclProbDir);
    
    path_prb = get_adr('pcl_unaries',obj.config,c.name);
    prb = obj.prb; %#ok<NASGU>
    save( path_prb , 'prb' );
    dl.Log(VerbosityLevel.Info,sprintf(' - Results saved to: %s and %s\n',path_labeling,path_prb)); 
    
    
end