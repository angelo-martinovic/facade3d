function facade_init_all_data(obj)
    dl = DispatchingLogger.getInstance();
    
    %% read data + descriptors
    %---- read basic data
    dl.Log(VerbosityLevel.Info,sprintf(' - read SScene_An.... dataset = ''%s'' \n',obj.config.name));

    fullScene = SScene_An();%path_mat_data_dir,postfix_path_data_orig);
    fullScene = fullScene.read_mat_data( obj.config );

    spinImageExtractorIdx = find(cellfun(@(x)strcmp(x.name,'spinImage'),obj.config.c3D.featureExtractors),1);
    if isempty(spinImageExtractorIdx)
        dl.Log(VerbosityLevel.Error,sprintf('Spin image extractor undefined! Check InitializeDataset.m and set datasetConfig.c3D to a 3D extractor'));
        error('Critical error. Terminating.');
    end
    spinImageExtractor = obj.config.c3D.featureExtractors{spinImageExtractorIdx};
    
     %--- separate into train/test
    test_data = fullScene.keep_spec_data_ids(fullScene.flag==2);
    train_data  = fullScene.keep_spec_data_ids(fullScene.flag==1);
    
    %--- read/calc  desc
    dl.Log(VerbosityLevel.Info,sprintf(' - Extracting descriptors from train set...\n'));
%     tic;
    train_data = train_data.process_data(...
        'compute_get_si',...
        'split','train',...
        'si_dimensions',spinImageExtractor.si_dimensions,...
        'binSize',spinImageExtractor.binSize,...
        'imgW',spinImageExtractor.imgW,...
        'datasetConfig',obj.config);
%     dl.Log(VerbosityLevel.Info,sprintf(' - -  done in %.2fsec\n',toc));
    
%     tic;
    dl.Log(VerbosityLevel.Info,sprintf(' - Extracting descriptors from test set...\n'));
    test_data = test_data.process_data(...
        'compute_get_si',...
        'split','test',...
        'si_dimensions',spinImageExtractor.si_dimensions,...
        'binSize',spinImageExtractor.binSize,...
        'imgW',spinImageExtractor.imgW,...
        'datasetConfig',obj.config);
%     dl.Log(VerbosityLevel.Info,sprintf(' - -  done in %.2fsec\n',toc));
    
    %--- fix labeling ids
    train_data.lindex = train_data.lindex-1; %%% applies to monge where zero should be background...
    test_data.lindex = test_data.lindex-1; %%% applies to monge where zero should be background crap...

    dl.Log(VerbosityLevel.Info,sprintf(' - Concatenating descriptors...\n'));
    %--- concatenate descriptors to one vector
    [obj.train_data, minDESC, maxDESC, V, D] = train_data.process_data('create_desc_from_weak_descs');

    %--- also desc
    obj.test_data = test_data.process_data('create_desc_from_weak_descs','minX',minDESC,'maxX',maxDESC,'V',V,'D',D);
    
    dl.Log(VerbosityLevel.Info,sprintf(' - init done.\n'));
end


