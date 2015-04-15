function facade_init_all_data(obj)
    dl = DispatchingLogger.getInstance();
    
    %% read data + descriptors
    %---- read basic data
    dl.Log(VerbosityLevel.Info,sprintf(' - read SScene_An.... dataset = ''%s'' \n',obj.config.name));

    obj.all_data = SScene_An();
    obj.all_data = obj.all_data.read_mat_data( obj.config );

    spinImageExtractorIdx = find(cellfun(@(x)strcmp(x.name,'spinImage'),obj.config.c3D.featureExtractors),1);
    if isempty(spinImageExtractorIdx)
        dl.Log(VerbosityLevel.Error,sprintf('Spin image extractor undefined! Check InitializeDataset.m and set datasetConfig.c3D to a 3D extractor'));
        error('Critical error. Terminating.');
    end
    spinImageExtractor = obj.config.c3D.featureExtractors{spinImageExtractorIdx};
    
     %--- separate into train/test
    obj.test_data = obj.all_data.keep_spec_data_ids(obj.all_data.flag==2);
    obj.train_data  = obj.all_data.keep_spec_data_ids(obj.all_data.flag==1);
    
    %--- read/calc  desc
    dl.Log(VerbosityLevel.Info,sprintf(' - Extracting descriptors from train set...\n'));
%     tic;
    obj.train_data = obj.train_data.process_data(...
        'compute_get_si',...
        'split','train',...
        'si_dimensions',spinImageExtractor.si_dimensions,...
        'binSize',spinImageExtractor.binSize,...
        'imgW',spinImageExtractor.imgW,...
        'datasetConfig',obj.config);
%     dl.Log(VerbosityLevel.Info,sprintf(' - -  done in %.2fsec\n',toc));
    
%     tic;
    dl.Log(VerbosityLevel.Info,sprintf(' - Extracting descriptors from test set...\n'));
    obj.test_data = obj.test_data.process_data(...
        'compute_get_si',...
        'split','test',...
        'si_dimensions',spinImageExtractor.si_dimensions,...
        'binSize',spinImageExtractor.binSize,...
        'imgW',spinImageExtractor.imgW,...
        'datasetConfig',obj.config);
%     dl.Log(VerbosityLevel.Info,sprintf(' - -  done in %.2fsec\n',toc));
    
    %--- fix labeling ids
    obj.train_data.lindex = obj.train_data.lindex-1; %%% applies to monge where zero should be background...
    obj.test_data.lindex = obj.test_data.lindex-1; %%% applies to monge where zero should be background...
%     obj.all_data.lindex = obj.all_data.lindex-1;
    
    dl.Log(VerbosityLevel.Info,sprintf(' - Concatenating descriptors...\n'));
    %--- concatenate descriptors to one vector
    [obj.train_data, minDESC, maxDESC, V, D] = obj.train_data.process_data('create_desc_from_weak_descs');

    %--- also desc
    obj.test_data = obj.test_data.process_data('create_desc_from_weak_descs','minX',minDESC,'maxX',maxDESC,'V',V,'D',D);
    
    % Remove unnecessary information
    obj.all_data.dist2plane=[];
    obj.all_data.nxyz=[];
    obj.all_data.heigh=[];
    obj.all_data.heigh_inv=[];
    obj.all_data.lindex=[];
    obj.all_data.oindex=[];
    obj.all_data.p_index=[];
    
    dl.Log(VerbosityLevel.Info,sprintf(' - init done.\n'));
end


