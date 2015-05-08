function file_str_idx = LoadFilenames(subset)
    dl = DispatchingLogger.getInstance();
    cf = DatasetConfig.getInstance();
    
    if strcmp(subset,'eval')
        filelist_str = cf.evalList;
    elseif strcmp(subset,'train')
        filelist_str = cf.trainList;
    elseif strcmp(subset,'all')
        filelist_str = cf.fullList;
    else
        dl.Log(VerbosityLevel.Error,sprintf('Unknown subset!'));
        fatal();
    end

    % LOAD DATA FILE NAMES & INDEX
    filename = [cf.dataLocation filelist_str];
    fid = fopen(filename);
    if fid==-1
        dl.Log(VerbosityLevel.Error,sprintf('File %s could not be opened!\n',filename));
        fatal();
    end
    file_str_idx = textscan(fid, '%s'); fclose(fid);
    numViews = length(file_str_idx{1});
    dl.Log(VerbosityLevel.Debug,sprintf(' - - Loaded %d filenames.\n', numViews));
    
    file_str_idx = file_str_idx{1};
end