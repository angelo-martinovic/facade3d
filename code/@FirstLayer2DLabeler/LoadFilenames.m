function file_str_idx = LoadFilenames(obj,subset)
    dl = DispatchingLogger.getInstance();
    
    if strcmp(subset,'eval')
        filelist_str = obj.config.evalList;
    elseif strcmp(subset,'train')
        filelist_str = obj.config.trainList;
    elseif strcmp(subset,'all')
        filelist_str = obj.config.fullList;
    else
        dl.Log(VerbosityLevel.Error,sprintf('Unknown subset!'));
        error('Critical error. Terminating.');
    end

    % LOAD DATA FILE NAMES & INDEX
    fid = fopen([obj.config.dataLocation filelist_str]);
    file_str_idx = textscan(fid, '%s'); fclose(fid);
    numViews = length(file_str_idx{1});
    dl.Log(VerbosityLevel.Debug,sprintf(' - - Loaded %d filenames.\n', numViews));
    
    file_str_idx = file_str_idx{1};
end