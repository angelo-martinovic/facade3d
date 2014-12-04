function file_str_idx = LoadFilenames(obj,subset)

    if strcmp(subset,'eval')
        filelist_str = obj.config.evalList;
    elseif strcmp(subset,'train')
        filelist_str = obj.config.trainList;
    elseif strcmp(subset,'all')
        filelist_str = obj.config.fullList;
    else
        error('Unknown subset!');
    end

    % LOAD DATA FILE NAMES & INDEX
    fid = fopen([obj.config.dataLocation filelist_str]);
    file_str_idx = textscan(fid, '%s'); fclose(fid);
    numViews = length(file_str_idx{1});
    fprintf('[Loaded %d filenames]', numViews);
    
    file_str_idx = file_str_idx{1};
end