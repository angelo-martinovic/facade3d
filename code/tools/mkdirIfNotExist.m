function mkdirIfNotExist(dirName)
    if ~exist(dirName,'dir')
        mkdir(dirName);
        fileattrib(dirName,'+w','g');
    end
end