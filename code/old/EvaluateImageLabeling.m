function score = EvaluateImageLabeling(dirName,outputFolder)
    %% SETUP PATHS
    cm = HaussmannColormap()/255;

%     filelist_all = 'listall.txt';
%     filelist_train = 'listtrain_new.txt';
    filelist_eval = 'listeval_new_full.txt';
    filelist_str = filelist_eval;

    % LOAD DATA FILE NAMES & INDEX
    fid = fopen([dirName filelist_str]);
    file_str_idx = textscan(fid, '%s'); fclose(fid);
    numViews = length(file_str_idx{1});
    display(['found ' num2str(numViews) ' files.'])

    % path.images = [dirName 'images/'];
    % path.train = [dirName 'labels/'];
    path.test = [dirName 'labels/'];

    path.predictmap = outputFolder; % can be generated

    %% EVALUATION TASK 1 - Image Labelling - vanilla 2d img labelling task (THIS CODE)

    fprintf('Loading data...\n');
    [gt, res] = evaluation_load_folder (path.test, path.predictmap, file_str_idx, numViews, cm);
    fprintf('Done!\n');
    fprintf('Evaluating...\n');
    score = evaluation_multilabel(gt,res,[1 9]);


end