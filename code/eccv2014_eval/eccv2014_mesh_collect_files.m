function data = eccv2014_mesh_collect_files (path, file_str_idx, numViews, numLabels, numFaces, type)
% data = eccv2014_mesh_collect_files (path, file_str_idx, numViews, numLabels, numFaces, type)
%
% Collect data from file on disk to data storage for multiview face storage.
% Runs through al images in the file_str_idx list and collects it data via the triangle index.
% 
% Sources: 
%       images          rgb mean color from images (.jpg)
%       train           label hist from GT training images (.png)
%       test            label hist from GT testing images (.png)
%       predictprob     probablities from predicted label probabilities (.mat)
%       predicttest     label hist from predicted label maps (.png)
%
% author: hayko riemenschneider, 2014

%% DATA ALLOCATION

data.area = zeros(numViews,1,numFaces,'single');
switch (type)
    
    case 'images'
        data.col = zeros(numViews,3,numFaces,'single');
    case 'train'
        data.train = zeros(numLabels,numFaces,'single');
    case 'test'
        data.test = zeros(numLabels,numFaces,'single');
    case 'predictprob'
        data.predictprob = zeros(numViews,numLabels,numFaces,'single');
    case 'predicttest'
        data.predicttest = zeros(numLabels,numFaces,'single');
        
end

%% DATA LOADING

for file_idx = 1: numViews
    
    progressbartime(file_idx, numViews, type)
    filename = [file_str_idx{1}{file_idx} '.jpg'];
    % display (['processing #' num2str(file_idx) '/' num2str(numViews) ': ' filename])
    
    im_idx = load([path.index filename(1:end-4) '.txt']);
    im_idx = im_idx+1; % get rid of index 0: NOTE: undone LATER
    
    % collect face area
    sumperindex = accumarray (im_idx(:), 1, [numFaces+1 1]);
    data.area(file_idx,:) = (sumperindex(2:end)); % LATER: GET RID OF +1
    
    switch(type)
        % ----------------------------------------------------------------------------------------------
        case 'images'
            % collect mean color from images
            im_rgb = imread([path.images filename(1:end-4) '.jpg']);
            data.col(file_idx,:,:) = mesh_collect_raw(im_rgb, im_idx, numFaces);
            
            % ----------------------------------------------------------------------------------------------
        case 'train'
            
            % collect histogram over training labelmaps
            try
                im_train_rgb = imread([path.train filename(1:end-4) '.png']);
                data.train = data_train + mesh_collect_labelmap(im_train_rgb, im_idx, cm, numFaces);
            catch
                % no training images available for this part of the mesh
            end
            
            % ----------------------------------------------------------------------------------------------
        case 'test'
            
            % collect histogram over testing labelmaps
            try
                im_test_rgb = imread([path.test filename(1:end-4) '.png']);
                data.test = data_test + mesh_collect_labelmap(im_test_rgb, im_idx, cm, numFaces);
            catch
                % no testing images available for this part of the mesh
            end
            
            
            
        case 'predictprob'
            % ----------------------------------------------------------------------------------------------
            % collect mean prob from images
            try
                 im_prob = load([path.predictprob  filename(1:end-4) '.mat']);
                if(isfield(im_prob,'im_res_probs'))
                    im_prob = single(im_prob.im_res_probs);
                end
                if(isfield(im_prob,'im_prob_test'))
                    im_prob = single(im_prob.im_prob_test(:,:,1:8));
                end
                %im_prob = save(['../data/probraw/' filename(1:end-4) '.mat'],'im_prob','-v7.3');
                
                data.predictprob(file_idx,:,:) = mesh_collect_raw(im_prob, im_idx, numFaces);
            catch
                % no prediction images available for this part of the mesh
            end
            
        case 'predicttest'
            % ----------------------------------------------------------------------------------------------
            % collect histogram over testing labelmaps
            try
                im_test_rgb = imread([path.predicttest filename(1:end-4) '.png']);
                data.predicttest = data_predicttest + mesh_collect_labelmap(im_test_rgb, im_idx, cm, numFaces);
            catch
                % no testing images available for this part of the mesh
            end
            
            
            
    end     % switch
    
end

