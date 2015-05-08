function [gt, res] = evaluation_load_folder (path_gt, path_result, file_str_idx, numViews, cm)
% [gt, res] = evaluation_load_folder (path_gt, path_result, file_str_idx, numViews, cm)
%
% Collect data for evaluation from .PNG files for GT and results.
% Runs through al images in the file_str_idx list and collect data.
%
% author: hayko riemenschneider, 2014

%% DATA ALLOCATION

gt_stack=cell(numViews,1); res_stack=cell(numViews,1);

for file_idx = 1: numViews
    
    filename = [file_str_idx{1}{file_idx} '.png'];
    progressbartime(file_idx, numViews)
%     display (['loading #' num2str(file_idx) '/' num2str(length(file_list)) ': ' filename])
    
    
    % LOAD DATA
    try
        im_gt = imread([path_gt '/' filename]);
    catch
        % skip of GT file not available (119 images with GT vs 202 all test images)
        continue
    end
    
    try
        im_res = imread([path_result '/' filename]);
    catch
        display(['evaluation_load_folder: missing result: ' filename])
        continue
    end
    
    % CONVERT TO INT LABELS (matlab 1 index)
    gt_stack{file_idx} = rgb2labelmap(im_gt, cm);
    res_stack{file_idx} = rgb2labelmap(im_res, cm);

end


gt = cat(3,gt_stack{:});
gt = gt(:);
clear gt_stack;
res = cat(3,res_stack{:});
res = res(:);
clear res_stack;

