
%
%
%   i_nn = knnsearch(pts',pts','k',5);
%

function tmp_labeling = facade_3d_morph_operations(method,tmp_labeling,i_nn,n,only_heigh)


if ~exist('only_heigh','var'),
    only_heigh = [];
end

if ~exist('n','var') || isempty(n)
    n = 1;
end


switch method
    case 'erode'
        for a=1:n
            idx_keep_label = sum(bsxfun(@eq , tmp_labeling(i_nn(:,2:end)) , tmp_labeling'),2) == (size(i_nn,2)-1); %%% find whcih points have the same label as ALL neighbors  
            tmp_labeling(~idx_keep_label & tmp_labeling'==1) = 0;%~tmp_labeling(~idx_keep_label && tmp_labeling==1);  %%% points that dotn have same label -> change! 
%             mode_labeling = mode(tmp_labeling(i_nn(:,2:end))'); %%% find neighs labeling
%             idx_diff_labeling = tmp_labeling~=mode_labeling;
%             tmp_labeling(idx_diff_labeling) = mode_labeling(idx_diff_labeling);            
        end
    case 'dilation'
        tmp_labeling = facade_3d_morph_operations('erode',~tmp_labeling,i_nn,n);
        tmp_labeling = ~tmp_labeling;
    case 'closing'
        tmp_labeling = facade_3d_morph_operations('erode',tmp_labeling,i_nn,n);
        tmp_labeling = facade_3d_morph_operations('dilation',tmp_labeling,i_nn,n);
end