


function tmp_labeling = facade_erode(tmp_labeling,i_nn,n)


if ~exist('n','var') || isempty(n)
    n = 1;
end

for a=1:n
    
    mode_labeling = mode(tmp_labeling(i_nn(:,2:end))'); %%% find neighs labeling
    
    idx_diff_labeling = tmp_labeling~=mode_labeling;
    tmp_labeling(idx_diff_labeling) = mode_labeling(idx_diff_labeling);
    
end
    
end