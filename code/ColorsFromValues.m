function cols_c=ColorsFromValues(cols,col_norm)

    if ~exist('col_norm','var'), %%% if col_norm does not exists
        col_norm(1) = min(min(cols))+eps;
        col_norm(2) = max(max(cols-col_norm(1)));
    end
    cmap = colormap;
    cols = cols-col_norm(1);
    cols = round(cols/col_norm(2)*(size(colormap,1)-1))+1;
    cols_c = round(cmap(cols,:)'*255);
end