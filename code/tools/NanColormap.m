function cmap = NanColormap()
    cmap = [ 
        0         0         0  ;...
    1.0000         0         0 ;...
    1.0000    1.0000         0 ;...
   ];
   cmap = round(255*cmap);
end