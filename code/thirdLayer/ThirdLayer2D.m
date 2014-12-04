function ThirdLayer2D(datasetConfig, inputName)
    % Setup
    tl = ThirdLayer2DLabeler ( datasetConfig, inputName);

    % Split point cloud into facades
    tl.SplitPointCloud();

    %
    tl.FitPlanes();
    
    % Create ortho images, labelings, unaries
    tl.OrthoImages();

    % Run ortho2D third layer
    tl.RunThirdLayer();
    
end

