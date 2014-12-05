function ThirdLayer2D(datasetConfig, inputName)
    % Setup
    tl = ThirdLayer2DLabeler ( datasetConfig, inputName);

    % Split point cloud into facade point clouds
    tl.SplitPointCloud();

    % Fit a plane to each facade
    tl.FitPlanes();
    
    % Create ortho images, labelings, unaries by projecting onto the plane
    tl.OrthoImages();

    % Run ortho2D third layer
    tl.RunThirdLayer();
    
    % Back-project the labeling to facade 3D point clouds
    tl.OrthoImagesBackProject();
    
    % Join the labeled facade point clouds into the original point cloud
    tl.ReassemblePointCloud();
    
    % Evaluate
    tl.EvaluateLabeling();
    
    
end

