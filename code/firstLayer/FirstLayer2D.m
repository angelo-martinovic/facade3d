function FirstLayer2D(datasetConfig)

    fl = FirstLayer2DLabeler ( datasetConfig );

    % Resize, rectify, segment, extract features, run detectors
    fl.PrepareData();

    % Train SVM on region features from the train set
    fl.TrainSVM();

    % Use the trained SVM to classify new images in the test set
    fl.ClassifyWithSVM();

    % Label images with SVM, detectors and CRF
    fl.LabelImagesATLAS();

    % Evaluates the accuracy of labeling
    fl.EvaluateLabeling();

    % Project labeling onto point cloud
    fl.Project2DOntoPointCloud();
    
end
