function FirstLayer2D(datasetConfig)

    fl = FirstLayer2DLabeler ( datasetConfig );

    % Resize, rectify, segment, extract features, run detectors
    fl.PrepareData();

    if ~datasetConfig.useCache || ~exist(get_adr('2D_classifier',datasetConfig,datasetConfig.c2D.classifier.name),'file')
        % Train SVM on region features from the train set
        fl.TrainClassifier();
    end
    
    % Use the trained SVM to classify new images in the test set
    fl.RunClassifier();

    % Label images with SVM, detectors and CRF
    fl.LabelImagesATLAS();

    % Evaluates the accuracy of labeling
    fl.EvaluateLabeling();

    if ~datasetConfig.skipAll3D
        % Project labeling onto point cloud
        fl.Project2DOntoPointCloud();
    end
    
end

