setup; % Set paths
datasetConfig = InitializeDataset('monge428'); % Set up the dataset: file locations, file names

if datasetConfig.nWorkers>1
    InitializeParallel(datasetConfig.nWorkers);
end
    
rng(1); % For reproducibility

dl = DispatchingLogger.getInstance();
 
%% ======================================
% First+second layer 2D: Image labeling
% =======================================

    fl2D = FirstLayer2DLabeler ( datasetConfig );
    imageNames = fl2D.LoadFilenames('eval');
    
    fl2D.PrepareData();      % Resize, rectify, segment, extract features, run detectors
    fl2D.TrainClassifier();  % Train SVM on region features from the train set
    fl2D.RunClassifier();    % Use the trained SVM to classify superpixels in the test set
    fl2D.LabelImagesATLAS(); % Label images with: - classified superpixels (layer 1)
                             %                    - detectors and CRF (layer 2)

    EvaluateImageLabeling(datasetConfig,imageNames,fl2D.GetOutputFolderLayer1()); % Eval layer 1
    EvaluateImageLabeling(datasetConfig,imageNames,fl2D.GetOutputFolderLayer2()); % Eval layer 2
    
%% =====================================
% Projection of 2D classification to point cloud
% ======================================    
    fl2D.Project2DOntoPointCloud(); 
    EvaluateMeshLabeling(datasetConfig,fl2D.GetOutputProjectedLayer1()); % Eval layer 1 projected on PCL
    EvaluateMeshLabeling(datasetConfig,fl2D.GetOutputProjectedLayer2()); % Eval layer 2 projected on PCL
    
%% ======================================
% First layer 3D: Point cloud labeling
% =======================================
    fl3D = FirstLayer3DLabeler(datasetConfig); % Initialize data, calculate descriptors
    fl3D.PrepareData();          % Prepare descriptors
    pcl_test = fl3D.sceneTest;
    pcl_all = fl3D.sceneFull;
    
    fl3D.TrainClassifier();      % Train 3D point cloud classifier
    fl3D.RunClassifier();        % Run classifier on test set
    
    EvaluateMeshLabeling(datasetConfig,fl3D.GetPCLLabeling()); % Evaluate PCL labeling

%% =====================================
% Second layer 3D: 3D CRF
% ======================================

    sl3D = SecondLayer3DLabeler(datasetConfig,pcl_all);     % Setup unary potentials
    
    sl3D.Run3DCRF();                                % Run the CRF
    sl3D.SavePotentialsAndLabels();                 % Save the obtained labeling
    
   %modelName = sl3D.GetOutputNameMAP();
    modelName = sl3D.GetOutputNameCRF();            % Get the generated model name

%% =====================================
% Projection of 3D classification to images
% ======================================
    outputFolder = BatchGenerate2DLabelings(datasetConfig,imageNames,modelName); % Generate images
    EvaluateImageLabeling(datasetConfig,imageNames,outputFolder);  % Evaluate projection

%% =====================================
% Third layer
% ======================================
    %% ======================================
    % Preprocessing
    % =======================================
    tl2D = ThirdLayer2DLabeler (datasetConfig, modelName, pcl_test, pcl_all);
    tl2D.SplitPointCloud();   % Split point cloud into facade point clouds
    tl2D.FitPlanes();         % Fit a plane to each facade
    
    %% ======================================
    % Ortho 2D version
    % =======================================
    tl2D.OrthoImages();       % Create ortho images, labelings, unaries by projecting onto the plane

%     submittedToCluster = tl2D.RunThirdLayer(); % Run ortho2D third layer
%     if submittedToCluster
%         dl.Log(VerbosityLevel.Info,...
%             sprintf(['Jobs submitted to condor.' ...
%             'Run the remaining commands when condor jobs have finished.\n']));
%         return;
%     end
%     
%     tl2D.OrthoImagesBackProject();  % Back-project the labeling to facade 3D point clouds
%     tl2D.ReassemblePointCloud();    % Join the labeled facades into the original point cloud
    parsedPCLName = tl2D.GetOutputName();    
    EvaluateMeshLabeling(datasetConfig,parsedPCLName); 
    
    %% ======================================
    % 3D version
    % =======================================
    tl3D = ThirdLayer3DLabeler(datasetConfig,modelName,pcl_test,pcl_all);
    tl3D.RunThirdLayer();
    
    parsedPCLName = tl3D.GetOutputName();
    EvaluateMeshLabeling(datasetConfig,parsedPCLName); % Evaluate PCL labeling
    
%=======================================
%=======================================


