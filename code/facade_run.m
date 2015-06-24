% Entry point

setup; % Set paths
datasetName = 'mongeToy'; % checked in DatasetConfig 

% Logging
sl = ScreenLogger(VerbosityLevel.Verbose);
fl = FileLogger(VerbosityLevel.Info,['log_' datasetName '.txt']);

dl = DispatchingLogger.getInstance();
dl.Clear();
dl.Subscribe(sl);
dl.Subscribe(fl);
        
% Set up the dataset: file locations, file names
datasetConfig = DatasetConfig.getInstance(datasetName);

if datasetConfig.nWorkers>1
    InitializeParallel(datasetConfig.nWorkers);
end
    
rng(1); % For reproducibility

 
%% ======================================
% First+second layer 2D: Image labeling
% =======================================
    FirstLayer2DLabeler.PrepareData();      % Resize, rectify, segment, extract features, run detectors
    FirstLayer2DLabeler.TrainClassifier();  % Train SVM on region features from the train set
    FirstLayer2DLabeler.RunClassifier();    % Use the trained SVM to classify superpixels in the test set
    FirstLayer2DLabeler.LabelImagesATLAS(); % Label images with: - classified superpixels (layer 1)
                                            %                    - detectors and CRF (layer 2)
    imageNames = LoadFilenames('eval');
    EvaluateImageLabeling(imageNames,FirstLayer2DLabeler.GetOutputFolderLayer1()); % Eval layer 1
    EvaluateImageLabeling(imageNames,FirstLayer2DLabeler.GetOutputFolderLayer2()); % Eval layer 2
    
%% =====================================
% Projection of 2D classification to point cloud
% ======================================    
    FirstLayer2DLabeler.Project2DOntoPointCloud(); 
    EvaluatePointCloudLabeling(FirstLayer2DLabeler.GetOutputProjectedLayer1()); % Eval layer 1 projected on PCL
    EvaluatePointCloudLabeling(FirstLayer2DLabeler.GetOutputProjectedLayer2()); % Eval layer 2 projected on PCL
    
%% ======================================
% First layer 3D: Point cloud labeling
% =======================================
    fl3D = FirstLayer3DLabeler(); % Initialize data, calculate some descriptors
    fl3D.PrepareData();          % Prepare descriptors
    pcl_test = fl3D.sceneTest;
    pcl_all = fl3D.sceneFull;
    
    fl3D.TrainClassifier();      % Train 3D point cloud classifier
    fl3D.RunClassifier();        % Run classifier on test set
    
    EvaluatePointCloudLabeling(FirstLayer3DLabeler.GetPCLLabelingFilename()); % Evaluate PCL labeling

%% =====================================
% Second layer 3D: 3D CRF
% ======================================

    sl3D = SecondLayer3DLabeler(pcl_all);     % Setup unary potentials
    
    sl3D.Run3DCRF();                                % Run the CRF
    sl3D.SavePotentialsAndLabels();                 % Save the obtained labeling
    
   %modelName = sl3D.GetOutputNameMAP();
    modelName = sl3D.GetOutputNameCRF();            % Get the generated model name

%% =====================================
% Projection of 3D classification to images
% ======================================
    outputFolder = BatchGenerate2DLabelings(imageNames,modelName); % Generate images
    EvaluateImageLabeling(imageNames,outputFolder);  % Evaluate projection

%% =====================================
% Third layer
% ======================================
    %% ======================================
    % Preprocessing
    % =======================================
    tl2D = ThirdLayer2DLabeler ( modelName, pcl_test, pcl_all);
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
    EvaluatePointCloudLabeling(parsedPCLName); 
    
    %% ======================================
    % 3D version
    % =======================================
    tl3D = ThirdLayer3DLabeler(modelName,pcl_test,pcl_all);
    tl3D.RunThirdLayer();
    
    parsedPCLName = tl3D.GetOutputName();
    EvaluatePointCloudLabeling(parsedPCLName); % Evaluate PCL labeling
    
%=======================================
%=======================================


