classdef ThirdLayer2DLabeler < handle
    %UNTITLED5 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
            config = [];  
            splitName = [];

            nClasses = [];

            fineTuneGravityVector = true;
            condorEnabled = true;

    end
    
    methods
        function tl = ThirdLayer2DLabeler(datasetConfig,splitName)
            tl.config = datasetConfig;
            tl.splitName = splitName;
            tl.nClasses = datasetConfig.nClasses;
        end

               
        SplitPointCloud(obj);
        g=GetGravityVector(obj,facadeID,facadeNormal);
        FitPlanes(obj);
        OrthoImages(obj);
        
        submitted=RunThirdLayer(obj);
        WaitForCondor(obj);
        
        OrthoImagesBackProject(obj);
        ReassemblePointCloud(obj);
        

        %Calculates 3D positions of cameras
        function [camerapos,cameras] = GetCameraPos(obj)
            
            cameras = ImportCameras(get_adr('cameras',obj.config));
            camerapos = zeros(length(cameras),3);

            for i=1:length(cameras)
                bla=cameras{i}.P;
                Phat=bla(:,1:3); 
                Fhat=bla(:,4); 
                camerapos(i,:)=Phat\Fhat;
            end

            camerapos = -camerapos;
        end
        
        function facadeIDs = GetFacadeIDs(obj)
            % Get facade separation info
            ss = load(get_adr('facadeIDs',obj.config,obj.splitName));
            facadeIDs = ss.facadeIDs';

            facadeIDs = facadeIDs(facadeIDs~=0); % Skip background
        end
       
        function score = EvaluateLabeling(obj)

            % After 3rd layer
            labFilename = get_adr('3D_L3_Ortho2D_labeling',obj.config,obj.splitName);
            gtFilename = get_adr('pcl_gt_test',obj.config);
            fprintf('Evaluating %s...\n',labFilename);
           
            % Get labeling
            [points_full,~,colors_full]=ReadPCLFromPly(labFilename);
            labels_full = Colors2Labels(colors_full,obj.config.cm);

            % Get GT
            [pointsGT_full,~,colorsGT_full]=ReadPCLFromPly(gtFilename);
            labelsGT_full = Colors2Labels(colorsGT_full,obj.config.cm);

            assert(isequal(size(points_full),size(pointsGT_full)));
            assert(isequal(size(colorsGT_full),size(colors_full)));

            ignoreClasses = obj.config.ignoreClasses+1;

            score = evaluation_multilabel(labelsGT_full,labels_full,ignoreClasses);
            disp(score);
        end
        

    end
    
end

