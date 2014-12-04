classdef ThirdLayer2DLabeler < handle
    %UNTITLED5 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
            config = [];  
            splitName = [];

            
            nClasses = [];


            fineTuneGravityVector = true;
            condorEnabled = false;

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
        
        RunThirdLayer(obj);
        WaitForCondor(obj);
        
        OrthoImagesBackProject(obj);
        ReassemblePointCloud(obj);
        

        %Calculates 3D positions of cameras
        function [camerapos,cameras] = GetCameraPos(obj)
            
            cameras = ImportCameras(get_addr('cameras',obj.config));
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
            gt = get_adr('pcl_gt_test',obj.config);
            fprintf('Evaluating %s...\n',labFilename);
            try
                scoreAfter = EvaluateMeshLabeling(labFilename,gt);
            catch
                fprintf('No labeling found.\n');
            end
        end
        

    end
    
end

