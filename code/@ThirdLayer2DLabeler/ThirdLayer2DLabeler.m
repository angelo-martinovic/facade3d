classdef ThirdLayer2DLabeler < handle
    %UNTITLED5 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
            config = [];  
            splitName = [];
    
            pcl_test = [];
            pcl_all = [];
        
            orthoParams = [];
            
           
            
            condorEnabled = true;
    end
    
    methods
        function tl = ThirdLayer2DLabeler(datasetConfig,splitName,pcl_test,pcl_all)
            tl.config = datasetConfig;
            tl.splitName = splitName;
            tl.pcl_test = pcl_test;
            tl.pcl_all = pcl_all;
            
            % Ortho images creation
            % Visualization is useful for debugging but makes the code
            % quite slow.
            tl.orthoParams.visualize = false;
            
            % Initial gravity vector might not be precise enough. We can
            % fine tune it by tilting the ortho camera and checking whether
            % edges in the ortho images align with the vertical direction.
            tl.orthoParams.fineTuneGravityVector = true; 
            tl.orthoParams.xVecRange = 0;%-0.05:0.02:0.05;
            tl.orthoParams.zVecRange = -0.05:0.02:0.05;
            
            % An ortho image is created by projecting images from 
            % nClosestCameras onto the facade plane.
            tl.orthoParams.nClosestCameras = 10; 
            
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
       
        function outputPCLName = GetOutputName(obj)
             outputPCLName = get_adr('3D_L3_Ortho2D_labeling',obj.config,obj.splitName);
        end
    end
    
end

