classdef ThirdLayer2DLabeler < handle
    %UNTITLED5 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
            dirName = '/esat/sadr/amartino/monge428New/data/';
            modelName = 'monge428New_lowres';
            
            gtTrainFilename = 'pcloud_gt_test_new_GCO.ply';
            gtTestFilename = 'pcloud_gt_test_new_GCO.ply';
            meshName = 'fullRes_mesh_colors_normals.ply';
            facadeSplit = 'fullRes_mesh_splitData.mat';
            
            evalListFilename = 'listeval_full.txt';
            trainListFilename = 'listtrain.txt';
            allListFilename = 'listall.txt';
            
            splitName = '3D';
            % TODO
%             classifier3D = '/esat/nihal/jknopp/3d_ret_recog_data/DT-SOL/monge482_fullres/res2angelo/3D_layer1-2000-200.mat';
%             classifierName = 'SVM';
            
            % low-res
%             gtFilename = 'fullRes_pcloud_test_new_GCO.ply'; % high-res
            
            nClasses = 7;
            cm = []; % Colormap

            fineTuneGravityVector = true;
            condorEnabled = false;
            
            allPoints = [];
            allColors = [];
    end
    
    methods
        function tl = ThirdLayer2DLabeler(dirName,modelName,gtTrainFilename,gtTestFilename,meshName,facadeSplit)
            tl.dirName = dirName;
            tl.modelName = modelName;
            tl.gtTrainFilename = gtTrainFilename;
            tl.gtTestFilename = gtTestFilename;
            tl.meshName = meshName;
            tl.facadeSplit = facadeSplit;
            
            tl.Configure();
        end
        
        
        Project2DOntoPointCloud(obj,labelingSource);
        filenames = LoadFilenames(obj,subset);
        
        LabelPointCloud(obj,weights);
        
        SplitPointCloud(obj);
        g=GetGravityVector(obj,facadeID,facadeNormal);
        FitPlanes(obj);
        OrthoImages(obj);
        
        RunThirdLayer(obj);
        WaitForCondor(obj);
        
        OrthoImagesBackProject(obj);
        ReassemblePointCloud(obj);
        
        function Configure(obj)
            obj.cm = HaussmannColormap();
        end
        
        % Lazy reading
        function [points,colors] = GetAllPoints(obj)
            if isempty(obj.allPoints)
                [obj.allPoints,~,obj.allColors] = ReadPCLFromPly([obj.dirName obj.meshName]);
            end

            points = obj.allPoints;
            colors = obj.allColors;
            
        end
        
        %Calculates 3D positions of cameras
        function [camerapos,cameras] = GetCameraPos(obj)
            
            cameras = ImportCameras([obj.dirName 'cameras.txt']);
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
            ss = load([obj.dirName 'work/pcl/split/' obj.splitName '/' obj.modelName '_facadeIDs.mat']);
            facadeIDs = ss.facadeIDs';

            facadeIDs = facadeIDs(facadeIDs~=0); % Skip background
        end
       
        function [scoreBefore,scoreAfter] = EvaluateLabeling(obj)
            % Before 3rd layer
            labFilename = [obj.dirName 'work/pcl/models/' obj.splitName '.ply'];
            gt = [obj.dirName obj.gtTestFilename];
            fprintf('Evaluating %s...\n',labFilename);
            try
                scoreBefore = EvaluateMeshLabeling(labFilename,gt);
            catch
                fprintf('No labeling found.\n');
            end
            
            % After 3rd layer
            labFilename = [obj.dirName 'work/pcl/models/' obj.splitName '_2Dlayer3.ply'];
            gt = [obj.dirName obj.gtTestFilename];
            fprintf('Evaluating %s...\n',labFilename);
            try
                scoreAfter = EvaluateMeshLabeling(labFilename,gt);
            catch
                fprintf('No labeling found.\n');
            end
        end
        

    end
    
end

