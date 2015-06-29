classdef FirstLayer3DLabeler < handle
    %FirstLayer3DLabeler Base class for semantic classification of 3D
    %point clouds.
    %   Detailed explanation goes here
    
    properties
        sceneFull = [];  % Whole point cloud
        sceneTrain = []; % Training subset
        sceneTest = [];  % Testing subset

        prb = [];   % Resulting class probability per point
        cprb = [];  % Resulting labeling per point
        
    end
    %%
    methods (Access = public)
        % Constructor
        function fl = FirstLayer3DLabeler()
            facade_init_scene(fl);
        end
        
        function PrepareData(obj)
            dl = DispatchingLogger.getInstance();
    
            dl.Log(VerbosityLevel.Info,' - Extracting PCL descriptors...\n');
            tic;
            obj.calc_features();
            elapsedTime = toc;
            dl.Log(VerbosityLevel.Info,sprintf(' - - done. Elapsed time: %.2f seconds.\n',elapsedTime));
        end
        
        TrainClassifier(obj);
        
        RunClassifier(obj);
         
        PlotResults(obj);
         
    end

    %%
    methods (Static)
        function pclLabeling = GetPCLLabelingFilename()
            cf = DatasetConfig.getInstance();
            c3d = cf.c3D;
 
            pclLabeling = get_adr('pcl_labeling','3D');
        end 
    end
    
    %%
    methods(Access = private)
       
       [out1,out2,out3,out4] = facade_unary_class (obj,method,varargin);
       
       
       function facade_init_scene(obj)
            dl = DispatchingLogger.getInstance();
            cf = DatasetConfig.getInstance();
            % read data + descriptors
            %---- read basic data
            dl.Log(VerbosityLevel.Info,sprintf(' - read SScene_An.... dataset = ''%s'' \n',cf.name));

            obj.sceneFull = SScene_An();
            obj.sceneFull = obj.sceneFull.read_mat_data( );
       end


       function calc_features(obj)
            cf = DatasetConfig.getInstance();
            dl = DispatchingLogger.getInstance();
            %---- calculate simple features from the entire point cloud
            obj.sceneFull = obj.sceneFull.calc_simple_features( );
            
            %--- separate into train/test
            obj.sceneTest = obj.sceneFull.copy();
            obj.sceneTest.keep_spec_data_ids(obj.sceneFull.flag==2);
            
            obj.sceneTrain = obj.sceneFull.copy();
            obj.sceneTrain.keep_spec_data_ids(obj.sceneFull.flag==1);
            
            % Remove unnecessary information
            obj.sceneFull.dist2plane=[];
            obj.sceneFull.nxyz=[];
            obj.sceneFull.heigh=[];
            obj.sceneFull.heigh_inv=[];
            obj.sceneFull.lindex=[];
            obj.sceneFull.oindex=[];
            obj.sceneFull.p_index=[];
            
            %--- calculate spin images, separate for train, test
            spinImageExtractorIdx = find(cellfun(@(x)strcmp(x.name,'spinImage'),cf.c3D.featureExtractors),1);
            if isempty(spinImageExtractorIdx)
                dl.Log(VerbosityLevel.Error,sprintf('Spin image extractor undefined! Check InitializeDataset.m and set datasetConfig.c3D to a 3D extractor'));
                error('Critical error. Terminating.');
            end
            spinImageExtractor = cf.c3D.featureExtractors{spinImageExtractorIdx};

            %--- read/calc desc
            dl.Log(VerbosityLevel.Info,sprintf(' - Extracting spin images from train set...\n'));
        %     tic;
            obj.sceneTrain.process_data(...
                'compute_get_si',...
                'split','train',...
                'si_dimensions',spinImageExtractor.si_dimensions,...
                'binSize',spinImageExtractor.binSize,...
                'imgW',spinImageExtractor.imgW);
        %     dl.Log(VerbosityLevel.Info,sprintf(' - -  done in %.2fsec\n',toc));

        %     tic;
            dl.Log(VerbosityLevel.Info,sprintf(' - Extracting spin images from test set...\n'));
            obj.sceneTest.process_data(...
                'compute_get_si',...
                'split','test',...
                'si_dimensions',spinImageExtractor.si_dimensions,...
                'binSize',spinImageExtractor.binSize,...
                'imgW',spinImageExtractor.imgW);
        %     dl.Log(VerbosityLevel.Info,sprintf(' - -  done in %.2fsec\n',toc));

            %--- fix labeling ids
            obj.sceneTrain.lindex = obj.sceneTrain.lindex-1; %%% applies to monge where zero should be background...
            obj.sceneTest.lindex = obj.sceneTest.lindex-1; %%% applies to monge where zero should be background...

            %--- concatenate descriptors to one vector
            dl.Log(VerbosityLevel.Info,sprintf(' - Concatenating descriptors...\n'));
            obj.sceneTrain.process_data('create_desc_from_weak_descs');
            obj.sceneTest.process_data('create_desc_from_weak_descs','minX',obj.sceneTrain.minDesc,'maxX',obj.sceneTrain.maxDesc);
               
         
       end

       
    end
    
end
