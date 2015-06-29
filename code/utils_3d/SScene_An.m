%
%

%
%
%   Matlab class for PCL
%         used for cvpr'15
%
%
%%
%
classdef SScene_An < handle
    
    properties (GetAccess = 'public', SetAccess = 'public')
        pts;
        tri;
        desc;
        si;
        dist2plane;
        nxyz;
        heigh;
        heigh_inv; 
        lindex;
        oindex;
        p_index;
        file_index;
        flag;
        rgb;
        facade_id;
               
        origIndices;
        
        source_file = '';
        read_file_name   = '';
        
        minDesc;
        maxDesc;

    end
    
    
    methods
        
        
        function obj = SScene_An( )

        end
        
        function obj2 = copy(obj)
            obj2 = SScene_An();
            obj2.pts = obj.pts;
            obj2.tri = obj.tri;
            obj2.desc = obj.desc;
            obj2.si = obj.si;
            obj2.dist2plane = obj.dist2plane;
            obj2.nxyz = obj.nxyz;
            obj2.heigh = obj.heigh;
            obj2.heigh_inv = obj.heigh_inv; 
            obj2.lindex = obj.lindex;
            obj2.oindex = obj.oindex;
            obj2.p_index = obj.p_index;
            obj2.file_index = obj.file_index;
            obj2.flag = obj.flag;
            obj2.rgb = obj.rgb;
            obj2.facade_id = obj.facade_id;

            obj2.origIndices = obj.origIndices;

            obj2.source_file =  obj.source_file;
            obj2.read_file_name  = obj.read_file_name;
        end
        
            
        
        %%=================================================================
        %%=================================================================
        %% GET SET
        %%=================================================================

        function keep_spec_data_ids(obj,id_keep)
            obj.origIndices     = find(id_keep);
            
            if ~isempty(obj.lindex),        obj.lindex          = obj.lindex(id_keep);end;
            if ~isempty(obj.oindex),        obj.oindex          = obj.oindex(id_keep);end;
            if ~isempty(obj.heigh),         obj.heigh           = obj.heigh(id_keep);end;
            if ~isempty(obj.pts),           obj.pts             = obj.pts(:,id_keep);end;
            if ~isempty(obj.nxyz),          obj.nxyz            = obj.nxyz(:,id_keep);end;
            if ~isempty(obj.rgb),           obj.rgb             = obj.rgb(:,id_keep);end;
            if ~isempty(obj.dist2plane),    obj.dist2plane      = obj.dist2plane(:,id_keep); end;
            if ~isempty(obj.p_index),       obj.p_index         = obj.p_index(id_keep); end;
            if ~isempty(obj.desc),          obj.desc            = obj.desc(:,id_keep); end;
            if ~isempty(obj.si),            obj.si              = obj.si(:,id_keep);  end;
            if ~isempty(obj.heigh_inv),     obj.heigh_inv       = obj.heigh_inv(:,id_keep);  end;
            if ~isempty(obj.flag),          obj.flag            = obj.flag(id_keep);  end;
%             if ~isempty(obj.facade_id),     obj.facade_id      = obj.facade_id(id_keep);  end;
        end

           
        %%=================================================================
        %%=================================================================
        %% READ FUNCTION
        %%=================================================================     
        

        function obj = read_mat_data(obj)
            datasetConfig = DatasetConfig.getInstance();
            %--- read plys with pts, rgb, labelings...
            dl = DispatchingLogger.getInstance();
            dl.Log(VerbosityLevel.Debug,sprintf(' - SScene: reading plys %s \n',datasetConfig.dataLocation));
            [pts,~           ,vertexColor1] = read_ply( [datasetConfig.dataLocation,datasetConfig.groundTruthTrain] ,'pcl');   %%% pts + test labeling
            [~  ,~           ,vertexColor2] = read_ply( [datasetConfig.dataLocation,datasetConfig.groundTruthTest] ,'pcl');   %%% pts + train labeling
            [~  ,vertexNormal,vertexColor4] = read_ply( [datasetConfig.dataLocation,datasetConfig.pointCloud] ,'pcl');   %%% here is color + normals
            %--- add flags where is train and where is test
            obj.flag = uint16(pts(:,1)*0)';
            obj.flag(sum(vertexColor1')>0) = 1;
            obj.flag(sum(vertexColor2')>0) = 2;
            %--- read color map and join train+test to create one lindex
            tmp_lindex = vertexColor1+vertexColor2;
            un_li = round(255*datasetConfig.cm);
            lindex = uint8(knnsearch(un_li,double(tmp_lindex)));
            %---- assign values to data
            obj.pts         = pts';
            obj.nxyz        = vertexNormal';
            obj.rgb         = vertexColor4';
            obj.p_index     = 1:length(lindex);
            %--- facade split, it was computed from RF! 
%             pts_split_label    = load([datasetConfig.dataLocation,datasetConfig.splitData]);
%             obj.facade_id      = pts_split_label.splitLabels';
            %--- indexes
            obj.lindex  = lindex';
            obj.oindex  = lindex'*0;
            dl.Log(VerbosityLevel.Debug,sprintf(' - SScene:init: pts+lindex+rgb+nxy done.\n'));
        end
        
        function obj = calc_simple_features(obj)
            datasetConfig = DatasetConfig.getInstance();
            dl = DispatchingLogger.getInstance();
            %--- depth
            calculateDepthMaps();
            path_dist2plane = [datasetConfig.dataLocation,datasetConfig.depth];
            kk = load(path_dist2plane);
            kk.depth(isnan(kk.depth)) = 0;
            obj.dist2plane = kk.depth';
            dl.Log(VerbosityLevel.Debug,sprintf(' - SScene:init: feature: depth done.\n'));
            %--- heigh
            cams = ImportCameras([datasetConfig.dataLocation,datasetConfig.cameras]);
            cams = cellfun(@(x) x.cameraPosition' , cams,'UniformOutput',0);
            cams = [cams{:}];
            i = knnsearch(cams([1 3],:)',obj.pts([1 3],:)');
            obj.heigh     = (obj.pts(2,:)-cams(2,i));% ./ precomp(2,i);
            %%%--- depth from top :) =inverse heigh
            obj.heigh_inv = obj.heigh*0;
            for a=unique(i)',
                obj.heigh_inv(i==a) = -obj.heigh(i==a)+min(obj.heigh(i==a));
            end
            dl.Log(VerbosityLevel.Debug,sprintf(' - SScene:init: feature: height and inverse height done.\n'));
        end
        
        
        
        
        
        %%=================================================================
        %%=================================================================
        %% EXPORT to 3ds and so on
        %%=================================================================
  
        function obj = export23ds(obj,adr_path,varargin)
            error('Unsupported code!');
            p = inputParser;
            p.addOptional('N', 1);
            p.addOptional('n_closest_pts', -1);
            p.addOptional('pt_arch', []);
            p.addOptional('col', []);
            p.addOptional('vote_pts', []);
            p.addOptional('vote_lines', []);
            p.addOptional('other_pts', []);
            p.addOptional('ids_show', []);
            p.addOptional('project_color2angelo', 0);
            p.parse(varargin{:}); fldnames = fieldnames(p.Results); for a=1:length(fldnames), eval([fldnames{a},' = p.Results.',fldnames{a},';']); end;
            adr_mesh = [adr_path,'.pc_max'];
            adr_vts  = [adr_path,'.vt_max'];
            adr_lns  = [adr_path,'.ln_max'];
            adr_pt   = [adr_path,'.pt_max'];
            if project_color2angelo,
                colmap = obj.get_class_colormap(); colmap = colmap(2:end,:)';
                     if ~isempty(obj.rgb)
                     rgb_  = ((obj.rgb/255).^4)*255;
                     col = (colmap(:,col)+rgb_*1)/2;
                     %col = rgb_;;
                 else
                    col = colmap(:,col);
                 end
            end
            %--- N closest points in scene
            if ~isempty(pt_arch),
                ids_pts = knnsearch(obj.pts',pt_arch','K',n_closest_pts);
            else
                ids_pts = 1:size(obj.pts,2);
            end
            if ~isempty(ids_show),
                ids_pts=ids_show;
            end
            data2save.pts = obj.pts(:,ids_pts);
            if ~isempty(col),
                if size(col,1)==3,
                    col = col(:,ids_pts)';
                else
                    col = col(ids_pts);
                end
            end
%             rad = max(max(data2save.pts,[],2)-min(data2save.pts,[],2)); %% radius of selected data
%             %--- also only relevant points in votes or whatever adn save
%             if ~isempty(vote_pts),
%                 id = sum(bsxfun(@plus , vote_pts(1:3,:) , -pt_arch).^2,1)<(rad/2)^2;
%                 data2save.vt = vote_pts(1:3,id);
%                 data2save.vt_weight = vote_pts(4,id);
%                 io_save_data3dsmax(adr_vts,'meshbin',data2save.vt,[],data2save.vt_weight,'center',0);
%                 disp(['     ...exported ',num2str(size(data2save.vt)),' vote_pts to : ',adr_vts]);
%             end
%             %--- put fitted model, save other points into pt
%             if ~isempty(other_pts),
%                 if size(other_pts,1)==3, other_pts(4,:) = 1; end; %%% if no weight given
%                 if ~isempty(pt_arch), %%% if no arch point, just save all :)
%                     id = sum(bsxfun(@plus , other_pts(1:3,:) , -pt_arch).^2,1)<(rad/2)^2;
%                 else
%                     id = 1:size(other_pts,2);
%                 end
%                 data2save.pt = other_pts(1:3,id);
%                 data2save.pt_weight = other_pts(4,id);
%                 io_save_data3dsmax(adr_pt,'meshbin',data2save.pt,[],data2save.pt_weight,'center',0);
%                 disp(['     ...exported ',num2str(size(data2save.pt)),' other_pts to : ',adr_pt]);
%             end
%             %--- save lines into lines, i.e. where they vote :-)
%             if ~isempty(vote_lines),
%                 id = sum(bsxfun(@plus , vote_lines(1:3,:) , -pt_arch).^2,1)<(rad/2)^2;
%                 data2save.pt = [vote_lines(1:3,id) ; obj.pts(:,vote_lines(4,id))];
%                 data2save.pt_weight = vote_lines(5,id);
%                 io_save_data3dsmax(adr_lns,'meshbin',data2save.pt,[],data2save.pt_weight,'center',0);
%                 disp(['     ...exported ',num2str(size(data2save.pt)),' vote_lines to : ',adr_lns]);
%             end
            %--- save mesh
            io_save_data3dsmax(adr_mesh,'meshbin',data2save.pts,[],col','center',0);
            %--- print I did something
            dl.Log(VerbosityLevel.Debug,sprintf(' - exported %d points to : %s\n',num2str(size(data2save.pts)),adr_mesh));
        end
        
        
        function export_as_full_pcl_data(obj,cprb,path2save)
            datasetConfig = DatasetConfig.getInstance();
            dl = DispatchingLogger.getInstance();
            dl.Log(VerbosityLevel.Debug,sprintf(' - - saving result\n'));
 
            pts_full = read_ply( [datasetConfig.dataLocation,datasetConfig.groundTruthTrain] ,'pcl');
            
%             dl.Log(VerbosityLevel.Debug,sprintf(' - - performing KNN search...\n'));
%             i_pcl2full = knnsearch(obj.pts',pts_full);
            
            cprb_full = zeros(size(pts_full,1),1);
            cprb_full(obj.origIndices) = cprb;
%             dl.Log(VerbosityLevel.Debug,sprintf(' - - - done.\n'));
            
%             cprb_full = cprb(i_pcl2full);
            %cprb_full(full_pcl.flag~=2)=0;
            
            %--- save to andelo
            cmap = round(datasetConfig.cm*255);%obj.get_class_colormap();
%             path2save =  get_adr('L1_labeling',type);%'[ADD.data.dtsol,'/2andelo/full_pcl_labeling_',input_type_into_3rd,'_3D3rdLayer.ply'];
            checkAdr_and_createDir( path2save );
            ExportMesh(path2save , pts_full ,[],cmap(cprb_full+1,:),[],[]);
            dl.Log(VerbosityLevel.Debug,sprintf(' - - result saved as pcl in the full-pcl.ply format for evaluation. path=%s\n',path2save));
        end
        
        
       
        
        
        %%=================================================================
        %%=================================================================
        %% dataset processing
        %%=================================================================
        %--- fucntions to add negative descs and so on...
        function [out1,out2,out3,out4,out5] = process_data(obj,method,varargin)
%             global ADD CFG
            dl = DispatchingLogger.getInstance();
            p = inputParser;
            p.addOptional('cls', []);
            p.addOptional('ids', []);
            p.addOptional('minX', []);
            p.addOptional('maxX', []);
            p.addOptional('sample', -1);
            p.addOptional('si_dim_reduction', 0);
            p.addOptional('D', []);
            p.addOptional('V', []);
            p.addOptional('si_dimensions', [.2 .25 .3 .35 .4]);
            p.addOptional('binSize', []);
            p.addOptional('imgW', []);
            p.addOptional('split', []);
            p.parse(varargin{:}); fldnames = fieldnames(p.Results); for a=1:length(fldnames), eval([fldnames{a},' = p.Results.',fldnames{a},';']); end;
            out1=[]; out2=[]; out3=[]; out4=[]; out5=[];
            switch method

                case 'monge428_delete_background_crap'
                    obj.keep_spec_data_ids(obj.lindex~=1);
                    obj.lindex = obj.lindex -1;
                    if 1 && ~isempty(obj.dist2plane),
                        tr_dist2plane = .8;
                        
                        dl.Log(VerbosityLevel.Debug,...
                            sprintf(' - - ...also deleting points which have > %f dist2plane.',...
                            num2str(tr_dist2plane)));
                        obj.keep_spec_data_ids(abs(obj.dist2plane)<tr_dist2plane);
                    end
                    
                case 'compute_get_si', %%% if desc does not exists, compute it, otherwise, jsut load and store it in obj.desc where it shold be :) 
                    obj.si = [];
                    
                    [~,i] = sort(obj.pts(3,:)); %%% if they are not sorted, SI does not work!!!!
                    pts_sorted = obj.pts(:,i);
                    %--- sort back
                    unsorted = 1:size(obj.pts,2);
                    i_sort_back = i;
                    i_sort_back(i) = unsorted;
                    
                    % Variables required when running in parallel
                    dims = si_dimensions;
                    subset = split;
                    binSz = binSize;
                    imW = imgW;
                    
                    nDim = length(dims);
                    desc_all = cell(1,nDim);
                    paths_all = cell(1,nDim);
                    tic;
                    for dimIdx = 1:nDim % use parfor(dimIdx = 1:nDim,dc.nWorkers) for parallel
                        imSize = dims(dimIdx);
                        paths_all{dimIdx} = get_adr('desc3d',['spinImagePC_split' subset],imSize);
                        if ~exist(paths_all{dimIdx},'file');
                            %--- compute desc
                            dl.Log(VerbosityLevel.Debug,sprintf(' - - SScene:: calculating spin images with size %.2f\n',imSize));
                            spinImgs = compSpinImages(pts_sorted', imSize, binSz, imW);%datasetConfig.parallel);
%                             spinImgs = compSpinImages2(pts_sorted', imSize, binSize, imgW);%datasetConfig.parallel);
                            % ANDELO: attempt to use MATLAB's internal
                            % kd_tree implementation, paralelizable
                            % but requires MASSIVE amounts of memory
%                             tic; 
%                             spinImgs2 = compSpinImages2(pts_sorted', imSize, binSize, 10);%datasetConfig.parallel);
%                             fprintf('done in %fsec\n',toc);
%                             assert(isequal(spinImgs,spinImgs2));

                            spinImgs = spinImgs(:,[3 4 5 6 7],:); %%% border is nothing...
%                             desc_single = zeros(binSz*5 , size(pts_sorted,1));  %%% 3 is relevant that I took three dimensions 
%                             for a=1:size(spinImgs,3),
%                                 tdesc = reshape(spinImgs(:,:,a),1,[]);
%                                 desc_single(:,a) = tdesc';
%                             end
                            desc_single = reshape(spinImgs,binSz*5,size(spinImgs,3));
%                             assert(isequal(desc_single,d2));
                            desc_all{dimIdx} = desc_single(:,i_sort_back);
                            
                        end
                    end
                    dl.Log(VerbosityLevel.Info,sprintf(' - -  done in %.2fsec\n',toc));
                    
                    for dimIdx = 1:nDim
                        desc = desc_all{dimIdx}; %#ok<PROP>
                        if ~isempty(desc) %#ok<PROP>
                            % just calculated, save it
                            checkAdr_and_createDir(paths_all{dimIdx});
                            save(paths_all{dimIdx},'desc');
                            obj.si = [obj.si ; desc];%#ok<PROP>
                        else
                            % file exists, load it
                            t = dir(paths_all{dimIdx});
                            dl.Log(VerbosityLevel.Debug,...
                                sprintf(' - SScene:reading desc from %s, created at:%s\n',...
                                paths_all{dimIdx},t.date));
                            
                            t = load(paths_all{dimIdx});
                            obj.si = [obj.si ; t.desc];
                            clear t;
                        end
                    end
                            
                    
                case 'create_desc_from_weak_descs',
                    dl.Log(VerbosityLevel.Debug,sprintf(' - SScene:: adding si,rgb,si+dist2plane :) ... to desc + normalize it!\n'));
                    X  = [double(obj.rgb) ;
                        rgb2lab(obj.rgb')' ; 
                        obj.heigh ;
                        obj.heigh_inv ;
                        double(obj.nxyz) ;
                        obj.dist2plane ;
                        obj.si];
                    
                    % Get rid of individual descriptors
                    obj.heigh=[];
                    obj.heigh_inv=[];
                    obj.dist2plane=[];
                    obj.si=[];
                    
                    if isempty(minX), minX = min(X,[],2); end;
                    X    = bsxfun(@plus , X , -minX);
                    if isempty(maxX), maxX = max(X,[],2); end;
                    obj.desc   = bsxfun(@times , X , 1./maxX);
                    
                    obj.minDesc = minX;
                    obj.maxDesc = maxX;
                    
%                     out1 = minX;
%                     out2 = maxX;
                   

            end
        end
        

        
        function obj = create_oidx_from_lidx (obj , varargin)
            p = inputParser;
            p.addOptional('cprb',[]);
            p.addOptional('cl',1);
            p.addOptional('max_objs',8e3);
            p.addOptional('K',7);
            p.parse(varargin{:}); fldnames = fieldnames(p.Results); for a=1:length(fldnames), eval([fldnames{a},' = p.Results.',fldnames{a},';']); end;
          
            obj.oindex = obj.lindex*0;
            unmet = logical(obj.lindex*0+1);
            if isempty(cprb),
                cprb    = obj.lindex;
            end
            cl_idx = cprb==cl;

            [edges] = knnsearch(obj.pts',obj.pts','K',K);
            edges = edges';
            for k=1:max_objs,  %%% use one point (k) and propagate unitll you see only other classes
%                 fprintf('\b\b\b\b\b%5i',k);
                ik = find(cl_idx,1);
                listgo = edges(:,ik)';
                obj.oindex(ik)=k;
                while sum(cprb(listgo)==cl)>0,
                    listgo = listgo(cl_idx(listgo) & unmet(listgo));
                    unmet(listgo)  = 0;
                    cl_idx(listgo) = 0;
                    obj.oindex(listgo) = k;
                    toadd  = unique(edges(:,listgo))';
                    toadd  = toadd(unmet(toadd));
                    listgo = toadd( cprb(toadd)==cl);
                end
            end
            obj.oindex(cprb~=cl) = 0;
        end
        
        
        %%=================================================================
        %%=================================================================
        %% CALCULATIONS
        %%=================================================================

%                
%         function [fit , labeling_cprb] = fit_centers_into_cprb(obj,cprb,clIdxs)
%             disp('   ...SScene_An: find centers given RF results');
%             global train_data
%             for c = clIdxs;
%                 fprintf('  c=%i  ',c);
%                 num_pts_in_o = [];
%                 %--- find size of the object...
%                 for o=unique(train_data.oindex),
%                     if o==0, continue;  end;
%                     num_pts_in_o = [num_pts_in_o sum(train_data.oindex==o)];
%                 end
%                 ideal_n_for_win = mean(num_pts_in_o(num_pts_in_o>20));
%                 %--- find centers uisng precomputed RF prob.
%                 scene_regions_oidx_cprb = obj.create_oidx_from_lidx('cprb',cprb,'K',20,'cl',c);
%                 labeling_cprb(:,c) = scene_regions_oidx_cprb.oindex;
%                 clear scene_regions_oidx_cprb; fit_rf{c}.pos=[]; fit_rf{c}.val=[];
%                 for o = unique(labeling_cprb(:,c)'),
%                     if i==0, continue;  end;
%                     idx_o = labeling_cprb(:,c)==o;
%                     fit_rf{c}.pos = [fit_rf{c}.pos , mean(obj.pts(:,idx_o),2)];
%                     n_pts_win = sum(idx_o);
%                     if n_pts_win<ideal_n_for_win*.5,
%                         score = get_gauss(ideal_n_for_win , 50 , n_pts_win);
%                     else
%                         score = get_gauss(ideal_n_for_win , 300 , n_pts_win);
%                     end
%                     fit_rf{c}.val = [fit_rf{c}.val , score];
%                 end
%                 idx_closest  = knnsearch(obj.pts',fit_rf{c}.pos','K',1);
%                 fit_rf{c}.id = idx_closest';
%                 trs = .75;
%                 fit{c}.id = fit_rf{c}.id(fit_rf{c}.val>trs);
%                 fit{c}.val = fit_rf{c}.val(fit_rf{c}.val>trs);
%                 fit{c}.pos = fit_rf{c}.pos(:,fit_rf{c}.val>trs);
%                 if 0,
%                     rnd_perm = randperm(max(labeling_cprb));
%                     labeling_cprb(labeling_cprb~=0) = rnd_perm(labeling_cprb(labeling_cprb~=0));
%                     cmap = scene.get_class_colormap();cmap = cmap(2:end,:);colors_show = uint8([(cmap(scene.lindex,:)+scene.rgb'*2)/3  ,  (cmap(cprb,:)+scene.rgb'*2)/3  ,  (cmap((labeling_cprb>1)*6+1,:)+scene.rgb')/2]); scene.plot_opengl_scene([d_vis,labeling_cprb'],'Q_rgb',colors_show);
%                 end
%             end
%             fprintf('   ....done :)');
%         end
        
        function wins = fit_bbox2center (obj,fit,varargin)
            p = inputParser;
            p.addOptional('c',1);
            p.addOptional('path_save', '');
            p.parse(varargin{:}); fldnames = fieldnames(p.Results); for a=1:length(fldnames), eval([fldnames{a},' = p.Results.',fldnames{a},';']); end;
        
            %window = [0 -.03 -47.5 ; 0 -.01 -47.29 ; 0 0.18 -47.29 ; 0 .17 -47.49]';   window = bsxfun(@plus , window , -mean(window,2));  
            w = .11; h = .1;
            window = [0 -w -w ; 0 -h w ; 0 w w ; 0 h -h]';
            wins{1} = [];
            for f=1:length(fit{c}.pos),
                twin = project_pts_to_plane(window , scene.pts(:,fit{c}.id(f)) , scene.nxyz(:,fit{c}.id(f)));
                twin = bsxfun(@plus , twin , -mean(twin,2));
                wins{f} = bsxfun(@plus,twin,scene.pts(:,fit{c}.id(f)));
            end
            if 0,
                obj.plot_scene('Q',obj.lindex','id_show',length(scene.lindex)-1e5:length(scene.lindex)); hold on;
                alph = .7;
                for f = 1:length(wins);
                    col = hsv2rgb([f/length(wins) .5 .8]);
                    h = patch(wins{f}(1,:),wins{f}(2,:),wins{f}(3,:),col,'EdgeAlpha',1,'FaceAlpha',alph,'FaceColor',col);
                end
            end
            if ~isempty(path_save)
                global ADD
                path_save = fullfile(ADD.data.dtsol,'res2angelo','bbox_result01.mat');
                save(path_save,'wins');
                dl.Log(VerbosityLevel.Debug,sprintf(' - SScene_an:fit_bbox2center:  windows saved to file %s',path_save));
            end
            
            
        end
        
   
        
%         %%=================================================================
%         %%=================================================================
%         %% gt 
%         % it returns centers and oindex of that points
%         %%=================================================================
%         function [pts_objCen,pts_oindex] = get_centers(obj,cl_id)
%             global ADD
%             mthd = 'labels';
%             pts_oindex = [];
%             switch mthd
%                 case 'labels'
%                     i_obj = obj.oindex; %%% indexes of objects that will be only from cl_idx
%                     if ~isempty(cl_id)
%                         i_obj(obj.lindex ~= cl_id) = 0;
%                     end
%                     pts_objCen = [];
%                     for a=unique(i_obj),
%                         if a==0, continue; end;
%                         pts_objCen  = [pts_objCen  [mean(obj.pts(:,obj.oindex == a),2)]];
%                         pts_oindex = [pts_oindex a];
%                     end
%             end
%         end
%        
        function [class_names] = get_class_names(obj,id)
            class_names{1} = 'window';
            class_names{2} = 'wall';
            class_names{3} = 'balc.';
            class_names{4} = 'door';
            class_names{5} = 'roof';
            class_names{6} = 'sky';
            class_names{7} = 'shop';
            if exist('id','var') && ~isempty(id),
                class_names = class_names{id};
            end
        end

         %%=================================================================
         %%=================================================================
         %% PLOT FUNCTIONS
         %%=================================================================

                
        %------------------------------------------------------------------
        function hf = plot_scene(obj,varargin)
            p = inputParser;
            p.addOptional('sample', 1); %% smapling points
            p.addOptional('id_show', []); %% point i want ot show (only for only scene plotting
            p.addOptional('fignum', []);
            p.addOptional('FaceAlpha', 1);
            p.addOptional('pts', []);
            p.addOptional('pts_size', 20);
            p.addOptional('Q', []);
            p.addOptional('rgb', []);
            p.parse(varargin{:}); fldnames = fieldnames(p.Results); for a=1:length(fldnames), eval([fldnames{a},' = p.Results.',fldnames{a},';']); end;
            set(0,'DefaultFigureRenderer','opengl');
            if ~isempty(fignum), figure(fignum); end
            
            if isempty(Q),
                if ~isempty('rgb'),
                    Q = rgb;
                    if max(max(Q))>30, Q = Q/256; end;
                else
                    Q = obj.pts(3,:);%    Q = obj.file_index;
                end
            end
            if isempty(id_show) && p.Results.sample>=1,
                id_show = 1:p.Results.sample:length(Q);
            end
            hf = honza_scatter([obj.pts(:,id_show)],Q(id_show)*0+pts_size,Q(id_show,:),'filled');
            colormap('jet'); colorbar;
            set(gcf, 'Color', [1 1 1]); axis equal off;
        end
        
        
    end
end


