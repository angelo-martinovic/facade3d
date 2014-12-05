%
%
%
%
%
%
%
%   Matlab class for PCL
%         used for cvpr'15
%
%
%
%
%
%%
%
classdef SScene_An
    
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
               
        source_file = '';
        read_file_name   = '';
    end
    
    
    methods
        
        
        function obj = SScene_An( )
%             obj.path_data    = path_data;
%             if iscell(file_name),
%                 for a=1:length(file_name)
%                     obj.source_file{a}  = fullfile(path_data,file_name{a});
%                 end
%             else
%                 obj.source_file  = fullfile(path_data,file_name);
%             end
%             obj.read_file_name = file_name;
        end
        
        
%         function out = get_adr(obj,type,par1,par2)
%              out = ['sdsd.sdsdsd'];
% %             path_desc =  '/esat/nihal/jknopp/3d_ret_recog_data/DT-SOL/monge428_27/';
% %             switch type
% %                 case 'desc'
% %                %     out = fullfile(path_desc,['desc_',par1,'-imSiz=',num2str(par2),'.mat']);
% %                 case 'cameras'
% %                     out = '/esat/sadr/amartino/monge428/reconstruction.nvm.cmvs/00/cameras_v2.txt';
% %                     %[datasetConfig.dataLocation,datasetConfig.cameras]
% %                 case 'wa_bboxes'
% %                       out = fullfile('/esat/nihal/jknopp/3d_ret_recog_data/DT-SOL/monge428_27/','3rd_layer',['result_NEW',num2str(par1),'_facadeid=',num2str(par2),'.mat']);
% %             end
%             
%             
%         end
%         

      
        
        %%=================================================================
        %%=================================================================
        %% GET SET
        %%=================================================================

        function obj = keep_spec_data_ids(obj,id_keep)
            obj.lindex  = obj.lindex(id_keep);
            obj.oindex  = obj.oindex(id_keep);
            obj.heigh   = obj.heigh(id_keep);
            obj.pts     = obj.pts(:,id_keep);
            obj.nxyz    = obj.nxyz(:,id_keep);
            obj.rgb     = obj.rgb(:,id_keep);
            if ~isempty(obj.dist2plane),    obj.dist2plane    = obj.dist2plane(:,id_keep); end;
            if ~isempty(obj.p_index),       obj.p_index       = obj.p_index(id_keep); end;
            if ~isempty(obj.desc),          obj.desc    = obj.desc(:,id_keep); end;
            if ~isempty(obj.si),            obj.si      = obj.si(:,id_keep);  end;
            if ~isempty(obj.heigh_inv),     obj.heigh_inv      = obj.heigh_inv(:,id_keep);  end;
            if ~isempty(obj.flag),          obj.flag      = obj.flag(id_keep);  end;
            if ~isempty(obj.facade_id),     obj.facade_id      = obj.facade_id(id_keep);  end;
        end

    
        function cmap  = get_class_colormap(obj,type)
            type = 'monge428';
            switch type
                case 'monge428'
                    cmap = [0 0 0;
                        255 0 0;         % red
                        255  255 0;      % yellow
                        128  0   255;    % purple
                        255  128 0;      % orange
                        0    0   255;    % blue
                        128  255 255;    % light blue
                        0    255 0;      % green
                        0    0   255];   % dark blue
            end
        end
                
        
        
        
        
        %%=================================================================
        %%=================================================================
        %% READ FUNCTION
        %%=================================================================     
        

        function obj = read_mat_data(obj,datasetConfig)
            %--- read plys with pts, rgb, labelings...
            fprintf('    ...SScene: reading plys %s \n',datasetConfig.dataLocation);
            [pts,~           ,vertexColor1] = read_ply( [datasetConfig.dataLocation,datasetConfig.groundTruthTrain] ,'pcl');   %%% pts + test labeling
            [~  ,~           ,vertexColor2] = read_ply( [datasetConfig.dataLocation,datasetConfig.groundTruthTest] ,'pcl');   %%% pts + train labeling
            [~  ,vertexNormal,vertexColor4] = read_ply( [datasetConfig.dataLocation,datasetConfig.pointCloud] ,'pcl');   %%% here is color + normals
            %--- add flags where is train and where is test
            obj.flag = uint16(pts(:,1)*0)';
            obj.flag(sum(vertexColor1')>0) = 1;
            obj.flag(sum(vertexColor2')>0) = 2;
            %--- read color map and join train+test to create one lindex
            tmp_lindex = vertexColor1+vertexColor2;
            un_li = obj.get_class_colormap;
            lindex = knnsearch(un_li,tmp_lindex);
            %---- assign values to data
            obj.pts         = pts';
            obj.nxyz        = vertexNormal';
            obj.rgb         = vertexColor4';
            obj.p_index     = 1:length(lindex);
            %--- facade split, it was computed from RF! 
            pts_split_label    = load([datasetConfig.dataLocation,datasetConfig.splitData]);
            obj.facade_id      = pts_split_label.splitLabels';
            %--- indexes
            obj.lindex  = lindex';
            obj.oindex  = lindex'*0;
            disp('   ...SScene:init: pts+lindex+rgb+nxy done');
            %--- depth
            path_dist2plane = [datasetConfig.dataLocation,datasetConfig.depth];
            kk = load(path_dist2plane);
            kk.depth(isnan(kk.depth)) = 0;
            obj.dist2plane = kk.depth';
            disp('   ...SScene:init: depth done');
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
            disp('   ...SScene:init: heigh(2,:) according to cameras done ')
        end
        
        
        
        
        
        %%=================================================================
        %%=================================================================
        %% EXPORT to 3ds and so on
        %%=================================================================
  
        function obj = export23ds(obj,adr_path,varargin)
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
            %--- print I did soemthing
            disp(['     ...exported ',num2str(size(data2save.pts)),' points to : ',adr_mesh]);
        end
        
        
        function export_as_full_pcl_data(obj,datasetConfig,cprb,path2save)
            fprintf('  ...saving result\n');
            pts_full = read_ply( [datasetConfig.dataLocation,datasetConfig.groundTruthTrain] ,'pcl');
            i_pcl2full = knnsearch(obj.pts',pts_full);
            
            cprb_full = cprb(i_pcl2full);
            %cprb_full(full_pcl.flag~=2)=0;
            
            %--- save to andelo
            cmap = obj.get_class_colormap();
%             path2save =  get_adr('L1_labeling',datasetConfig,type);%'[ADD.data.dtsol,'/2andelo/full_pcl_labeling_',input_type_into_3rd,'_3D3rdLayer.ply'];
            checkAdr_and_createDir( path2save );
            ExportMesh(path2save , pts_full ,[],cmap(cprb_full+1,:),[],[]);
            fprintf(['   ...result saved as pcl in the full-pcl.ply format for evaluation. path=',path2save]);
        end
        
        
       
        
        
        %%=================================================================
        %%=================================================================
        %% dataset processing
        %%=================================================================
        %--- fucntions to add negative descs and so on...
        function [obj,out1,out2,out3,out4,out5] = process_data(obj,method,varargin)
            global ADD CFG
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
            p.addOptional('datasetConfig', []);
            p.parse(varargin{:}); fldnames = fieldnames(p.Results); for a=1:length(fldnames), eval([fldnames{a},' = p.Results.',fldnames{a},';']); end;
            out1=[]; out2=[]; out3=[]; out4=[]; out5=[];
            switch method

                case 'monge428_delete_background_crap'
                    obj = obj.keep_spec_data_ids(obj.lindex~=1);
                    obj.lindex = obj.lindex -1;
                    if 1 && ~isempty(obj.dist2plane),
                        tr_dist2plane = .8;
                        disp(['...also deleteing points that has >',num2str(tr_dist2plane),' dist2plane...']);
                        obj = obj.keep_spec_data_ids(abs(obj.dist2plane)<tr_dist2plane);
                    end
                    
                case 'compute_get_si', %%% if desc does not exists, compute it, otherwise, jsut load and store it in obj.desc where it shold be :) 
                    disp('  ::SScene.m:: calculating/reading desc');
                    obj.si = [];
                    for imSize = si_dimensions;
                        %path_desc = scene.get_adr('desc','spinImagePC',imSiz);
                        path_desc = get_adr('desc3d',datasetConfig,'spinImagePC',imSize);
                        if ~exist(path_desc,'file');
                            [~,i] = sort(obj.pts(3,:)); %%% if they are not sorted, SI does not work!!!!
                            pts_sorted = obj.pts(:,i);
                            %--- compute desc
                            binSize = 8;  %%% bins in that radius
                            tic; 
                            spinImgs = compSpinImages(pts_sorted', imSize, binSize, 10, datasetConfig.parallel);
                            fprintf('done in %fsec\n',toc);
                            spinImgs = spinImgs(:,[3 4 5 6 7],:); %%% border is nothing...
                            desc = zeros(binSize*5 , size(pts_sorted,1));  %%% 3 is relevant that I took three dimensions 
                            for a=1:size(spinImgs,3),
                                tdesc = reshape(spinImgs(:,:,a),1,[]);
                                desc(:,a) = tdesc';
                            end
                            %--- sort back
                            unsorted = 1:size(obj.pts,2);
                            i_sort_back = i;
                            i_sort_back(i) = unsorted;
                            desc = desc(:,i_sort_back);
                            %--- check dir and save
                            checkAdr_and_createDir(path_desc);
                            save(path_desc,'desc');
                        end
                        t = dir(path_desc);
                        fprintf('  SScene:reading desc from %s, created at:%s\n',path_desc,t.date);
                        t = load(path_desc);
                        obj.si = [obj.si ; t.desc];
                        clear t;
                    end
                    
                case 'create_desc_from_weak_descs',
                    disp('  ::SScene.m:: adding si,rgb,si+dist2plane :) ... to desc + normalize it!');
                    lab = rgb2lab(obj.rgb')';
                    X_si = obj.si;
                    X          = [obj.rgb ; lab ; obj.heigh ; obj.heigh_inv ; obj.nxyz ; obj.dist2plane ; X_si];
                    if isempty(minX), minX = min(X,[],2); end;
                    X    = bsxfun(@plus , X , -minX);
                    if isempty(maxX), maxX = max(X,[],2); end;
                    obj.desc   = bsxfun(@times , X , 1./maxX);
                    out1 = minX;
                    out2 = maxX;
                   

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
                fprintf('\b\b\b\b\b%5i',k);
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
                disp([' ...SScene_an:fit_bbox2center:  windows saved to file ',path_save]);
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


