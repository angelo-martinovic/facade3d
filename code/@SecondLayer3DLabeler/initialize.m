function initialize(obj,scene)
    dl = DispatchingLogger.getInstance();
    
    % Load point cloud
%     obj.idxs = scene.p_index(:);
%     [obj.pointsFull,~,colorsFull] = ReadPCLFromPly(get_adr('pcl_gt_test',obj.config));
    obj.pointsFull = scene.pts';
%     colorsFull = scene.
    % Indices of the labeled subset
%     obj.idxs  = find(sum(colorsFull,2)~=0);
    obj.idxs = find(scene.flag==2)';

    dl.Log(VerbosityLevel.Info,sprintf(' - Found the following unary potentials: \n'));
    probs = [];
    weights = [];
    unaryNames = {};
  
    % Load 3D labeling
    filename = get_adr('pcl_unaries',obj.config,'3D');
    if exist(filename,'file')
        s_L = load(filename);
        assert(size(s_L.prb,1)==obj.config.nClasses,'Wrong file format!'); % TODO
        assert(size(s_L.prb,2)==size(obj.idxs,1),'Wrong number of points!'); % TODO
        probs_3D = s_L.prb(1:obj.config.nClasses,:); 
        probs = cat(3,probs,probs_3D);
        unaryNames = [unaryNames {'3D'}];
        weights = [weights obj.config.c3D.crf.weight3DClassification];
        dl.Log(VerbosityLevel.Info,sprintf(' - - 3D classification\n'));
    else
        if obj.config.c3D.crf.weight3DClassification~=0
            dl.Log(VerbosityLevel.Error,sprintf(' - No 3D potentials found!\n'));
            fatal();
        end
    end


    % Load 2D labeling
    filename = get_adr('pcl_unaries',obj.config,'2D');
    if exist(filename,'file')
        s_L = load(filename);
        probs_2D_seg = s_L.unary(3+1:3+obj.config.nClasses,obj.idxs);
        probs = cat(3,probs,probs_2D_seg);
        unaryNames = [unaryNames {'2D'}];
        weights = [weights obj.config.c3D.crf.weight2DClassification];
        dl.Log(VerbosityLevel.Info,sprintf(' - - 2D classification\n'));

        if ~isempty(obj.config.c3D.crf.weights2DDetectors)
            nDet = length(s_L.unaryDet);
            detNames = cell(1,nDet);
            for i=1:nDet
                detNames{i} = s_L.unaryDet{i}.name;
                probs = cat(3,probs,s_L.unaryDet{i}.unary(3+1:3+obj.config.nClasses,obj.idxs));
                dl.Log(VerbosityLevel.Info,sprintf(' - - 2D detector: %s\n',detNames{i}));

            end
            unaryNames = [unaryNames detNames];
            weights = [weights obj.config.c3D.crf.weights2DDetectors];

            if nDet~=length(obj.config.c3D.crf.weights2DDetectors)
                dl.Log(VerbosityLevel.Error,...
                    sprintf(' - Mismatch between expected (%d) and received (%d) number of detectors!\n',...
                            length(obj.config.c3D.crf.weights2DDetectors),nDet));
                fatal();
            end
        end

    else
        if obj.config.c3D.crf.weight2DClassification~=0 
            dl.Log(VerbosityLevel.Error,sprintf(' - No 2D classification potentials found!\n'));
            fatal();
        end
    end

    % Setup unary potentials
    nUnary = length(unaryNames);
    obj.unaries = struct('unary',[],'name',[],'weight',[]);
    for i=1:nUnary
        p = probs(:,:,i);
        p(p==0) = eps;
        p = p-min(p(:));
        p = p/max(p(:));
        obj.unaries(i).unary = -log(bsxfun(@rdivide,p,sum(p)));
        obj.unaries(i).name = unaryNames{i};
        obj.unaries(i).weight = weights(i);
    end

end

