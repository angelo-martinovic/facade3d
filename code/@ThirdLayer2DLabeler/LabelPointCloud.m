function LabelPointCloud(obj,weights)

% TODO
classifier3DFilename = '/esat/nihal/jknopp/3d_ret_recog_data/DT-SOL/monge428_27/res2angelo/3D_layer1-2000-200.mat';

% Load point cloud
[pointsFull,~,colorsFull] = ReadPCLFromPly([obj.dirName obj.gtTestFilename]);

% Indices of the labeled subset
idxs = find(sum(colorsFull,2)~=0);
points = pointsFull(idxs,:);

fprintf('Found the following unary potentials: \n');
probs = [];
unaryNames = {};
% Load 3D labeling
if exist(classifier3DFilename,'file')
    s_L = load(classifier3DFilename);
    assert(size(s_L.dt,1)==3+2*obj.nClasses,'Wrong file format!');
    assert(size(s_L.dt,2)==size(idxs,1),'Wrong number of points!');
    probs_3D = s_L.dt(3+1:3+obj.nClasses,:); 
    probs = cat(3,probs,probs_3D);
    unaryNames = [unaryNames {'3D'}];
    fprintf('3D\n');
else
    warning('No 3D potentials found!');
end


% Load 2D labeling
filename = [obj.dirName 'work/pcl/probs/' obj.modelName '_2Dunary.mat'];
if exist(filename,'file')
    s_L = load(filename);
    probs_2D_seg = s_L.unary(3+1:3+obj.nClasses,idxs);
    probs = cat(3,probs,probs_2D_seg);
    unaryNames = [unaryNames {'2D'}];
    fprintf('2D\n');
    
    nDet = length(s_L.unaryDet);
    detNames = cell(1,nDet);
    for i=1:nDet
        detNames{i} = s_L.unaryDet{i}.name;
        probs = cat(3,probs,s_L.unaryDet{i}.unary(3+1:3+obj.nClasses,idxs));
        fprintf('%s\n',detNames{i});
    end
    unaryNames = [unaryNames detNames];
    
else
    warning('No 2D potentials found!');  
end

nUnary = length(unaryNames);
if nUnary<1
    error('No unary potentials defined!');
end
if length(weights)~=nUnary+1
    error('Weight vector size mismatch! %d elements expected, %d received.',nUnary+1,length(weights));
end

%% Unary potentials
unaries = struct('unary',[],'name',[],'weight',[]);
for i=1:nUnary
    p = probs(:,:,i);
    p(p==0) = eps;
    p = p-min(p(:));
    p = p/max(p(:));
    unaries(i).unary = -log(bsxfun(@rdivide,p,sum(p)));
    unaries(i).name = unaryNames{i};
    unaries(i).weight = weights(i);
end
pairwiseCost = weights(nUnary+1);

[labelsMAP, labelsCRF, outputPCLName, unarySub] = LabelPointCloudWithUnaries(points,unaries,pairwiseCost,obj.modelName);
unary = zeros(obj.nClasses,size(pointsFull,1));
unary(:,idxs) = unarySub;
save([obj.dirName 'work/pcl/probs/' outputPCLName '.mat'],'unary');
save([obj.dirName 'work/pcl/probs/' outputPCLName '_3DCRF.mat'],'unary');

colors =  zeros(size(pointsFull,1),3);
colors(idxs,:) = obj.cm(labelsMAP+1,:);
ExportMesh([obj.dirName 'work/pcl/models/' outputPCLName '.ply'],pointsFull,[],colors,[],[]);

colors =  zeros(size(pointsFull,1),3);
colors(idxs,:) = obj.cm(labelsCRF+1,:);
ExportMesh([obj.dirName 'work/pcl/models/' outputPCLName '_3DCRF.ply'],pointsFull,[],colors,[],[]);
    
obj.splitName = [outputPCLName '_3DCRF'];

end

function [labelsMAP, labelsCRF, outputPCLName, unary] = LabelPointCloudWithUnaries(points,unaries,pairwiseCost,baseName)
    outputPCLName = baseName;
    unary = 0;
    connector = '_';
    fprintf('Using the following energy function:\n');
    for i=1:length(unaries)
        if unaries(i).weight>0
            outputPCLName = [outputPCLName connector unaries(i).name];   
            fprintf('%f*%s + ',unaries(i).weight,unaries(i).name);
            if strcmp(connector,'_')
                connector='+';
            end
            unary = unary + unaries(i).weight * unaries(i).unary;
        end
    end
    fprintf(' %f*pairwise\n',pairwiseCost);
    if sum(unary(:))==0
        error('No unaries found!');
    end
    
    n = size(points,1);
    [~,labelsMAP] = min(unary);

    %% Graph cuts
    
    %% Pairwise potentials
    % Find k nearest neighbors
    k = 4;
    [idx,d]=knnsearch(points,points,'k',k+1);

    % Remove 1st column, corresponding to same element
    idx = idx(:,2:end);
    d = d(:,2:end);

    % Create sparse adjacency matrix
    % i = [11112222...nnnn]
    ii = zeros(1,k*n);
    for i=1:k
        ii(i:k:end)=1:n;
    end

    jj = reshape(idx',1,k*n);
%     ss = reshape(d',1,k*n);

    % Pairwise costs: w = exp(-(d/sigma)^2)
%     sigma = mean(ss(:));
    ss = 1;%3* exp( - (ss/sigma).^2  );    % When ss==mean(ss), penalty should be 1

    pairwise = sparse(ii,jj,ss,n,n);

    %% Label cost
    C = size(unary,1);
    labelcost = ones(C,C);
    labelcost(1:C+1:end)=0;

    lambda = pairwiseCost;
    pairwise = lambda * pairwise;

    segclass = labelsMAP-1;

    disp('Running the graph cut...');
    tic;
    [labelsCRF,E,Eafter] = GCMex(segclass, double(unary), pairwise, double(labelcost),0);
    CRFtime = toc;
    fprintf('Done. Elapsed time: %d\n',CRFtime);
    labelsCRF = labelsCRF + 1;

end

