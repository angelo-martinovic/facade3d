    addpath /users/visics/amartino/RNN_link/RNN/repo/source/GCMex2.3/
    addpath /users/visics/amartino/RNN_link/RNN/repo/source/
    
    cmvsLocation = '/esat/sadr/amartino/monge3dRight/reconstruction.nvm.cmvs/00/';
    
    % Load point cloud
    [points,normals,colors]=ReadPointCloudFromPly([cmvsLocation 'models/option-0000.ply']);
    
    n = length(points);
    
        
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
    ss = reshape(d',1,k*n);
    
    % Pairwise costs: w = exp(-(d/sigma)^2)
    sigma = mean(ss(:));
    ss = 3* exp( - (ss/sigma).^2  );    % When ss==mean(ss), penalty should be 1
    
    pairwise = sparse(ii,jj,ss,n,n);
    % There will typically be many connected components
    % TODO: connect all of them
    %[comps,sizes] = components(pairwise);
    
    %% Label cost
    C = 8;
    labelcost = ones(C,C);
    labelcost(1:C+1:end)=0;
    

    % Haussmann colormap
    colormap = [255 0 0;
                   255  255 0;
                   128  0   255;
                   255  128 0;
                   0    0   255;
                   128  255 255;
                   0    255 0;
                   0    0   255];
    colormap = colormap / 256;
    
    
    %% Unary potentials - classifier
%     load('/esat/nihal/jknopp/data/facades/res_unary/AnCity01-lad.mat');
    load('/esat/nihal/jknopp/data/facades/res_unary/AnCity01_lad+si+n_hf.mat');
    prb = prb([6 8 4 7 2 5 3 1],:);
    unary =  -log(bsxfun(@rdivide,prb,sum(prb)));
    


    %% Unary potentials - detector
    load ('/esat/nihal/jknopp/data/facades/res/AnCity01_ism_votes.mat');
    Q  = 2*Q;
    unaryDet = repmat((1-Q')/7,8,1);
    unaryDet(1,:) = Q';
    unaryDet = -log(unaryDet);
    
    
    alpha = 1;
    unary =  unaryDet;
    
    %% Initial labeling
    [~,segclass] = min(unary);
     newColors_orig = colormap(segclass,:);
    disp('Exporting to ply...');
    ExportPointCloud('models/sub28_honza_unary.ply',points,normals,round(256*newColors_orig));

    %% Graph cuts
    lambda = 1;
    labelcost = lambda * labelcost;
    
    segclass = segclass-1;

    disp('Running the graph cut...');
    [labels,E,Eafter] = GCMex(segclass, double(unary), pairwise, double(labelcost),0);
    disp('Done.');
    labels = labels + 1;

    E_begin = E;
    E_end = Eafter;

    % Mapping colors
    newColors_crf = colormap(labels,:);
    
%     disp(sum(newColors==newColors_crf));
    
    disp('Exporting to ply...');
    ExportPointCloud('models/sub28_honza_rf_ism_crf.ply',points,normals,round(256*newColors_crf));

%     pause(1);
%     system('meshlab /users/visics/amartino/monge3d/models/sub28_honza.ply');
    return;