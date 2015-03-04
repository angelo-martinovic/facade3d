function [result,E_begin,E_end] = RunCRF(obj,unarySegmentationPotentials,unaryDetectionPotentials)   
    crf = obj.config.c2D.crf;
    nClasses = obj.config.nClasses;
    
    %% Image size
    H = size(unarySegmentationPotentials,1);
    W = size(unarySegmentationPotentials,2);
       
    %% Initial labels
    [~,oldImg] = max(unarySegmentationPotentials,[],3);
    oldImg = oldImg-1;
    segclass = reshape(oldImg',W*H,1);

    %% Pairwise term
    % 4 - neighborhood: 1-ball with L1 distance
    [ii, jj] = sparse_adj_matrix([W H], 1, 1);
    pairwise = sparse(ii,jj,ones(1,numel(ii)), W * H, W * H);
    pairwise(1:size(pairwise,1)+1:end)=0;
                                          
    %% Unary term
    unary = zeros(nClasses,W*H);
    nDetectors = length(unaryDetectionPotentials);
    unary_mat = crf.weightUnarySegmentation * (- log(unarySegmentationPotentials) );
        
    for d=1:nDetectors
        unary_mat = unary_mat + crf.weightsUnaryDetectors(d) * (-log(unaryDetectionPotentials{d}));
    end
    
    for c=1:nClasses
        unary(c,:) = reshape(unary_mat(:,:,c)',1,W*H);
    end
           
    %% Label term
    labelcost = crf.labelCost;

    % Final expression
    pairwise = crf.weightPairwise*pairwise;

	%% Calling the MEX file
    [labels,E,Eafter] = GCMex(segclass, double(unary), pairwise, double(labelcost),0);
    labels = labels + 1;

    E_begin = E;
    E_end = Eafter;
    
    result = reshape(labels,W,H)';
end