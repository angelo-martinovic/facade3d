function [result,E_begin,E_end] = RunCRF(segMap,detectionMaps,nClasses,crfParams)   
    %% Image size
    H = size(segMap,1);
    W = size(segMap,2);
       
    %% Initial labels
    [~,oldImg] = max(segMap,[],3);
    oldImg = oldImg-1;
    segclass = reshape(oldImg',W*H,1);

    %% Pairwise term
    % 4 - neighborhood: 1-ball with L1 distance
    [ii, jj] = sparse_adj_matrix([W H], 1, 1);
    pairwise = sparse(ii,jj,ones(1,numel(ii)), W * H, W * H);
    pairwise(1:size(pairwise,1)+1:end)=0;
                                          
    %% Unary term
    unary = zeros(nClasses,W*H);
    nDetectors = length(detectionMaps);
    unary_mat = crfParams.weightSegmentationUnary * (- log(segMap) );
        
    for i=1:nDetectors
        unary_mat = unary_mat-detectionMaps(i).crfWeight*log(detectionMaps(i).detectionMap);
    end
    
    for i=1:nClasses
        unary(i,:) = reshape(unary_mat(:,:,i)',1,W*H);
    end
           
    %% Label term
    labelcost = crfParams.labelCost;

    % Final expression
    pairwise = crfParams.weightPairwise*pairwise;

	%% Calling the MEX file
    [labels,E,Eafter] = GCMex(segclass, double(unary), pairwise, double(labelcost),0);
    labels = labels + 1;

    E_begin = E;
    E_end = Eafter;
    
    result = reshape(labels,W,H)';
end