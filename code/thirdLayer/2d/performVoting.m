function [vs, wss, hss] = performVoting(vs, bbs, old_bbs, featureVectors, featureCoords, draw, dw, dh)

nn =10;
ks = round(max(dw,dh));
if mod(ks,2) ==0
    ks = ks +1;
end
voteKernel = fspecial('gaussian',ks,4);
%ws2 = cell(size(vs));
%hs2 = cell(size(vs));
ws = containers.Map('KeyType','int32','ValueType','any');
hs = containers.Map('KeyType','int32','ValueType','any');
for i=1:size(bbs,1)
    %split features
    bb = bbs(i,:);
    old_bb = old_bbs(i,:);
    height = old_bb(4)- old_bb(2);
    width = (old_bb(3)-old_bb(1));
    %this seems to be correct
    bbCenter = [old_bb(1)+(old_bb(3)-old_bb(1))/2 old_bb(2)+(old_bb(4)- old_bb(2))/2];
    detectionFeatMask = featureCoords(:,1) > bb(1) & featureCoords(:,1) < bb(3)  & featureCoords(:,2) > bb(2) & featureCoords(:,2) < bb(4);

    queryPointFeatureVectors = featureVectors(detectionFeatMask,:);
    queryPointsFeaturePositions = featureCoords(detectionFeatMask,:);

    votingVectors = -queryPointsFeaturePositions+repmat(bbCenter,size(queryPointsFeaturePositions,1),1);

    
    %NN search
    [neighbors,distances] = knnsearch(featureVectors, queryPointFeatureVectors, 'k', nn,  'Distance', 'euclidean');
    %neighbors =  neighbors(:,2:end);
    %distances =  distances(:,2:end);
    for ptidx=1:size(neighbors,1)
       
        %get the neighbors and voting Points for point ptidx
            votePoints = featureCoords(neighbors(ptidx,:),:) + repmat(votingVectors(ptidx,:), size(neighbors,2),1);
            vs = castVotes(vs, votePoints, voteKernel);
            votePoints = round(votePoints);
            votePoints = votePoints((votePoints(:,1) <= size(vs,2)-dw),:);
            votePoints = votePoints((votePoints(:,2) <= size(vs,1)-dh),:);
            votePoints = votePoints((votePoints(:,1) >=dw),:);
            votePoints = votePoints((votePoints(:,2) >=dh),:);
            
            for votePointIndex=1:size(votePoints,1)
                    %ws2{votePoints(votePointIndex,2),votePoints(votePointIndex,1)}(end+1)=width;
                    %hs2{votePoints(votePointIndex,2),votePoints(votePointIndex,1)}(end+1)=height;
                    key_val = sub2ind(size(vs),votePoints(votePointIndex,2),votePoints(votePointIndex,1));
                    if ws.isKey(key_val)
                        ws(key_val) = [ws(key_val) width];
                        hs(key_val) = [hs(key_val) height];
                    else
                        ws(key_val) = width;
                        hs(key_val) = height;
                    end
                   
            end

       
    end

    if false
        figure(667); imagesc(vs);axis image;
    end


end
%wss2=cellfun(@median,ws2);
%hss2= cellfun(@median,hs2);

wss_t=cellfun(@median,ws.values);
hss_t= cellfun(@median,hs.values);
wss = nan(size(vs));
hss = nan(size(vs));

wss(cell2mat(ws.keys))=wss_t;
hss(cell2mat(hs.keys))=hss_t;



end









