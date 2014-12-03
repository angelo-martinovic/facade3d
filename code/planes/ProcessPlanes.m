load('facadePlanes.mat');
datasetName = 'monge428';
cmvsLocation = ['/esat/sadr/amartino/' datasetName '/reconstruction.nvm.cmvs/00/'];
[pointsAll,normalsAll,colorsAll]=ReadPointCloudFromPly([cmvsLocation 'models/Paris_RueMonge_part1_ruemonge_107_027_800px.01.ply']);
points = pointsAll(1:100:end,:);
normals = normalsAll(1:100:end,:);
colors = colorsAll(1:100:end,:);

cameras = ImportCameras([cmvsLocation 'cameras_v2.txt']);

%% Calculate 3d positions of cameras
camerapos = zeros(length(cameras),3);

for i=1:length(cameras)
    bla=cameras{i}.P;
    Phat=bla(:,1:3); 
    Fhat=bla(:,4); 
    camerapos(i,:)=Phat\Fhat;
end

camerapos = -camerapos;
    

%% Re-order cameras based on PCA
[pc,score,latent,tsquare] = princomp(camerapos);
cameraProj = score(:,1);
[~,order] = sort(cameraProj);
camerapos2 = camerapos(order,:);

% Sort the facade planes in the same order
facadePlanes.n = facadePlanes.n(order,:);
facadePlanes.p = facadePlanes.p(order,:);
facadePlanes.b = facadePlanes.b(order,:);

%% Filter planes not perpendicular to the gravity vector
nPlanes = size(facadePlanes.p,1);

% gravity vector
% Perform a cross product of every 2 planes
combs  = combnk(1:nPlanes,2);

n1 = facadePlanes.n(combs(:,1),:);
n2 = facadePlanes.n(combs(:,2),:);
gVecs=cross(n1,n2);

% Normalize the resulting vectors
gVecs=bsxfun(@times, gVecs, 1./sqrt(sum(gVecs.^2, 2)));

% Remove the NaN rows
gVecs(isnan(sum(gVecs,2)),:)=[];

% Get the gravity vector
g = median(gVecs,1);

% plane normals * g
cosAngles = facadePlanes.n(:,:)*g';

skip = zeros(nPlanes,1);
for i=1:nPlanes
    if abs(cosAngles(i))>0.1
        skip(i) = 1;
        continue;
    end
end

facadePlanes.n(skip==1,:)=[];
facadePlanes.p(skip==1,:)=[];
facadePlanes.b(skip==1,:)=[];
%VisualizePlanes(facadePlanes, points,normals,colors);

%% Select a subset of best planes
% For each plane, determine the distance of all points to that plane
% plane equation: ax+by+cz+d=0
% [a,b,c] given by facadePlanes.n
% d calculated by
facadePlanes.d=-sum(facadePlanes.n.*facadePlanes.p,2);

% Distance from each point in "points" to the plane
pointPlaneDists = abs(facadePlanes.n*points'+ repmat(facadePlanes.d,1,size(points,1)));

pointInliers = pointPlaneDists<1;

n = size(pointPlaneDists,1);
unityVec = ones(1,size(pointPlaneDists,2));
cvx_begin
    cvx_solver_settings('MSK_DPAR_MIO_MAX_TIME',300)
    variable x(n) binary
    minimize( norm(x'*pointInliers-unityVec) )
cvx_end

fprintf('Selected %d planes.\n',sum(x==1));
selectedPlanes = facadePlanes;
selectedPlanes.n = selectedPlanes.n(x==1,:);
selectedPlanes.p = selectedPlanes.p(x==1,:);
selectedPlanes.b = selectedPlanes.b(x==1,:);
selectedPlanes.d= - sum(selectedPlanes.n.*selectedPlanes.p,2);
save('selectedPlanes.mat','selectedPlanes');
nFilteredPlanes = size(selectedPlanes.n,1);

% Distance from each point in "points" to the selectedPlanes
pointPlaneDists = abs(selectedPlanes.n*points'+ repmat(selectedPlanes.d,1,size(points,1)));

% Separate facade points into groups based on plane assignment
[~,pointPlaneAssignments] = min(pointPlaneDists);
planeColors = round(255*(distinguishable_colors(size(selectedPlanes.n,1))));

colors = planeColors(pointPlaneAssignments,:);
VisualizePlanes(selectedPlanes, points,normals,colors);



%% Separate facade points into groups based on plane assignment and local connectivity
% facadeGroups = struct('points',[]);
lastGroupID = 0;
groupIDs = zeros(size(points,1),1);
pb = ProgressBar(nFilteredPlanes);
for i=1:nFilteredPlanes
    origIndices = find(pointPlaneAssignments==i);
    p = points(origIndices,:);
    
    % Calculate euclidean distances
    D = pdist2(p,p);
    
    D(D<=1)=1;
    D(D>1)=0;
    D = sparse(D);
    
    [nGroups,C] = graphconncomp(D);
    
    % Transform into weights
%     W = exp(-D) / exp(-mean(D(:)));
%     W(1:size(p,1)+1:end) = 0;
%     
%     % Run spectral clustering
%     clustering = SpectralClustering(W, 30, 2);
%     
%     % Interpret results
%     [aa,bb]=find(clustering);
%     [~,order]=sortrows(aa);
%     C = bb(order);
    
    figure(101);scatter3(p(:,1),p(:,2),p(:,3),[],C');
    
%     nGroups = length(unique(C));
    for j=1:nGroups
%         facadeGroups(lastGroupID+j).groupID = 
%         facadeGroups(lastGroupID+j).points = p(C==j,:);
        groupIDs(origIndices(C==j)) = lastGroupID+j;
    end
    lastGroupID = lastGroupID+nGroups;
    pb.progress;
end
pb.stop;
save('groupIDs.mat','groupIDs');

%% Remove groups with low point count
lowCount = 100;
load('groupIDs.mat','groupIDs');
lastGroupID = max(groupIDs);
pointsPerGroup = hist(groupIDs,lastGroupID);

lastGroupID = lastGroupID+1;
nullGroup = lastGroupID;

groupIDsFiltered = groupIDs;
groupIDsFiltered(ismember(groupIDsFiltered,find(pointsPerGroup<lowCount))) = nullGroup;

[~,~,newGroupIDs]=unique(groupIDsFiltered);

planeColors = round(255*(distinguishable_colors(lastGroupID)));
colors = planeColors(newGroupIDs,:);
VisualizePlanes(selectedPlanes, points,normals,colors);

%% Project to the full dataset
% [pointsAll,normalsAll,colorsAll]=ReadPointCloudFromPly([cmvsLocation 'models/Paris_RueMonge_part1_ruemonge_107_027_800px.01.ply']);
idx = knnsearch(points,pointsAll,'K',1);
groupIDAll = newGroupIDs(idx);
colorsAll = planeColors(groupIDAll,:);
VisualizePlanes([], pointsAll,normalsAll,colorsAll);