load('facadePlanes.mat');
datasetName = 'monge428';
cmvsLocation = ['/esat/sadr/amartino/' datasetName '/reconstruction.nvm.cmvs/00/'];
[pointsAll,normalsAll,colorsAll]=ReadPointCloudFromPly([cmvsLocation 'models/Paris_RueMonge_part1_ruemonge_107_027_800px.01.ply']);
points = pointsAll(1:100:end,:);
normals = normalsAll(1:100:end,:);
colors = colorsAll(1:100:end,:);

cameras = ImportCameras([cmvsLocation 'cameras_v2.txt']);

%%
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

cameras = cameras(order);

% Sort the facade planes in the same order
facadePlanes.n = facadePlanes.n(order,:);
facadePlanes.p = facadePlanes.p(order,:);
facadePlanes.b = facadePlanes.b(order,:);

%% Separate points into groups based on local connectivity
% facadeGroups = struct('points',[]);
groupIDs = zeros(size(points,1),1);


% Calculate euclidean distances
D = squareform(pdist(points));

D(D<=1)=1;
D(D>1)=0;
D = sparse(D);

[nGroups,C] = graphconncomp(D);

for j=1:nGroups
    groupIDs(C==j) = j;
end

save('groupIDsAll.mat','groupIDs');
%%
load('groupIDsAll.mat','groupIDs');

nGroups = max(groupIDs);
groupColors = round(255*(distinguishable_colors(nGroups)));
colors = groupColors(groupIDs,:);
% VisualizePlanes([], points,normals,colors);

%% Fit planes to groups
[filteredPlanesCell, filteredPlanes] = SelectPlanes(points,normals,colors,cameras,g,facadePlanes,groupIDs);

%% Find separator lines
% % Order planes by location
% nPlanes = size(filteredPlanes.n,1);
% filteredPlanes.d= - sum(filteredPlanes.n.*filteredPlanes.p,2);
% 
% cc=filteredPlanes.p;
% [pc,score,latent,tsquare] = princomp(cc);
% ccProj = score(:,1);
% [~,order] = sort(ccProj);
% 
% p = points;
% figure(101);scatter3(p(:,1),p(:,2),p(:,3));
% xlabel('x');ylabel('y');zlabel('z');
% 
% % Determine separators
% separatorLines = struct('x0',[],'a',[]);
% for i=1:length(order)-1
%     j = i+1;
%     
%     ii = order(i);
%     jj = order(i+1);
%        
%     n1=filteredPlanes.n(ii,:);
%     n2=filteredPlanes.n(jj,:);
% 
%     p1=filteredPlanes.d(ii,:);
%     p2=filteredPlanes.d(jj,:);
% 
% 
% 
%    [x0,a] = PlaneIntersection(n1,n2,p1,p2);
% 
%    x1 = x0 + a *10;
%    x2 = x0 - a *10;
% 
%    line([x2(1) x1(1)],[x2(2) x1(2)],[x2(3) x1(3)]);
%            
%    separatorLines(i).x0 = x0;
%    separatorLines(i).a = a;
%     
% end

%% For each group with an assigned plane, project 3D points to the plane
for j=1:nGroups
    plane = filteredPlanesCell{j}; 
    if ~isempty(plane)
       pointSubset = points(groupIDs==j,:);
       figure;scatter3(pointSubset(:,1), pointSubset(:,2), pointSubset(:,3));
    end
end
