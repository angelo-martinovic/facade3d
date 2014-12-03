% Load plane positions and normals
load('facadePlanes.mat');

% Normalize to 0-1
x = [facadePlanes.p facadePlanes.n];
% for i=1:6
%    x(:,i) = x(:,i)-min(x(:,i));
%    x(:,i) = x(:,i)/max(x(:,i));
% end
% Cluster them in 6D space
bandwidth = 10; % TODO: determine this from train data
[clustCent,data2cluster,cluster2dataCell] = MeanShiftCluster(x',bandwidth);

% Display clustered planes
x = facadePlanes.p;
figure;scatter3(x(:,1),x(:,2),x(:,3),[],data2cluster');

x = facadePlanes.n;
figure;scatter3(x(:,1),x(:,2),x(:,3),[],data2cluster');