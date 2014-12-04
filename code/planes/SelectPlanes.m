function [filteredPlanesCell, filteredPlanes] = SelectPlanes(points,normals,colors,cameras,g,facadePlanes,groupIDs)

nGroups = max(groupIDs);

%% For each group
filteredPlanesCell = cell(1,nGroups);
filteredPlanes = struct('n',[],'p',[],'b',[],'d',[]);
minSize = 100;
for i=1:nGroups
   fprintf('Subset %d ...',i);
   
   pointSubset = points(groupIDs==i,:);
   normalSubset = normals(groupIDs==i,:);
   
   % Ignore small subsets
   if size(pointSubset,1)<minSize
       fprintf('Skipping.\n');
       continue;
   end
   fprintf('\n');
   
   % Find cameras observing the subset
   cameraIDs = CamerasObservingPoints(cameras,pointSubset);
   
   % Get the facade planes extracted from the cameras
   fPlanes.n = facadePlanes.n(cameraIDs,:);
   fPlanes.p = facadePlanes.p(cameraIDs,:);
   fPlanes.b = facadePlanes.b(cameraIDs,:);
   
   % Select a subset of best planes
   fprintf('Initial number of planes/cameras: %d\n',length(cameraIDs));
   
   % 1. The average of all point normals
   n_fPlane_avg = mean(fPlanes.n,1)';

   [p_fPlane,n_fPlane,b_fPlane] = FitPlane(pointSubset);
   
   % Invert the RANSAC plane normal if it has the wrong direction
    if dot(n_fPlane,n_fPlane_avg)<0
        n_fPlane = - n_fPlane;
    end

    filteredPlanesCell{i} = struct('n',n_fPlane,'p',p_fPlane,'b',b_fPlane,'d',-dot(n_fPlane,p_fPlane));
    filteredPlanes.p = [filteredPlanes.p ; p_fPlane'];
    filteredPlanes.n = [filteredPlanes.n ; n_fPlane'];
    filteredPlanes.b = [filteredPlanes.b ; b_fPlane];
end
          
planeColors = round(255*(distinguishable_colors(nGroups)));
colors = planeColors(groupIDs,:);
% VisualizePlanes(filteredPlanes, points,normals,colors);
   
%     % For each plane, determine the distance of all points to that plane
%     % plane equation: ax+by+cz+d=0
%     % [a,b,c] given by facadePlanes.n
%     % d calculated by
%     fPlanes.d=-sum(fPlanes.n.*fPlanes.p,2);
% 
%     % Distance from each point in "points" to the plane
%     pointPlaneDists = abs(fPlanes.n*pointSubset'+ repmat(fPlanes.d,1,size(pointSubset,1)));
% 
%     pointInliers = pointPlaneDists<0.1;
% % 
% %     noSupport = sum(pointInliers,2);
% %     noSupport = 1-(noSupport<500);
%     
%     n = size(pointPlaneDists,1);
%     unityVec = ones(1,size(pointPlaneDists,2));
% %     cvx_begin
% %         cvx_solver_settings('MSK_DPAR_MIO_MAX_TIME',30)
% %         variable x(n) binary
% %         minimize( )
% %     cvx_end
% 
%     cvx_begin
%         cvx_solver_settings('MSK_DPAR_MIO_MAX_TIME',30)
%         variable x(n) binary
%         minimize norm(x'*pointInliers-unityVec) 
%     cvx_end
%     
% 
%     fprintf('Selected %d planes.\n',sum(x==1));
%     selectedPlanes = fPlanes;
%     selectedPlanes.n = selectedPlanes.n(x==1,:);
%     selectedPlanes.p = selectedPlanes.p(x==1,:);
%     selectedPlanes.b = selectedPlanes.b(x==1,:);
%     selectedPlanes.d= - sum(selectedPlanes.n.*selectedPlanes.p,2);
% %     save('selectedPlanes.mat','selectedPlanes');
%     nSelectedPlanes = size(selectedPlanes.n,1);
%     fprintf('Final number of planes: %d\n',nSelectedPlanes);
    
   
%     if nSelectedPlanes>0
% 
%         % Distance from each point in "points" to the selectedPlanes
%         pointPlaneDists = abs(selectedPlanes.n*pointSubset'+ repmat(selectedPlanes.d,1,size(pointSubset,1)));
% 
%         % Separate facade points into groups based on plane assignment
%         [~,pointPlaneAssignments] = min(pointPlaneDists,[],1);
% 
% 
%         % Re-estimate planes based on assigned points
%         % Fit a facade plane
%         for j=1:nSelectedPlanes
%             normalSubsetPlane = normalSubset(pointPlaneAssignments==j,:);
%             pointSubsetPlane = pointSubset(pointPlaneAssignments==j,:);
% 
%             % 1. The average of all point normals
%             n_fPlane_avg = mean(normalSubsetPlane,1)';
%             p_fPlane_avg = median(pointSubsetPlane,1)';
% 
%             % 2. Fit a plane with RANSAC
%             [p_fPlane,n_fPlane,b_fPlane] = FitPlane(pointSubsetPlane);
% 
%             % Invert the RANSAC plane normal if it has the wrong direction
%             if dot(n_fPlane,n_fPlane_avg)<0
%                 n_fPlane = - n_fPlane;
%             end
% 
%             selectedPlanes.p(j,:) = p_fPlane;
%             selectedPlanes.n(j,:) = n_fPlane;
%             selectedPlanes.b(j,:) = b_fPlane;
%         end
% 
%         filteredPlanes.n = [filteredPlanes.n; selectedPlanes.n];
%         filteredPlanes.p = [filteredPlanes.p; selectedPlanes.p];
%         filteredPlanes.b = [filteredPlanes.b; selectedPlanes.b];
%         filteredPlanes.d = [filteredPlanes.d; selectedPlanes.d];
%         
%         % Distance from each point in "points" to the selectedPlanes
%         pointPlaneDists = abs(selectedPlanes.n*pointSubset'+ repmat(selectedPlanes.d,1,size(pointSubset,1)));
% 
%         % Separate facade points into groups based on plane assignment
%         [~,allAssignments{i}] = min(pointPlaneDists,[],1);
% 
% %         planeColors = round(255*(distinguishable_colors(size(filteredPlanes.n,1))));
% % 
% %         colors = planeColors(pointPlaneAssignments,:);
%     
%     end
%     
% 
% 
% 
% end
% 
% nonemptyGroups = find(cellfun(@(x) size(unique(x),1),allAssignments));
% emptyGroups = find(cellfun(@(x) size(unique(x),1),allAssignments)==0);
% nPlanesPerGroup = cellfun(@(x)size(unique(x),2),allAssignments(nonemptyGroups));
% planeBorderIndices = [0 cumsum(nPlanesPerGroup)];
% 
% pointPlaneDists = abs(filteredPlanes.n*points'+ repmat(filteredPlanes.d,1,size(points,1)));
% 
% for i=1:length(nonemptyGroups)
%     groupID = nonemptyGroups(i);
%    % Allow assignment of planes only from the subset
%    lowBorder = planeBorderIndices(i)+1;
%    highBorder = planeBorderIndices(i+1);
%    pointPlaneDists(1:lowBorder-1,groupIDs==groupID) = Inf(size(pointPlaneDists(1:lowBorder-1,groupIDs==groupID)));
%    pointPlaneDists(highBorder+1:end,groupIDs==groupID) = Inf(size(pointPlaneDists(highBorder+1:end,groupIDs==groupID)));
% end
% 
% pointsB = points;
% normalsB = normals;
% 
% % for i=1:length(emptyGroups)
% %     groupID = emptyGroups(i);
%    % Allow assignment of planes only from the subset
%    pointPlaneDists(:,ismember(groupIDs,emptyGroups)) = [];
%    pointsB(ismember(groupIDs,emptyGroups),:) = [];
%    normalsB(ismember(groupIDs,emptyGroups),:) = [];
% 
% % end
% 
% [~,pointPlaneAssignments] = min(pointPlaneDists,[],1);
% planeColors = round(255*(distinguishable_colors(size(filteredPlanes.n,1))));
% colors = planeColors(pointPlaneAssignments,:);
% VisualizePlanes(filteredPlanes, pointsB,normalsB,colors);

% for i=1:length(emptyGroups)
%     groupID = emptyGroups(i);
%    % Allow assignment of planes only from the subset
%    pointPlaneDists(:,groupIDs==groupID) = Inf(size(pointPlaneDists(:,groupIDs==groupID) ));
% end
% 
% [~,pointPlaneAssignments] = min(pointPlaneDists,[],1);








