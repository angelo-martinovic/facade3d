function g = GetGravityVector(facadePlanes)
%     error('Old code!');
   
    nPlanes = size(facadePlanes,2);

    % gravity vector
    % Perform a cross product of every 2 planes' normals
    combs  = combnk(1:nPlanes,2);

    nn = [facadePlanes.n]';
    
    n1 = nn(combs(:,1),:);
    n2 = nn(combs(:,2),:);
    gVecs=cross(n1,n2);

%     norms = sqrt(sum(gVecs.^2, 2));
%     gVecs = gVecs(norms>0.3,:);
    
    % Normalize the resulting vectors
    gVecs=bsxfun(@times, gVecs, 1./sqrt(sum(gVecs.^2, 2)));

    gVecs(abs(gVecs(:,1))>0.1,:) = [];
    gVecs(abs(gVecs(:,3))>0.1,:) = [];
    
    % Remove the NaN rows
    gVecs(isnan(sum(gVecs,2)),:)=[];

%     aa=kmeans(gVecs,2);
    
%     gVecs1 = gVecs(aa==1,:);
%     g1 = median(gVecs1,1);
%     g=g1;
%     gVecs2 = gVecs(aa==2,:);
%     g2 = median(gVecs2,1);
%     
%     g = mean([g1;-g2]);
    
%     figure;scatter3(gVecs(:,1),gVecs(:,2),gVecs(:,3))
%     xlabel('x');ylabel('y');zlabel('z');axis equal;
    
    % Get the gravity vector
    g = median(gVecs,1);
    
    g = g/norm(g);
    if g(2)<0
        g = -g;
    end
end