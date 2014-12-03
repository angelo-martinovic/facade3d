load('facadePlanes.mat');

% Load plane positions
% Ignore y value
x = facadePlanes.p(:,[1 3]);
figure;scatter(x(:,1),x(:,2));hold on;



x0 = x;
nPoints = size(x0,1);


n = nPoints;
while(n>0.05*nPoints)
    
    % Fit a line
    iterations = 100;
    distThresh = 1;
    inlierRatio = 0.2;
    
    [theta,ro,inliers] = ransac(x',iterations,distThresh,inlierRatio);
    
    minVal = min(x(inliers,1));
    maxVal = max(x(inliers,1));
    
    % Display the line
    line([minVal maxVal],1/cos(theta)*[ro-minVal*sin(theta) ro-maxVal*sin(theta)],'Color','r');
    
    scatter(x(inliers,1),x(inliers,2),'r');
    
    % Remove the inliers and repeat
    x = x(setdiff(1:n,inliers),:);
    
    n = size(x,1);
    waitforbuttonpress;
    
end