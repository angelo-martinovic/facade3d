%FITPLANE Fits a plane to a set of 3D points.
%   FITPLANE(points) returns a structure defining the plane, with the 
%   following fields:
%     plane.p : A 3D point that lies in the plane.
%     plane.n : Normal vector
%     plane.n : Bounding box [minX, maxX, minY, maxY, minZ, maxZ]
function plane = FitPlane(points)

    [n,~,p] = affine_fit(points);
    
    p = p';
    b=[min(points(:,1)),max(points(:,1)),min(points(:,2)),max(points(:,2)),min(points(:,3)),max(points(:,3))];
    
    plane = struct('p',p,'n',n,'b',b);
end