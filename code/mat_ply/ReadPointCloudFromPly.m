%
function [points,normals,colors] = ReadPointCloudFromPly(filename)
    
    if nargin<1
        filename = '/esat/sadr/amartino/monge3d/reconstruction.nvm.cmvs/00/models/option-0000.ply';
    end
    
    [x,y,z,nx,ny,nz,r,g,b]=ImportPointCloud(filename);
    
    
    points = [x y z];
    normals = [nx ny nz];
    colors = [r g b];
    

end