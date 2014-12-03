function [points,normals,colors,faces,faceColors]=ReadMeshFromPly(filename)

    [points,normals,colors,faces,faceColors] = read_ply(filename,'mesh');
end
