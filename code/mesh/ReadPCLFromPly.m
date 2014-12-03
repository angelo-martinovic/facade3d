function [points,normals,colors]=ReadPCLFromPly(filename)

    [points,normals,colors,~,~] = read_ply(filename,'PCL');
end