function [points,normals,colors] = ReadPCLFromPly(filename)
%     dl = DispatchingLogger.getInstance();
%     dl.Log(VerbosityLevel.Debug,sprintf(' - Reading the point cloud ...\n'));
    [points,normals,colors,~,~] = read_ply(filename,'PCL');
end