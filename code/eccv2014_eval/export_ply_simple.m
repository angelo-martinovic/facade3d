function export_ply_simple(filename, vertex, face, face_color)
% export_ply_simple(filename, vertex, face, face_color)
%  A simple PLY writer from vertex and face data to a .PLY file
%
% author: hayko riemenschneider, 2014

numFaces =  size(face,2);
numVertices =  size(vertex,2);

% write ply header
fid = fopen(filename,'w');
fprintf(fid,'ply\n');
fprintf(fid,'format ascii 1.0\n');
fprintf(fid,'element vertex %d\n', numVertices);
fprintf(fid,'property float x\n');
fprintf(fid,'property float y\n');
fprintf(fid,'property float z\n');
fprintf(fid,'element face %d\n', numFaces);
fprintf(fid,'property list uchar int vertex_indices\n');
fprintf(fid,'property uchar red\n');
fprintf(fid,'property uchar green\n');
fprintf(fid,'property uchar blue\n');
fprintf(fid,'end_header\n');

% write ply data
fprintf(fid, '%f %f %f \n', vertex);
fprintf(fid, '%d %d %d %d %d %d %d \n', [repmat(3, 1, numFaces); face-1; round(face_color)]);
fclose(fid);
