function pairwise = mesh_adjacency_onehit(faces, vertex)
% pairwise = mesh_adjacency_onehit(faces, vertex)
%  Calculate pairwise neighborhood as face adjacency by sharing faces
%
% note: fastest because of direct index lists and one hit sparse construction
% author: hayko riemenschneider, 2014

if (nargin==1)
    tri = faces;
else
    warning off
    tri = TriRep(faces', vertex');
    warning on
end

tetra_cnt = size(tri.Triangulation,1);
dim_cnt = size(tri.Triangulation,2);

neigh = neighbors(tri);
faceidx=repmat(1:tetra_cnt,dim_cnt,1)';
neigh_list = reshape(neigh,tetra_cnt*dim_cnt,1);
faceidx_list = reshape(faceidx,tetra_cnt*dim_cnt,1);
faceidx_list(find(isnan(neigh_list)))=[];
neigh_list(find(isnan(neigh_list)))=[];
pairwise = sparse([faceidx_list ],[neigh_list ],1,tetra_cnt,tetra_cnt);

