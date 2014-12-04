% Fixes the 897325 points retarded mesh to normal 897324 pcl
 dirName = '/esat/sadr/amartino/monge428New/data/';
 meshName = 'mesh_colors_normals.ply';
 gtName = 'pcloud_gt_test_new_GCO.ply';
 
[points,normals,colors]=ReadPCLFromPly([dirName meshName]);
[pointsGT,~,~] = ReadPCLFromPly([dirName gtName]);

idx = knnsearch(points,pointsGT);

pointsNew = points(idx,:);
normalsNew = normals(idx,:);
colorsNew = colors(idx,:);

ExportMesh([dirName meshName 'fixed.ply'],pointsNew,normalsNew,colorsNew,[],[]);

