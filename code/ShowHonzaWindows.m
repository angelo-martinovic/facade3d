%%
figure;
load /esat/nihal/jknopp/3d_ret_recog_data/DT-SOL/monge428/res2angelo/bbox_result01.mat

nPlanes = length(wins);

for i=1:nPlanes
    patch(wins{i}(1,:),wins{i}(2,:),wins{i}(3,:));
end
grid on; box on; axis equal; xlabel('x');ylabel('y');zlabel('z');



%%
datasetName = 'monge428';
cmvsLocation = ['/esat/sadr/amartino/' datasetName '/reconstruction.nvm.cmvs/00/'];
[pointsAll,normalsAll,colorsAll]=ReadPointCloudFromPly([cmvsLocation 'models/Paris_RueMonge_part1_ruemonge_107_027_800px.01.ply']);
points = pointsAll(1:100:end,:);
normals = normalsAll(1:100:end,:);
colors = colorsAll(1:100:end,:);

% points = [];
% normals = [];
% colors=[];
faces = [];

pb = ProgressBar(nPlanes);
for i=1:nPlanes

    faceX = wins{i}(1,:)';
    faceY = wins{i}(2,:)';
    faceZ = wins{i}(3,:)';

    % Add the points
    points = [points; [faceX faceY faceZ]];
    normals = [normals; ones(4,3)];
    colors = [colors; ones(4,3)];

    % Face connecting the last 4 points
    faces = [faces; 4 length(points)-1 length(points)-2 length(points)-3 length(points)-4];
    pb.progress;
end
pb.stop;

%%
    %     VisualizePointCloud2(points,normals,colors,1);
VisualizePointCloud(points,normals,colors,faces);
