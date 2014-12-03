function VisualizePointCloud(points,normals,colors,faces)

    if nargin<4
        faces = [];
    end
    
    if max(colors(:))<1
        colors = round(256*colors);
    end
    ExportMesh('tmp/tmp.ply',points,normals,colors,faces);

%     ply_write ( Data, 'tmp/tmp.ply' );
        
    system('cd tmp; meshlab tmp.ply');

    
end