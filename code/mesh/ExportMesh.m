function ExportMesh(filename,points,normals,colors,faces,faceColors)

    if nargin~=6
        error('Usage: ExportMesh(filename,points,normals,colors,faces,faceColors)');
    end

    points = double(points);
    normals = double(normals);
    colors = double(colors);
    faces = double(faces);
    faceColors = double(faceColors);

    nPoints = size(points,1);
    nFaces = size(faces,1);
    
    header = '';
    header = sprintf('%sply\n',header);
    header = sprintf('%sformat ascii 1.0\n',header);
    header = sprintf('%selement vertex %d\n',header,nPoints);
    
    if isempty(points)
        error('No point locations specified!');
    end
    header = sprintf('%sproperty float x\n',header);
    header = sprintf('%sproperty float y\n',header);
    header = sprintf('%sproperty float z\n',header);
    
    if ~isempty(normals)
        header = sprintf('%sproperty float nx\n',header);
        header = sprintf('%sproperty float ny\n',header);
        header = sprintf('%sproperty float nz\n',header);
    end
    
    if ~isempty(colors)
        header = sprintf('%sproperty uchar diffuse_red\n',header);
        header = sprintf('%sproperty uchar diffuse_green\n',header);
        header = sprintf('%sproperty uchar diffuse_blue\n',header);
    end
    
    if isempty(faces)
%         warning('No faces specified!');
    else
        header = sprintf('%selement face %d\n',header,nFaces);
        header = sprintf('%sproperty list uchar int vertex_indices\n',header);

        if ~isempty(faceColors)
            header = sprintf('%sproperty uchar red\n',header);
            header = sprintf('%sproperty uchar green\n',header);
            header = sprintf('%sproperty uchar blue\n',header);
        end
    end
    header = sprintf('%send_header\n',header);

  
    f=fopen(filename,'w');
    fprintf(f,'%s',header);


    if ~isempty(normals)
        if ~isempty(colors)
             fprintf(f,'%f %f %f %f %f %f %d %d %d\n',[points normals colors]');
        else
             fprintf(f,'%f %f %f %f %f %f\n',[points normals]');
        end
    else
        if ~isempty(colors)
             fprintf(f,'%f %f %f %d %d %d\n',[points colors]');
        else
             fprintf(f,'%f %f %f\n',points');
        end 
    end
   
    if ~isempty(faces)
        faces = [3*ones(nFaces,1) faces-1];
        if ~isempty(faceColors)
            fprintf(f,'%d %d %d %d %d %d %d\n',[faces faceColors]');
        else
            fprintf(f,'%d %d %d %d\n',faces');
        end
    end
    fclose(f);
   
end