function VisualizePlanes(facadePlanes,points,normals,colors)
    faces = [];
    if nargin<2
        points = [];
        normals = [];
        colors=[];
    end
    %     
    if ~isempty(facadePlanes)
   
        nPlanes = size(facadePlanes.p,1);
        pb = ProgressBar(nPlanes);
        for i=1:nPlanes


            A = facadePlanes.n(i,1);
            B = facadePlanes.n(i,2);
            C = facadePlanes.n(i,3);
            x1 = facadePlanes.p(i,1);
            y1 = facadePlanes.p(i,2);
            z1 = facadePlanes.p(i,3);

            extend = 20;
            xMin = facadePlanes.b(i,1)-extend;
            xMax = facadePlanes.b(i,2)+extend;
            yMin = facadePlanes.b(i,3)-extend;
            yMax = facadePlanes.b(i,4)+extend;
            zMin = z1-extend;%facadePlanes.b(i,5);
            zMax = z1+extend;%facadePlanes.b(i,6);

            faceX = [xMin xMax xMax xMin]';
            faceY = [yMax yMax yMin yMin]';

            faceZ = z1 - A/C*(faceX-x1) - B/C*(faceY-y1);

            while any(faceZ<zMin | faceZ>zMax)
                faceX = 0.8*faceX + 0.2*x1;
                faceY = 0.8*faceY + 0.2*y1;

                faceZ = z1 - A/C*(faceX-x1) - B/C*(faceY-y1);
            end

            % Add the points
            points = [points; [faceX faceY faceZ]];
            normals = [normals; ones(4,3)];
            colors = [colors; ones(4,3)];

            % Face connecting the last 4 points
            faces = [faces; 4 length(points)-1 length(points)-2 length(points)-3 length(points)-4];
            pb.progress;
        end
        pb.stop;
    else
        faces=[];
    end

    %     VisualizePointCloud2(points,normals,colors,1);
    VisualizePointCloud(points,normals,colors,faces);
end