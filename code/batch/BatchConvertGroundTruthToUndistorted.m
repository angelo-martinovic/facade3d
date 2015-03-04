function BatchConvertGroundTruthToUndistorted
    
%     cmvsReconstructionPath = '/esat/sadr/amartino/monge3dRight/reconstruction.nvm.cmvs/00/';
%     gtPath = '/esat/sadr/amartino/Facades/Monge3D/subset30_800px_labels/labels_angelo/';

    cmvsReconstructionPath = '/esat/sadr/amartino/monge428/reconstruction.nvm.cmvs/00/';
    gtPath = '/usr/data/amartino/Facades/Monge3D/full428_splithalf/';
    cameras = ImportCameras([cmvsReconstructionPath 'cameras_v2.txt']);

    
    for i=1:length(cameras)
        fprintf('.');
        
        basename = cameras{i}.originalImageFilename(1:end-4); 
        visFilename = cameras{i}.visualizeImageName(1:end-4);
        
        gtFilename = [gtPath basename '.txt'];
        if exist(gtFilename,'file')
            gt = dlmread(gtFilename);

            newImage = imread([cmvsReconstructionPath 'visualize/' visFilename '.jpg']);

            newImage = imrotate(newImage,-90);
            h = size(newImage,1);
            w = size(newImage,2);

%             gtUndistorted = RadialUndistort(imresize(gt,[h,w],'nearest'),cameras{i});
            gtUndistorted = gt;
            
            newFilename = [cmvsReconstructionPath 'visualize/' visFilename '.txt'];
            dlmwrite(newFilename,gtUndistorted,' ');
        else
            warning(['No GT found for image ' basename]);
        end
        
    end

    disp('Done.');

end



