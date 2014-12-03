function BatchMoveRectifiedData
    
    cmvsPath = '/esat/sadr/amartino/monge428/reconstruction.nvm.cmvs/00/';
    sourcePath = [cmvsPath 'visualize/'];
    
    targetPath = [cmvsPath 'visualizeRect/'];
    if ~exist(targetPath,'dir')
        mkdir(targetPath);
    end
    
    cameras = ImportCameras([cmvsPath 'cameras_v2.txt']);

    
    for i=1:length(cameras)
        fprintf('.');
        
        % Base file name
        visFilename = cameras{i}.visualizeImageName(1:end-4);   % 0000000

        % Load the rectified image and resize it to common width
        imageName = [sourcePath visFilename '.rect.jpg'];
        try
            imRectified = imread(imageName);
        catch err
            warning(['Could not read the rectified image. Reason: ' err.message]);
            continue;
        end
        imResized = imresize(imRectified,[NaN 800]);
%         imResized = imRectified;
        imwrite(imResized,[targetPath visFilename '.jpg'],'jpg','Quality',100);
        
        % Load the original homography
        homography = dlmread([sourcePath visFilename '.rect.dat']);
        
        
        % Recalculate the homography due to scaling
        scaleFactor = size(imResized,1)/size(imRectified,1);

        scaling = zeros(3,3);
        scaling(1,1) = scaleFactor;
        scaling(2,2) = scaleFactor;
        scaling(3,3) = 1;

        % Homography is now from undistorted to rescaled
        homography = reshape(reshape(homography,3,3)*scaling,1,9);
        dlmwrite([targetPath visFilename '.rect.dat'],homography,' ');
         
        % Read the ground truth

         homography = reshape(inv(reshape(homography,3,3)),1,9);
         gtFilename = [sourcePath visFilename '.txt'];
         if exist(gtFilename,'file')
            gt = dlmread(gtFilename);
            gt = imrotate(gt,90);
            gtRectified = rewarp(imResized,gt,homography);
            dlmwrite([targetPath visFilename '.txt'],gtRectified,' ');
         else
             warning(['No GT found for image ' visFilename]);
         end

%         % Project the ground truth directly to rescaled rectified
%         homography = reshape(inv(reshape(homography,3,3)),1,9);
%         gtRectified = rewarp(imResized,gt,homography);
%         
%         dlmwrite([targetPath visFilename '.txt'],gtRectified,' ');

    
      

    end

    disp('Done.');

end
