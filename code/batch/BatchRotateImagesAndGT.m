function BatchRotateImagesAndGT
    angle = -90;
    
    cmvsPath = '/esat/sadr/amartino/monge428/reconstruction.nvm.cmvs/00/';
    sourcePath = [cmvsPath 'visualizeRect/'];
    
    targetPath = [cmvsPath 'visualizeRotated/'];
    mkdirIfNotExist(targetPath);
    
    cameras = ImportCameras([cmvsPath 'cameras_v2.txt']);

    
    for i=1:length(cameras)
        fprintf('.');
        
        % Base file name
        visFilename = cameras{i}.visualizeImageName(1:end-4);   % 0000000

        % Load the rectified image and rotate by 90
        imageName = [sourcePath visFilename '.jpg'];
        try
            imOriginal = imread(imageName);
        catch err
            warning(['Could not read the image. Reason: ' err.message]);
            continue;
        end
        imRotated = imrotate(imOriginal,angle);
        
        imwrite(imRotated,[targetPath visFilename '.jpg'],'jpg','Quality',100);
        
     
        % Read the ground truth       
         gtFilename = [sourcePath visFilename '.txt'];
         if exist(gtFilename,'file')
            groundTruth = dlmread(gtFilename);
            groundTruth = imrotate(groundTruth,angle);
            dlmwrite([targetPath visFilename '.txt'],groundTruth,' ');
         else
             warning(['No GT found for image ' visFilename]);
         end

    end

    disp('Done.');

end
