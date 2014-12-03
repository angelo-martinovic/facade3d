function BatchConvertDepthToRectified
    
    datasetPath = '/esat/sadr/amartino/monge3d/reconstruction.nvm.cmvs/00/visualizeRect/';
    cameras = ImportCameras;
   
    load('indices.mat');
    for i=1:length(idx)
        fprintf('.');
        
        basename = cameras{i}.visualizeImageName(1:end-4);
        
        dFilename = [datasetPath basename '_depth.txt'];
        depthMap = dlmread(dFilename);
        
        gtFilename = [datasetPath basename '.txt'];
        gt = dlmread(gtFilename);
        
        homography = dlmread([datasetPath basename 'rect.dat']);
        target = dlmread([datasetPath basename '.seg']);
       
        homography = reshape(inv(reshape(homography,3,3)),1,9);
        depthMapRectified = rewarp(target,depthMap,homography);
        gtRectified = rewarp(target,gt,homography);
        
        movefile(dFilename,[datasetPath 'depthNonRect/' basename '_depth.txt']);
        movefile(gtFilename,[datasetPath 'depthNonRect/' basename '.txt']);
       
        dlmwrite(dFilename,depthMapRectified,' ');
        dlmwrite(gtFilename,gtRectified,' ');
        
    end



end