


for n=1:428
    
    imName = sprintf('%08d',n);
    
    detFilename = ['/esat/sadr/amartino/monge428/reconstruction.nvm.cmvs/00/visualizeRotated/detections/detections_window/' imName '.txt'];
    if exist(detFilename,'file')
        f = fopen(detFilename,'r');
        dets = textscan(f,'%f %f %f %f %f','CollectOutput',true);
        dets = dets{1};
        fclose(f);
     
    else
        continue;
    end
    
    imFilename = ['/esat/sadr/amartino/monge428/reconstruction.nvm.cmvs/00/visualizeRotated/detections/imgs/' imName '.jpg'];
    im=imread(imFilename);
    
    figure(3);imagesc(im);
    for i=1:size(dets,1)
    %     rectangle('Position',[dets(i,2) dets(i,1) dets(i,4)-dets(i,2)+1 dets(i,3)-dets(i,1)+1],'EdgeColor','r');
     rectangle('Position',[dets(i,1) dets(i,2) dets(i,3)-dets(i,1)+1 dets(i,4)-dets(i,2)+1],'EdgeColor','r');
    end
    waitforbuttonpress;
end