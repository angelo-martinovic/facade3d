%% Import NVM image mappings
filename = '/usr/data/amartino/monge428/reconstruction.nvm.cmvs/00/Paris_RueMonge_part1_ruemonge_107_027_800px.nvm';
delimiter = '\t';
startRow = 3;
endRow = 430;
formatSpec = '%s%*s%[^\n\r]';
fileID = fopen(filename,'r');
textscan(fileID, '%[^\n\r]', startRow-1, 'ReturnOnError', false);
dataArray = textscan(fileID, formatSpec, endRow-startRow+1, 'Delimiter', delimiter, 'ReturnOnError', false);
fclose(fileID);
NVM_V3 = [dataArray{1:end-1}];
clearvars filename delimiter startRow endRow formatSpec fileID dataArray ans;


loc = '/usr/data/amartino/monge428/reconstruction.nvm.cmvs/00/visualizeRotated/output/svm_monge428_newTrain/';
for i=1:428
    fprintf('.');
    if mod(i,10)==0
        fprintf('%d\n',i);
    end
    basename = sprintf('%08d',i-1);
    
    filename = [loc basename '.layer1.mat'];
    if exist(filename,'file')
        load(filename); % segMap
        load([loc basename '.label_layer2_withDet.mat']); % outImg detectionMaps(1).detectionMap

        svmPotentials = segMap;
        detectorPotentials = detectionMaps(1).detectionMap;
        labeling = outImg;

        outName = NVM_V3{i}(end-11:end-4);
        save([loc outName '.2Dpotentials.mat'],'svmPotentials','detectorPotentials','labeling');
    else
        fprintf('\bx');
    end
    
end