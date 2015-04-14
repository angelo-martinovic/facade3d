dirName = datasetConfig.dataLocation;%'/esat/sadr/amartino/Facade3D/dataFullRes/';

loc = get_adr('pclModelDir', datasetConfig);

gt = get_adr('pcl_gt_test',datasetConfig);
baseName = datasetConfig.name;
% highRes = false;
% if highRes
%     % High-res mesh
%     baseName = 'monge428_fullRes';
%     gt = '/esat/sadr/amartino/monge428New/data/fullRes_pcloud_test_new_GCO.ply';
% else
%     % Low-res mesh
%     baseName = 'monge428';
%     gt = '/esat/sadr/amartino/monge428New/data/pcloud_gt_test_new_GCO.ply';
% end


%% 
fprintf('Images layer 1\n------------\n');
outputFolder = [dirName 'work/probmap_l1/'];
score = EvaluateImageLabeling(dirName,outputFolder)

%% 
fprintf('Images layer 2\n------------\n');
outputFolder = [dirName 'work/probmap_l2/'];
score = EvaluateImageLabeling(dirName,outputFolder)

%% 
fprintf('3D + MAP projected\n------------\n');
outputFolder = [dirName 'work/output-' baseName '_3D.ply/'];
score = EvaluateImageLabeling(dirName,outputFolder)

%% 
fprintf('3D + 3D CRF projected\n------------\n');
outputFolder = [dirName 'work/output-' baseName '_3D_3DCRF.ply/'];
score = EvaluateImageLabeling(dirName,outputFolder)

%% 
fprintf('Layer1 + 3D + MAP projected\n------------\n');
outputFolder = [dirName 'work/output-' baseName '_layer1+3D.ply/'];
score = EvaluateImageLabeling(dirName,outputFolder)

%% 
fprintf('Layer1 + 3D + 3D CRF projected\n------------\n');
outputFolder = [dirName 'work/output-' baseName '_layer1+3D_3DCRF.ply/'];
score = EvaluateImageLabeling(dirName,outputFolder)

%% 
fprintf('Layer2 + 3D + MAP projected\n------------\n');
outputFolder = [dirName 'work/output-' baseName '_layer1+layer2+3D.ply/'];
score = EvaluateImageLabeling(dirName,outputFolder)

%% 
fprintf('Layer2 + 3D + 3D CRF projected\n------------\n');
outputFolder = [dirName 'work/output-' baseName '_layer1+layer2+3D_3DCRF.ply/'];
score = EvaluateImageLabeling(dirName,outputFolder)

%%
%%
%%
fprintf('3D feat + MAP\n------------\n');
l = [loc baseName '_3D.ply'];
score = EvaluateMeshLabeling(l,gt)

%%
fprintf('3D feat + 3D CRF\n------------\n');
l = [loc baseName '_3D_3DCRF.ply'];
score = EvaluateMeshLabeling(l,gt)

%%
fprintf('Layer 1 + majority vote\n------------\n');
l = [loc baseName '_2D_layer1_majorityVote.ply'];
score = EvaluateMeshLabeling(l,gt)

%%
fprintf('Layer 1 + MAP\n------------\n');
l = [loc baseName '_2D_3DMAP.ply'];
score = EvaluateMeshLabeling(l,gt)

%%
fprintf('Layer 1 + 3D CRF\n------------\n');
l = [loc baseName '_2D_3DCRF.ply'];
score = EvaluateMeshLabeling(l,gt)

%%
fprintf('Layer 2 (Layer 1 + Detectors + 2D CRF) + majority vote\n------------\n');
l = [loc baseName '_2D_layer2_majorityVote.ply'];
score = EvaluateMeshLabeling(l,gt)

%%
fprintf('Layer 1 + Detectors + MAP\n------------\n');
l = [loc baseName '_layer1+layer2.ply'];
score = EvaluateMeshLabeling(l,gt)

%%
fprintf('Layer 1 + Detectors + 3D CRF\n------------\n');
l = [loc baseName '_layer1+layer2_3DCRF.ply'];
score = EvaluateMeshLabeling(l,gt)

%%
fprintf('Layer 1 + 3D + MAP\n------------\n');
l = [loc baseName '_layer1+3D.ply'];
score = EvaluateMeshLabeling(l,gt)
% score.accuracy_classwise

%%
fprintf('Layer 1 + 3D + 3D CRF\n------------\n');
l = [loc baseName '_layer1+3D_3DCRF.ply'];
score = EvaluateMeshLabeling(l,gt)
% score.accuracy_classwise

%%
fprintf('Layer 1 + Detectors + 3D + MAP\n------------\n');
l = [loc baseName '_layer1+layer2+3D.ply'];
score = EvaluateMeshLabeling(l,gt)
% score.accuracy_classwise

%%
fprintf('Layer 1 + Detectors + 3D + 3D CRF\n------------\n');
l = [loc baseName '_layer1+layer2+3D_3DCRF.ply'];
score = EvaluateMeshLabeling(l,gt)
% score.accuracy_classwise

%%
%% 2D 3rd layer
%%

fprintf('2D 3rd layer\n------------\n');
l = [loc baseName '_3D_3DCRF_L3_Ortho2D.ply'];
score = EvaluateMeshLabeling(l,gt)

%%
fprintf('2D 3rd layer\n------------\n');
l = [loc baseName '_layer1_3DCRF_2Dlayer3.ply'];
score = EvaluateMeshLabeling(l,gt)

%%
fprintf('2D 3rd layer\n------------\n');
l = [loc baseName '_layer1+layer2_3DCRF_2Dlayer3.ply'];
score = EvaluateMeshLabeling(l,gt)


%%
fprintf('2D 3rd layer\n------------\n');
l = [loc baseName '_layer1+3D_3DCRF_2Dlayer3.ply'];
score = EvaluateMeshLabeling(l,gt)

%%
%% 3D 3rd layer
%%

%%
fprintf('3D 3rd layer on top of 3D\n------------\n');
l = [loc baseName '_3D_3DCRF_L3_Pure3D.ply'];
score = EvaluateMeshLabeling(l,gt)

%%
fprintf('3D 3rd layer on top of 2D\n------------\n');
l = [loc baseName '_2D_3DCRF_L3_Pure3D.ply'];
score = EvaluateMeshLabeling(l,gt)

%%
fprintf('3D 3rd layer on top of 3D+2D\n------------\n');
l = [loc baseName '_3D+2D_3DCRF_L3_Pure3D.ply'];
score = EvaluateMeshLabeling(l,gt)

%%
%old
fprintf('3D 3rd layer on top of 3D-old\n------------\n');
l = ['/esat/nihal/jknopp/3d_ret_recog_data/DT-SOL/monge428_27/2andelo/full_pcl_labeling_3D_3DCRF_3D3rdLayer.ply'];
score = EvaluateMeshLabeling(l,gt)


%%

datasets = {'monge482_fullres'}; % 'monge428_27',
% datasets = {'monge428_27'}; 
types={'layer1+3D_3DCRF'}; % 
% types={'3D_3DCRF','layer1_3DCRF','layer1+3D_3DCRF'}; % 
% types={'3D_3DCRF','layer1_3DCRF'}; % 
% types={'3D_3DCRF'}; % 'layer1_3DCRF',
for i = 1:length(datasets)
    for j = 1:length(types)
        dataset = datasets{i};
        type = types{j};
        if ~isempty(strfind(dataset,'fullres'))
            gt = '/esat/sadr/amartino/monge428New/data/fullRes_pcloud_test_new_GCO.ply';
        else
            gt = '/esat/sadr/amartino/monge428New/data/pcloud_gt_test_new_GCO.ply';
        end
        fprintf('3D 3rd layer %s %s\n------------\n',dataset,type);
        l = ['/esat/nihal/jknopp/3d_ret_recog_data/DT-SOL/' dataset '/2andelo/full_pcl_labeling_' type '_3D3rdLayer.ply'];
        score = EvaluateMeshLabeling(l,gt)
%         score.accuracy_pascal
    end
end

%% mine
gt = '/esat/sadr/amartino/monge428New/data/pcloud_gt_test_new_GCO.ply';
l = '/esat/sadr/amartino/monge428New/data/work/pcl/models/monge428New_layer1+3D_3DCRF_2Dlayer3.ply';
score = EvaluateMeshLabeling(l,gt)

%%
gt = '/esat/sadr/amartino/monge428New/data/pcloud_gt_test_new_GCO.ply';
l = '/esat/nihal/jknopp/3d_ret_recog_data/DT-SOL/monge428_27/2andelo/full_pcl_labeling_layer1+3D_3DCRF_3D3rdLayer.ply';
score = EvaluateMeshLabeling(l,gt)

%% SFM
gt = '/esat/sadr/amartino/monge428New/data/sfm_pcloud_test.ply';
l = '/esat/sadr/amartino/monge428New/data/work/pcl/models/sfm_3D_3DCRF.ply';
score = EvaluateMeshLabeling(l,gt)

%% PMVS
gt = '/esat/sadr/amartino/monge428New/data/pmvs_pcloud_test.ply';
l = '/esat/sadr/amartino/monge428New/data/work/pcl/models/pmvs_3D_3DCRF.ply';
score = EvaluateMeshLabeling(l,gt)

