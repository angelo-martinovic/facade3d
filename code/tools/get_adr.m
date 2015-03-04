
%
%
%  get_adr('L1_res',datasetConfig,'2D')
%

function adr = get_adr(type , datasetConfig , par1 , par2 , par3)


switch type
    case '2D_labels'
        adr = [datasetConfig.dataLocation datasetConfig.labelLocation];
    case '2D_images'
        adr = [datasetConfig.dataLocation datasetConfig.imageLocation];  
    case 'work'
        adr = [datasetConfig.outputLocation 'work/'];      
    case '2D_image'
        adr = [datasetConfig.outputLocation 'work/' par1 '.jpg'];
    case '2D_image_orig'
        adr = [datasetConfig.dataLocation datasetConfig.imageLocation par1 '.jpg'];
    case '2D_label'
        adr = [datasetConfig.dataLocation datasetConfig.labelLocation par1 '.png'];
    case '2D_ppm'
        adr = [datasetConfig.outputLocation 'work/' par1 '.ppm'];
    case '2D_segmentation'
        adr = [datasetConfig.outputLocation 'work/' par1 '.seg'];
    case '2D_rectParams'
        adr = [datasetConfig.outputLocation 'work/' par1 '_rect.dat'];
    case '2D_rectSkip'
        adr = [datasetConfig.outputLocation 'work/' par1 '_rect.skip'];
    case '2D_scale'
        adr = [datasetConfig.outputLocation 'work/' par1 '_scale.dat'];  
    case '2D_features'
        adr = [datasetConfig.outputLocation 'work/' par1 '.features.' par2 '.mat'];
    case '2D_classifier_folder'
        adr = [datasetConfig.outputLocation 'work/classifier/'];  
    case '2D_classifier'
        adr = [datasetConfig.outputLocation 'work/classifier/' par1 '.mat'];  
    case '2D_classification'
        adr = [datasetConfig.outputLocation 'work/classifier/' par1 '/'];  
    case '2D_marginals'
        adr = [datasetConfig.outputLocation 'work/classifier/' par1 '/' par2 '.marginal.txt'];  
    case '2D_detectionFolder'
        adr = [datasetConfig.outputLocation 'work/detections/'];  
    case '2D_detectorOutputFolder'
        adr = [datasetConfig.outputLocation 'work/detections/detections-' par1];  
    case '2D_detectorMeanDetection'
        adr = [datasetConfig.outputLocation 'work/detections/detections-' par1 '/meanDet.mat'];  
    case '2D_detections'
        adr = [datasetConfig.outputLocation 'work/detections/detections-' par1 '/' par2 '.txt'];  
    case 'cache'
        adr = [datasetConfig.outputLocation 'work/cache/'];   
    case 'cache_classifier_train'
        adr = [datasetConfig.outputLocation 'work/cache/classifier_train_' par1 '_' par2 '.mat'];   
    case 'cache_classifier_test'
        adr = [datasetConfig.outputLocation 'work/cache/classifier_test_' par1 '_' par2 '.mat']; 
    case 'cache_classifier_prediction'
        adr = [datasetConfig.outputLocation 'work/cache/classifier_prediction_' par1 '_' par2 '.mat'];  
    case 'pcl'
        adr = [datasetConfig.dataLocation datasetConfig.pointCloud];   
    case 'split'
        adr = [datasetConfig.dataLocation datasetConfig.splitData];  
    case 'pcl_gt_test'
        adr = [datasetConfig.dataLocation datasetConfig.groundTruthTest];   
    case 'cameras'
        adr = [datasetConfig.dataLocation datasetConfig.cameras];   
        
    case 'pclDir'
        adr = [datasetConfig.outputLocation 'work/pcl/'];      
    case 'pclModelDir'
        adr = [datasetConfig.outputLocation 'work/pcl/models/'];   
    case 'pclProbDir'
        adr = [datasetConfig.outputLocation 'work/pcl/probs/']; 
    case 'image_classifier_unaries'
        adr = [datasetConfig.outputLocation 'work/classifier/' par1 '/' par2 '_2Dpotentials.mat'];  

    case 'pcl_labeling'
        adr = [datasetConfig.outputLocation 'work/pcl/models/' datasetConfig.name '_' par1 '.ply'];
    case 'pcl_unaries'
        adr = [datasetConfig.outputLocation 'work/pcl/probs/' datasetConfig.name '_' par1 '.mat'];

        
    case 'pcl_3DCRF_labeling'
        adr = [datasetConfig.outputLocation 'work/pcl/models/' datasetConfig.name '_' par1 '_3DCRF.ply'];
    case 'pcl_3DCRF_unaries'
        adr = [datasetConfig.outputLocation 'work/pcl/probs/' datasetConfig.name '_' par1 '_3DCRF.mat'];
        
    case 'splitOutputDir'
        adr = [datasetConfig.outputLocation 'work/pcl/split/' datasetConfig.name '_' par1 '/'];
    case 'facadeIDs'
        adr = [datasetConfig.outputLocation 'work/pcl/split/' datasetConfig.name '_' par1 '/' datasetConfig.name '_facadeIDs.mat'];

    case 'splitPotentials'
        adr = [datasetConfig.outputLocation 'work/pcl/split/' datasetConfig.name '_' par1 '/' datasetConfig.name '_split_' par2 '_potentials.mat'];
    case 'splitLabeling'
        adr = [datasetConfig.outputLocation 'work/pcl/split/' datasetConfig.name '_' par1 '/' datasetConfig.name '_split_' par2 '_labeling.ply'];
    case 'splitGT'
        adr = [datasetConfig.outputLocation 'work/pcl/split/' datasetConfig.name '_' par1 '/' datasetConfig.name '_split_' par2 '_GT.ply'];
    case 'splitColors'
        adr = [datasetConfig.outputLocation 'work/pcl/split/' datasetConfig.name '_' par1 '/' datasetConfig.name '_split_' par2 '_colors.ply'];
    case 'splitPlane'
        adr = [datasetConfig.outputLocation 'work/pcl/split/' datasetConfig.name '_' par1 '/' datasetConfig.name '_split_' par2 '_plane.mat'];
        
    case 'orthoColors'
        adr = [datasetConfig.outputLocation 'work/pcl/split/' datasetConfig.name '_' par1 '/' datasetConfig.name '_split_' par2 '_ortho_colors.png'];
    case 'orthoLabels'
        adr = [datasetConfig.outputLocation 'work/pcl/split/' datasetConfig.name '_' par1 '/' datasetConfig.name '_split_' par2 '_ortho_labeling.png'];
    case 'orthoPotentials'
        adr = [datasetConfig.outputLocation 'work/pcl/split/' datasetConfig.name '_' par1 '/' datasetConfig.name '_split_' par2 '_ortho_potentials.mat'];
    case 'orthoLabelingLayer3Img'    
        adr = [datasetConfig.outputLocation 'work/pcl/split/' datasetConfig.name '_' par1 '/' datasetConfig.name '_split_' par2 '_ortho_labeling_layer3.png'];
    case 'orthoLabelingLayer3Ply'    
        adr = [datasetConfig.outputLocation 'work/pcl/split/' datasetConfig.name '_' par1 '/' datasetConfig.name '_split_' par2 '_ortho_labeling_layer3.ply'];
        
    case '3D_L3_Ortho2D'
        adr = [datasetConfig.outputLocation 'work/pcl/split/' datasetConfig.name '_' par1 '_Ortho2D/'];
    case '3D_L3_Pure3D'
        adr = [datasetConfig.outputLocation 'work/pcl/split/' datasetConfig.name '_' par1 '_Pure3D/'];
    
    case '3D_L3_Ortho2D_labeling'
        adr = [datasetConfig.outputLocation 'work/pcl/models/' datasetConfig.name '_' par1 '_L3_Ortho2D.ply'];
    case '3D_L3_Ortho2D_labeling_overlay'
        adr = [datasetConfig.outputLocation 'work/pcl/models/' datasetConfig.name '_' par1 '_L3_Ortho2D_overlay.ply'];
        
    case '3D_L3_Pure3D_labeling'
        adr = [datasetConfig.outputLocation 'work/pcl/models/' datasetConfig.name '_' par1 '_L3_Pure3D.ply'];
    case '3DL_bboxes'
        adr = [get_adr('3D_L3_Pure3D',datasetConfig,par1),'bbox/','facid=',num2str(par2),'.mat'];
    case 'desc3d'
        adr = fullfile(datasetConfig.outputLocation,['work/pcl/desc/' datasetConfig.name '_' par1,'_imSiz=',num2str(par2),'.mat']);
    case 'classifier3d'
%         adr = fullfile('/users/visics/amartino/',[datasetConfig.name '_3Dclassifier_' par1 '.mat']);
        adr = fullfile(datasetConfig.outputLocation,['work/pcl/' datasetConfig.name '_3Dclassifier_' par1 '.mat']);
    otherwise
        error('Unrecognized type!');
end

end