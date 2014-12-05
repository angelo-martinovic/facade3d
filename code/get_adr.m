
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
    case '2D_classifier'
        adr = [datasetConfig.outputLocation 'work/classifier/'];  
    case '2D_classification'
        adr = [datasetConfig.outputLocation 'work/classifier/' par1 '/'];  
    case '2D_detections'
        adr = [datasetConfig.outputLocation 'work/detections/'];  
    case 'cache'
        adr = [datasetConfig.outputLocation 'work/cache/'];   
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
    case '2D_classifier_unaries'
        adr = [datasetConfig.outputLocation 'work/classifier/' par1 '/' par2 '_2Dpotentials.mat'];  

    case '3D_L1_labeling'
        adr = [datasetConfig.outputLocation 'work/pcl/models/' datasetConfig.name '_' par1 '.ply'];
    case '3D_L1_unaries'
        adr = [datasetConfig.outputLocation 'work/pcl/probs/' datasetConfig.name '_' par1 '.mat'];

        
    case '3D_L2_labeling'
        adr = [datasetConfig.outputLocation 'work/pcl/models/' datasetConfig.name '_' par1 '_3DCRF.ply'];
    case '3D_L2_unaries'
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
        %adr = fullfile('/esat/nihal/jknopp/3d_ret_recog_data/DT-SOL/monge428_27/','3rd_layer',['result_NEW',num2str(par1),'_facadeid=',num2str(par2),'.mat']);
    case 'grav_vec'
        adr = ['/esat/sadr/amartino/monge428New/data/work/pcl/split/monge428New_fullRes_layer1+3D_3DCRF/monge428New_fullRes_split_',num2str(par1),'_plane.mat'];
    case 'desc3d'
        adr = fullfile(datasetConfig.outputLocation,['work/pcl/desc/' datasetConfig.name '_' par1,'_imSiz=',num2str(par2),'.mat']);
    case 'image_data'
        adr = ['/esat/sadr/amartino/monge428New/data/work/pcl/split/monge428New_fullRes_layer1_3DCRF/monge428New_fullRes_split_',num2str(par1),'_plane.mat'];
    case 'image'
        adr = [datasetConfig.dataLocation datasetConfig.imageLocation,'monge428New_fullRes_split_',num2str(par1),'_ortho_colors.png'];
    otherwise
        error('Unrecognized type!');
end

end