
%
%
%  get_adr('L1_res',datasetConfig,'2D')
%

function adr = get_adr(type , datasetConfig , par1 , par2)


switch type
    case 'L1_labeling'
        adr = [datasetConfig.outputLocation '/work/pcl/models/' datasetConfig.name '_' par1 '_3DCRF.ply'];
    case 'L1_prob'
        adr = [datasetConfig.outputLocation '/work/pcl/probs/' datasetConfig.name '_' par1 '_3DCRF.ply'];

end