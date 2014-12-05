




function FirstLayer3D(datasetConfig)

fprintf('------- 3D first layer ------\n');
global scene train_data
if ~exist('scene','var') || isempty(scene);
     facade_init_all_data
end




%--- init some pars
desc_reco      = 'desc'; %%% which descriptor, desc=concatenation of all descs.  
unary_clssifer = 'rf';   %%% classifier, RF=random forest
sample     = 10;2e3;   %%% # of samples per class
nrf_tree   = 10;200;   %%% # of trees in RF




%--- store only descriptors with labels, then learn classifier and then classfiy data  
     fprintf('   ...correct desc format creation');
[X,Y,Xtest] = facade_unary_class('get_desc','desc_reco',desc_reco,'unary_clssifer',unary_clssifer,'sample',sample,'norm_data',1);
     fprintf(' -done\n   ...learning %s for sample=%i, nrftrees=%i',unary_clssifer,sample,nrf_tree);
model       = facade_unary_class('learn','unary_clssifer',unary_clssifer,'X',X,'Y',Y,'rf_trees',nrf_tree);
     fprintf(' -done\n   ...test part');
[prb,cprb]  = facade_unary_class('test','unary_clssifer',unary_clssifer,'model',model,'Xtest',Xtest);
     fprintf(' -done\n');


     
     
     
%--- plot results in matlab, only the sub-sampled set as matlab is slow...
if 0,
    scene.plot_scene('sample',30,'Q',cprb');
    scene.plot_scene('sample',30,'Q',scene.lindex');
end

%--- save result
if 1,
    path_labeling = get_adr('3D_L1_labeling',datasetConfig,'3D');
    % scene.export_as_full_pcl_data( datasetConfig , cprb , path_labeling ); %%% export labeling
    path_prb = get_adr('3D_L1_unaries',datasetConfig,'3D');
    %dt = [scene.pts ; prb];
    save( path_prb , 'prb' );
    fprintf('   results saved to: %s and %s\n',path_labeling,path_prb);
end

end
