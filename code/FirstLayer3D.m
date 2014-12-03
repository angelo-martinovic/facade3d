% if ~exist('scene','var') || isempty(scene);
%     facade_init_all_data;

%% ========================================================================
%
%      3D: RF=Layer 1
%
%  ========================================================================

desc_reco      = 'desc'; %%% which descriptor, desc=concatenation of all descs.  
unary_clssifer = 'rf';   %%% classifier, RF=random forest
sample     = 10;2e3;   %%% # of samples per class
nrf_tree   = 10;200;   %%% # of trees in RF


%--- store only descriptors with labels, then learn classifier and then classfiy data  
[X,Y,Xtest] = facade_unary_class('get_desc','desc_reco',desc_reco,'unary_clssifer',unary_clssifer,'sample',sample,'norm_data',1);
model       = facade_unary_class('learn','unary_clssifer',unary_clssifer,'X',X,'Y',Y,'rf_trees',nrf_tree);
[prb,cprb]  = facade_unary_class('test','unary_clssifer',unary_clssifer,'model',model,'Xtest',Xtest);



%--- plot results in matlab, only the sub-sampled set as matlab is slow...
if 0,
    scene.plot_scene('sample',10,'Q',cprb');
    scene.plot_scene('sample',10,'Q',scene.lindex');
end
