


desc_reco      = 'desc'; %%% which descriptor, desc=concatenation of all descs.  
unary_clssifer = 'rf';   %%% classifier, RF=random forest
sample     = 2e3;   %%% # of samples per class
nrf_tree   = 200;   %%% # of trees in RF


%--- store only descriptors with labels
[X,Y,Xtest] = facade_unary_class('get_desc','desc_reco',desc_reco,'unary_clssifer',unary_clssifer,'sample',sample,'norm_data',1);
%--- leanr classifier
model       = facade_unary_class('learn','unary_clssifer',unary_clssifer,'X',X,'Y',Y,'rf_trees',nrf_tree);
%--- classify test data
[prb,cprb]  = facade_unary_class('test','unary_clssifer',unary_clssifer,'model',model,'Xtest',Xtest);


% 
% %perfs=[]; cperfs=[]; prrecacc = []; cprbs = []; prbs = []; tags = {};
% fprintf('dim(scene.desc)=%i\n',size(scene.desc,1));
% for nrf_tree = nrf_trees;
%     for desc_reco = desc_recos,
%         desc_reco = desc_reco{1};
%         for sample = samples,
%             for unary_clssifer = u_classfrs,
%                 unary_clssifer = unary_clssifer{1};
%                 fprintf('  %s,%s; samples=%i, trees=%i;           ',unary_clssifer,desc_reco,sample,nrf_tree);
%                 tperf = []; tcperf = []; 
%                 for a=1:1,
%                     [X,Y,Xtest] = facade_unary_class('get_desc','desc_reco',desc_reco,'unary_clssifer',unary_clssifer,'sample',sample,'norm_data',1);
% %                    tic
%                     model       = facade_unary_class('learn','unary_clssifer',unary_clssifer,'X',X,'Y',Y,'rf_trees',nrf_tree);
%  %                   train_toc   = toc;
%   %                  tic
%                     [prb,cprb]  = facade_unary_class('test','unary_clssifer',unary_clssifer,'model',model,'Xtest',Xtest);
%    %                 test_toc = toc;
%                     %cprbs = [cprbs;cprb];
%                     %prbs = [prbs;prb];
%                     %[tperf(a) tcperf(a,:)] = scene.eval_perf_angleo('Q',prb);
%                     %fprintf('\b\b\b\b\b%1.3f',tperf(a));
%                 end
%                 %fprintf('   cnt=%i;',a);
%                 %perf    = mean(tperf);
%                 %tcperf  = mean(tcperf);
% %                 fprintf('     perf=<strong>%.3f</strong> ,',perf);
% %                 fprintf(' time[train,test]=[%.2fsec,%.2f.sec]\n',train_toc,test_toc);
% %                 perfs   = [perfs perf];
% %                 %cperfs  = [cperfs cperf'];
% %                 tags{1,length(perfs)}    = desc_reco;
% %                 tags{2,length(perfs)}    = sample;
% %                 tags{3,length(perfs)}    = unary_clssifer;
% %                 tags{4,length(perfs)}    = nrf_tree;
% %                 tags{5,length(perfs)}    = test_toc;
% %                 tags{6,length(perfs)}    = train_toc;
% %                 
% %                 [t.pr t.rec t.acc t.cpr t.crec t.cacc] = scene.eval_perf('Q',prb);
% %                 prrecacc{length(prrecacc)+1} = t;
%             end
%         end
%     end
% end
