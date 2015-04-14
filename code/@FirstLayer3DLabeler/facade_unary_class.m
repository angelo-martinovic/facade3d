%
%
%
%
%
%    classification of 3D points
%
%    it does:
%         1) method='get_desc' extracts only descs and labels from SScene class and store then in vars: X, Y and Xtest   
%         2) method='learn'    learns classifcation model from X and Y
%         3) method='test'     classifies data Xtest using model
%
%    see i.e. facade_run.m for its usage
%
%
%
%         Honza Knopp, 2014
%
%
%
%
%
%
%

function [out1,out2,out3,out4] = facade_unary_class (obj,method,varargin)
p = inputParser;
% p.addOptional('sample', 900);
% p.addOptional('desc_reco', 'desc');
% p.addOptional('unary_classifier', 'rf');
p.addOptional('model', []);
p.addOptional('X', []);
p.addOptional('Y', []);
p.addOptional('Xtest', []);
% p.addOptional('norm_data', 1);
% p.addOptional('rf_trees', 40);
p.parse(varargin{:}); fldnames = fieldnames(p.Results); for a=1:length(fldnames), eval([fldnames{a},' = p.Results.',fldnames{a},';']); end;

out1=[]; out2=[]; out3=[]; out4=[];

%%
scene               = obj.test_data;
train_data          = obj.train_data;

c = obj.config.c3D.classifier;
unary_classifier    =  c.unary_classifier;
desc_reco           =  c.desc_reco;
sample              =  c.sample;
rf_trees            =  c.nrf_tree;
norm_data           =  c.norm_data;
min_leaf            =  c.min_leaf;
oob_pred            =  c.oob_pred;
cost_matrix         =  c.cost_matrix;
%%
switch method,
    case 'get_desc' %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        switch desc_reco,
            case 'desc'
                X       = [train_data.desc];
                Xtest   = scene.desc;
            case 'desc_from_train'
                X       = [train_data.desc];
                Xtest   = train_data.desc;
            case 'si'
                X       = [train_data.si];
                Xtest   = [scene.si];
            case 'rgb+si+n'
                X       = [train_data.rgb ; train_data.nxyz ; train_data.si];
                Xtest   = [scene.rgb ; scene.nxyz ; scene.si];
            case 'n'
                X       = [train_data.nxyz];
                Xtest   = [scene.nxyz];
            case 'rgb'
                X       = [train_data.rgb];
                Xtest   = [scene.rgb];
            case 'depth'
                X       = [train_data.dist2plane];
                Xtest   = [scene.dist2plane];
            case 'heigh-inv'
                X       = [train_data.heigh_inv];
                Xtest   = [scene.heigh_inv];
            case 'heigh'
                X       = [train_data.heigh];
                Xtest   = [scene.heigh];
            case 'without_heigh'
                X       = [train_data.desc([1:6,8:end],:)];
                Xtest   = scene.desc([1:6,8:end],:);
            case 'without_heigh_inv'
                X       = [train_data.desc([1:7,9:end],:)];
                Xtest   = scene.desc([1:7,9:end],:);
            case 'lab'
                [dl,da,db] = rgb2lab(scene.rgb(1,:),scene.rgb(2,:),scene.rgb(3,:));
                [tdl,tda,tdb] = rgb2lab(train_data.rgb(1,:),train_data.rgb(2,:),train_data.rgb(3,:));
                X       = [tdl;tda;tdb];
                Xtest   = [dl;da;db];
            case 'lab+si+n'
                [dl,da,db] = rgb2lab(scene.rgb(1,:),scene.rgb(2,:),scene.rgb(3,:));
                [tdl,tda,tdb] = rgb2lab(train_data.rgb(1,:),train_data.rgb(2,:),train_data.rgb(3,:));
                X       = [tdl;tda;tdb;train_data.nxyz ; train_data.si];
                Xtest   = [dl;da;db;scene.nxyz ; scene.si];
            case 'rgb+lab+si+n'
                [dl,da,db] = rgb2lab(scene.rgb(1,:),scene.rgb(2,:),scene.rgb(3,:));
                [tdl,tda,tdb] = rgb2lab(train_data.rgb(1,:),train_data.rgb(2,:),train_data.rgb(3,:));
                X       = [tdl;tda;tdb;train_data.nxyz ; train_data.si ; train_data.rgb];
                Xtest   = [dl;da;db;scene.nxyz ; scene.si ; scene.rgb];
        end
        Y = train_data.lindex;
        %--- normalize
        if norm_data,
            minX = min(X,[],2);
            X     = bsxfun(@plus , X , -minX);
            Xtest = bsxfun(@plus , Xtest , -minX);
            maxX = max(X,[],2);
            X     = bsxfun(@times , X , 1./maxX);
            Xtest = bsxfun(@times , Xtest , 1./maxX);
        end
        %--- sample
        if sample~=0,
            Xa = [];
            Ya = [];
            n_tot_ftrs    = length(Y);
            n_tot_samples = abs(sample)*length(unique(Y));
            for c=unique(Y),
                samples_per_class = sample;
                Yc       = Y==c;
                idx_pos  = find(Yc);
                rand_ordering = randperm(length(idx_pos));
                if sample<0, %%% arbitrary # of samples per class
                    samples_per_class = ceil(length(idx_pos)/n_tot_ftrs*n_tot_samples);  %%% keep the distribution of classes
                end
                while length(rand_ordering)<samples_per_class, %%% repeat some data to have enough samples if necassary :)
                    fprintf('      facade_unary: repeating some data to have enough samples per class %d\n',c);
                    rand_ordering = [rand_ordering rand_ordering];%(1:samples_per_class-length(rand_ordering))];
                end
                idx_pos  = idx_pos( rand_ordering( 1:samples_per_class ) );
                Xa = [Xa , X(:,idx_pos)];
                Ya = [Ya , idx_pos*0+c];
            end;
        else
            Xa = X;
            Ya = Y;
        end
        out1 = Xa;
        out2 = Ya;
        out3 = Xtest;
        
    case 'learn' %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        error = 0;
        switch unary_classifier
            case 'rf'         
                if obj.config.nWorkers>1
                    InitializeParallel(obj.config.nWorkers);
                    
                    % Reproducibility 
                    s = RandStream('mlfg6331_64');
                    paroptions = statset('UseParallel',true,...
                        'Streams',s,'UseSubstreams',true);             
                else
                    paroptions = statset();
                end

                model = TreeBagger(rf_trees,X',Y','Method','classification',...
                    'Prior','Uniform',...
                    'MinLeaf',min_leaf,...
                    'OOBPred',oob_pred,...
...%                     'Cost',cost_matrix,...
                    'Options',paroptions);
                
                if strcmp(oob_pred,'Off')
                    model = model.compact();
                else
                    error = oobError(model);
                end
            case 'svm'
                model  = svmtrain(Y',full(X)','-b 1');
            case {'knn'}
                model = [X;Y];
            case 'liblinear'
                
                cmd = '-s 2'; 
                % Weight the classes with their inverse frequency
                aa=hist(Y,1:7);
                aa = numel(Y)./aa;
                bb = 1:7;
                ww = sprintf('-w%d %.2f ',[bb;aa]);
                cmd = [cmd ' ' ww];
                model = train(Y', sparse(X') ,cmd);
        end
        out1 = model;
        out2 = error;
        
    case 'test' %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        prb = [];
        cprb = [];
        switch unary_classifier
            case 'rf'
                [~,scores] = predict(model,Xtest');
                prb = scores';
                [~,cprb]=max(prb);
            case 'svm'
                %svmclassify(model,Xtest');
                [cprb, accuracy, prb] = svmpredict(Xtest(1,:)'*0, Xtest' , model,'-b 1'); % test the tra
                cprb = cprb';
                prb = prb';
            case 'knn'
                k = 2;
                Y = model(end,:);
                for c=unique(Y), %%% distance to model = trainset
                    % fprintf('\b\b%2i',c);
                    [i,d] = knnsearch(model(1:end-1,Y==c)',Xtest','K',k);
                    dd(c,:) = min(d,[],2)';
                end; % fprintf('\b\b');
                dd = -dd;
                dd = bsxfun(@plus  , dd , -min(dd)); %%% normalize
                prb = bsxfun(@times , dd , 1./sum(dd)); %%% normalize
                [~,cprb]=max(prb);
            case 'liblinear'
                 [cprb, accuracy, prb] = predict(Xtest(1,:)'*0, sparse(Xtest'), model, '-b 1 -q');

                 % Re-order classes based on model order
                 prb(:,model.Label) = prb;
                 
                 cprb = cprb';
                 prb = prb';
                    
        end
        out1 = prb;
        out2 = cprb;
end
        
end
