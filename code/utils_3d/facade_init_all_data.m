

%% paths



test_data_separ = 1;
si_dimensions = .3;




%% read data + descriptors
%---- read basic data
fprintf('  read SScene_An.... dataset = ''%s'' ',dataset);
clear train_data scene
global train_data scene
train_data = SScene_An(path_mat_data_dir,postfix_path_data_orig);
train_data = train_data.read_mat_data();

%--- read/calc  desc
train_data = train_data.process_data('compute_get_si','si_dimensions',si_dimensions);

%--- fix labeling ids
train_data.lindex = train_data.lindex-1; %%% applies to monge where zero should be background crap...

%--- separate into train/test
scene       = train_data.keep_spec_data_ids(train_data.flag==2);
train_data  = train_data.keep_spec_data_ids(train_data.flag==1);

%--- merge descriptors together to one vector
[train_data minDESC maxDESC V D] = train_data.process_data('create_desc_from_weak_descs');

%--- also desc
scene = scene.process_data('create_desc_from_weak_descs','minX',minDESC,'maxX',maxDESC,'V',V,'D',D);

disp(' ---- init done :) -----');


