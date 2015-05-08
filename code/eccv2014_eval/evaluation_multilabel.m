function score = evaluation_multilabel(gt, result, skip_idx)
% score = evaluation_multilabel(gt, result, skip_idx)
%  
%  Evaluation for multi class classification
%  measures are: 
%           tp, fn, fp, tn
%           pixel, classwise, pascal accuracy
%           precision, recall, f1
%
% input:        
%               gt ... label indices of ground truth
%               res ... label indices of predicted result
%               skip_idx  label indices to skip (e.g. void)
%
% author: hayko riemenschneider, 2014
%
% See also evaluation_multilabel_print

% todo
% - add actual segmentation based evaluation


%% DEMO EXAMPLE
% demo shows accuary for 5 classes and their harshness in evaluation exactness

if(~exist('skip_idx','var')), skip_idx = []; end;

if(~exist('gt','var') || isempty(gt))
    
    % generate gt - 6 classes, each with their cardinality as frequency
    gt = []; for c = 0:5; gt = [gt repmat(c,1,c^2+1)];  end
    
    % generate result
    clear score
    indices = randperm(length(gt));
    for i = 1: length(gt)
        result = gt;
        result(indices(1:i)) = gt(indices(1:i))-1;
        if(result(1)< 0), result(1) = 5; end
        score(i) = evaluation_multilabel(gt,result);
    end
    
    figure, hold on, grid on
    plot((1: length(gt))/ length(gt), cat(1,score(:).mean_pixel),'r')
    plot((1: length(gt))/ length(gt), cat(1,score(:).mean_class),'g')
    plot((1: length(gt))/ length(gt), cat(1,score(:).mean_pascal),'b')
    xlabel('percent of incorrect samples')
    ylabel('accuracy')
    legend({'pixel','class','pascal'})
    
    return
end

%% BASIC EVALUATION STATS
clear score
% Commented out the following 2 lines (unnecessary). 14.4.2015. Angelo Martinovic
% gt = double(gt(:));      
% result = double(result(:));


% scores of all stacks (global score vs. stack individually / image wise)
%num_labels = length(unique(gt));
num_labels = max(gt)+1;

% confusion table
% diagonal is TP
% sum over columns = all positive
% sum over rows = frequency in GT per label


% fast way to get the confusion table
sumim = 1+gt+(result)*num_labels;
conf = zeros(num_labels);
conf(:) = histc(sumim(:),1:num_labels*num_labels);
skip_idx(skip_idx>num_labels)  = [];
conf(skip_idx,:) = [];
conf(:,skip_idx) = [];
score.confusion = conf;

% cant use result because it still contains void labels
% needs size of all remaining labels after void removal
score.all = sum(score.confusion(:));

tp = diag(conf)';
fp = sum(conf,1) - tp; % columns = all positive, hence fp = sum - tp;
fn = sum(conf,2)' - tp; % rows = all in GT (tp+fn), hence fn = sum - tp
tn = sum(score.confusion(:)) - tp - fp - fn; % all that remains from all samples

% basic components
score.tp = tp;
score.fn = fn;
score.fp = fp;
score.tn = tn;
score.all = tp + fn + fp + tn;
score.class = sum(score.confusion,2)'; % number of all positive (sum over column)


%% HIGH LEVEL EVALUATIONS

% precision = true positive / total positive
score.precision = 100*tp ./ max(1,(tp+fp));

% recall = true positive / true positive + false negative (== missed positive)
score.recall = 100*tp ./ max(1,(tp+fn));

% f1 measure = balance of recall and precision
score.f1 = 2* score.precision .* score.recall ./ max(1,(score.precision + score.recall));

% pixelwise accuracy
% true positive + true negative / overall length of GT
% effect: simple scoring, bias towards size of class
%score.accuracy_pixel = (tp+tn) ./ score.all; % HARSHER?
%score.accuracy_pixel = sum(tp) ./ score.all; % SOMEHOW WRONG??
score.accuracy_pixel = 100*sum(tp) ./ max(1,sum(score.confusion(:)));


% classwise accuracy
% true positive  / length of each class (tp in GT)
% effect: scores per class, then averages (no class bias)
score.accuracy_classwise = 100*tp ./ max(1,score.class);


% pascal classwise accuracy (aka intersection of union, aka jaccard distance)
% true positive / (true positive + false positive + false negative)
% effect: also punishes fp (too many) and fn (missed) scores
score.accuracy_pascal = 100*tp ./ max(1, tp + fp + fn);


score.mean_p = mean(score.precision);
score.mean_r = mean(score.recall);
score.mean_f1 = mean(score.f1);

score.mean_pixel = mean(score.accuracy_pixel);
score.mean_class = mean(score.accuracy_classwise);
score.mean_pascal = mean(score.accuracy_pascal);



%% OBSOLETE OLD SLOW CODE

% 
% tp = zeros(1, num_labels); tn = tp; fp = tp; fn = tp;
% for c = 0: num_labels-1
%     gtc = gt(gt==c);
%     resc = result(gt==c);
%     tp(c+1) = sum(gtc == resc);
%     fn(c+1) = sum(gtc ~= resc);
%     fp(c+1) = sum(gt(result==c) ~= result(result==c));
%     tn(c+1) = sum(result(gt~=c) ~= c);
% 
% %    tp(c+1) = sum(gt(gt==c) == result(gt==c));
% %    fn(c+1) = sum(gt(gt==c) ~= result(gt==c));
% %    fp(c+1) = sum(gt(result==c) ~= result(result==c));
% %    tn(c+1) = sum(result(gt~=c) ~= c);
% 
% end
