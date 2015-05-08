function evaluation_multilabel_print(score, labels, labels_unused)
% evaluation_multilabel_print (score, labels) displays the accuracies.
% Prints a nice wrapper for the score struct.
%
% author: hayko riemenschneider, 2014
%
% See also evaluation_multilabel


labels(labels_unused)=[];

% fprintf('Average pixel accuracy: %6.3f%%\n',score.mean_pixel);
% fprintf('Average class accuracy: %6.3f%%\n',score.mean_class);
% fprintf('Average PASCAL accuracy: %6.3f%%\n',score.mean_pascal);
% fprintf('\n');

if exist('labels','var')
    for j=1:size(score.confusion,1)
        fprintf('%14s: %6.2f%% %6.2f%%\n',labels{j},score.accuracy_classwise(j), score.accuracy_pascal(j));
    end
end
fprintf('\n');

fprintf('%6s: %6.2f%% %6.2f%% %6.2f%%\n','mean', score.mean_pixel,score.mean_class, score.mean_pascal);
fprintf('%6s: %s %s %s\n','mean', ' pixel','  class','  pascal');



