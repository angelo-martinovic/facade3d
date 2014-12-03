%EVALUATELABELING Evaluates the labeling of a single image.
% 
% [ correctPixels,totalPixels,confusionMatrixImg ] = 
%   EvaluateLabeling( ourLabeling, groundTruth, nClasses, ignoreClasses)
%
% ignoreClasses should contain at least class '0' - outliers, e.g. [0 8]
% nClasses - total number of classes in the ground truth, including ignored
% classes, but excluding 0, e.g. 8
% 
% Returns the number of correctly classified pixels, total number of
% pixels, and the confusion matrix.
function [ correctPixels,totalPixels,confusionMatrixImg ] = EvaluateLabeling(ourLabeling, groundTruth, nClasses, ignoreClasses )

    if nargin~=4
        error('Usage: EvaluateLabeling( ourLabeling, groundTruth, nClasses, ignoreClasses)');
    end
    
    if size(ourLabeling,1)<size(ourLabeling,2)
        ourLabeling = ourLabeling';
    end
    
    ourLabeling(ismember(ourLabeling,ignoreClasses)) = NaN;
    groundTruth(ismember(groundTruth,ignoreClasses)) = NaN;
    
    % Create the confusion matrix
    [confusionMatrixImg, order] = confusionmat(groundTruth(:),ourLabeling(:),'order',1:nClasses);
    
%     for i=1:size(ourLabeling,1)
%         for j=1:size(ourLabeling,2)
%             if ( ~ismember(ourLabeling(i,j), ignoreClasses) && ...
%                 ~ismember(groundTruth(i,j), ignoreClasses) )
%                confusionMatrixImg(groundTruth(i,j),ourLabeling(i,j)) = confusionMatrixImg(groundTruth(i,j),ourLabeling(i,j)) + 1;
%             end
%         end
%     end


    % Calculate the number of correctly classified pixels
    correctTestImg = (ourLabeling==groundTruth & ~isnan(groundTruth));
    correctPixels = sum(correctTestImg(:));
 
    % Calculate the total number of pixels
    totalPixels = sum(sum(~isnan(groundTruth)));

end

