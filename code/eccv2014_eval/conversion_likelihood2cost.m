function unary_cost = conversion_likelihood2cost(likelihood, do_log)
% unary_cost = conversion_likelihood2cost(likelihood, do_log)
%  Convert likelihoods to costs by normalizing per row.
%
% note: cleans out first column (void label)
% author: hayko riemenschneider, 2014

% convert from likelihoods to costs
unary_cost = likelihood';

% kill void label evidence
unary_cost(1,:) = 0;
% normalize rows
unary_cost = bsxfun(@times, unary_cost, 1 ./ max(1,sum(unary_cost,1)));

if(do_log)
    % todo: log needs different pairwise balancing alpha
    unary_cost = -log(unary_cost + 1e-2);
    unary_cost(1,:) = -log(1-1e-5);
else
    % simple since all probs, cost = 1-probs;
    unary_cost = 1-unary_cost;
end

