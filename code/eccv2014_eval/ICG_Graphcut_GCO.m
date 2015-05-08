function [labels, E, D, S, L, t] = ICG_Graphcut_GCO(unary, pairwise, alpha, labelcost, smoothcost, expansion, factor_int32)
% ICG_Graphcut_GCO(unary, pairwise, alpha, labelcost, smoothcost, expansion, factor_int32)
%   A one-line wrapper for GCO's Graphcut library given unary and pairwise energies (costs).
%
% Description:
%   [see GCO readme]
%
% Input
%   unary       NxL unary data cost (NumSites x NumLabels)
%               Unary is the sum over all unary potentials (weighted outside)
%   pairwise    NxN pairwise data cost (NumSites x NumSites)
%               Pairwise is the sum over all pairwise potentials (weighted outside)
%   alpha       Weight between unary and pairwise... [default: 1]
%               pairwise = pairwise * alpha;
%
%  Warning: GCO uses only int32, thus all 'decimal' costs are multiplied by 100.
%  Run without parameters to see demo.
%
% Example:
%   [run without arguments]
%
% Authors:
%         Hayko Riemenschneider, 2012
%
% See also GCO_SetDataCost, GCO_SetNeighbors, GCO_Expansion, GCO_GetLabeling.


addpath(genpath('/scratch_net/biwisrv01_second/varcity/code/lib/gco'))

if(~exist('unary','var'))
    
    unary = [0 9 2 0;      % Sites 1,4 prefer  label 1
        3 0 3 3;      % Site  2   prefers label 2 (strongly)
        5 9 0 5;]' ;   % Site  3   prefers label 3
    
    pairwise = [0 1 0 0;     % Sites 1 and 2 connected with weight 1
        0 0 1 0;     % Sites 2 and 3 connected with weight 1
        0 0 0 2;     % Sites 3 and 4 connected with weight 2
        0 0 0 0;];
end


[NumSites, NumLabels] = size(unary);

if(~exist('factor_int32','var')); factor_int32 = 100; end;
if(~exist('alpha','var')); alpha = 1; end;
if(~exist('expansion','var')); expansion = 1; end;


% Create Multi-Label-Graph-Cut
h = GCO_Create(NumSites,NumLabels);

% Set unary costs
GCO_SetDataCost(h,int32(unary'*factor_int32));

% Set pairwise costs (double but integral values)
GCO_SetNeighbors(h,round(pairwise*factor_int32*alpha));


% Set label costs
if(exist('smoothcost','var') && ~isempty(smoothcost))
    GCO_SetSmoothCost(h,smoothcost);
end

% Set label costs
if(exist('labelcost','var') && ~isempty(labelcost))
    GCO_SetLabelCost(h,labelcost*factor_int32);
end


if(expansion)
    t = cputime;
    GCO_Expansion(h);
    t = cputime-t;
else
    t = cputime;
    GCO_Swap(h);
    t = cputime-t;
end


labels = GCO_GetLabeling(h);

if (nargout>1)
    [E, D, S, L] = GCO_ComputeEnergy(h);
else
    D=0; S=0; L=0;
end


GCO_Delete(h);
