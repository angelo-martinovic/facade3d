%
%
%
%
%
%
% 
%
%
%   cx = [min(crop_xt),max(crop_xt)]; cy = [min(crop_yt),max(crop_yt)]; cz = [min(crop_zt),max(crop_zt)];
%   honza_plot_3d_cube(cx,cy,cz)
%
%

function [out p f] = honza_plot_3d_cube(cx,cy,cz,varargin)
global ADD CFG
out = []; ht = [];
p = inputParser;
p.addOptional('hf', 456423185);
p.addOptional('FaceColor', [119 114 105]/255);
p.addOptional('EdgeColor', 'k');
p.addOptional('FaceAlpha', .7);
p.addOptional('dont_plot', 0);

p.parse(varargin{:}); fldnames = fieldnames(p.Results); for a=1:length(fldnames), eval([fldnames{a},' = p.Results.',fldnames{a},';']); end;

%%


clear p f
p(:,1) = [cx(1) cy(1) cz(1)]';
p(:,2) = [cx(2) cy(1) cz(1)]';
p(:,3) = [cx(2) cy(2) cz(1)]';
p(:,4) = [cx(1) cy(2) cz(1)]';
p(:,5) = [cx(1) cy(1) cz(2)]';
p(:,6) = [cx(2) cy(1) cz(2)]';
p(:,7) = [cx(2) cy(2) cz(2)]';
p(:,8) = [cx(1) cy(2) cz(2)]';
f(:,1) = [1 2 3 4]';
f(:,2) = [1 2 6 5]';
f(:,3) = [2 6 7 3]';
f(:,4) = [3 7 8 4]';
f(:,5) = [4 8 5 1]';
f(:,6) = [5 6 7 8]';
p = p([3 2 1],:);
if ~dont_plot
    ht = patch('Faces',f','Vertices',p','FaceColor',FaceColor,'EdgeColor',EdgeColor,'FaceAlpha',FaceAlpha);
end

out = ht;


end



