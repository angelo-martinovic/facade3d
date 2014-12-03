%
%
%
%
%
%
% 
%   honza, 2014
%

function [p f] = get_box_pos_from_its_cenAndSize(cx,cy,cz)


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


end



