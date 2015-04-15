%
% check if address to the file exist and create it if not
%
%
% example:
%  checkAdr_and_createDir(~/tmp_created/ahoj.cpp);
%
% honza knopp, 2009
%



function [] = checkAdr_and_createDir(address)

idx_sl  = find(address=='/');
if isempty(idx_sl), return; end;
dir_adr = address(1:idx_sl(end));
if ~exist(dir_adr,'dir');
%     disp(['  creating dir : ',dir_adr]);
    mkdir(dir_adr);
    fileattrib(dir_adr,'+w','g');
end 

end
