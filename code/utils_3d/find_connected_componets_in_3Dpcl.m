%
%
%
%
%
%
function oindex = find_connected_componets_in_3Dpcl (pts , cprb , varargin)
p = inputParser;
p.addOptional('cl',1);
p.addOptional('max_objs',8e3);
p.addOptional('K',7);
p.addOptional('edges',[]);
p.parse(varargin{:}); fldnames = fieldnames(p.Results); for a=1:length(fldnames), eval([fldnames{a},' = p.Results.',fldnames{a},';']); end;



oindex = cprb*0;
unmet = logical(cprb*0+1);

cl_idx = cprb==cl;
%            scene.plot_opengl_scene(cl_idx');
%            mukta_plot(scene.pts([2,3],1:10:end)); axis equal;
if isempty(edges)
    [edges] = knnsearch(pts',pts','K',K);
    edges = edges';
end
for k=1:max_objs,  %%% use one point (k) and propagate unitll you see only other classes
    fprintf('\b\b\b\b\b%5i',k);
    ik = find(cl_idx,1);
    listgo = edges(:,ik)';
    oindex(ik)=k;
    while sum(cprb(listgo)==cl)>0,
        listgo = listgo(cl_idx(listgo) & unmet(listgo));
        unmet(listgo)  = 0;
        cl_idx(listgo) = 0;
        oindex(listgo) = k;
        toadd  = unique(edges(:,listgo))';
        toadd  = toadd(unmet(toadd));
        listgo = toadd( cprb(toadd)==cl);
    end
end

oindex(cprb~=cl) = 0;

end
