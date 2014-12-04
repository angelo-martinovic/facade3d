

function output_data = honza_optimize_bbox_postions(input_data , max_val , bbox)


%{
 truc_dist = @(x,max_val) (max_val.^2/6) .* (1 - (1-(x).^2).^3);
 objfun3   = @(x,max_val) sum( min([repmat(truc_dist(max_val,max_val),1,length(x)) ; truc_dist(x,max_val)]) );
 objfun3   = @(x,max_val,old)...
                          sum( min([repmat(truc_dist(max_val,max_val),1,length(x)) ; truc_dist(x,max_val)])./truc_dist(max_val,max_val) )+...
                          .0*norm(old-x);
  %}                   
                  
max_val = max_val;
% truc_dist = @(x,max_val) 1-exp(-(  (x.^2) / (2*max_val^2)));

objfun3   = @(x,max_val,old) sum(  tukey_fce(pdist(x'),max_val)  );
% x = -1:.0001:1; plot(x, tukey(x,max_val));


% input_data = [1 2 3 4 1.1 2.2 3.1 4.2]; max_val = .13;
%opts =
%optimoptions(@fminunc,'Algorithm','quasi-newton','Display','off','OutputFcn',@(x,y,z)myOutputFcn(x,y,z,bbox)); with visualizaton   
opts = optimoptions(@fminunc,'Algorithm','quasi-newton','Display','off');

output_data = fminunc(@(x) objfun3(x,max_val, input_data), input_data, opts);

if 0,
    [truc_dist(output_data,max_val) ; truc_dist(input_data,max_val)]
    norm (output_data-input_data)
end




end




function stop = myOutputFcn(x,optimValues,state)%,bbox)

% 
%  for cnt=1:length(bbox.class),   
%      honza_plot_3d_cube(bbox.corners([3 6],cnt),bbox.corners([2 5],cnt),bbox.corners([1 4],cnt),'FaceColor','r'); hold on; 
%  end;

end

