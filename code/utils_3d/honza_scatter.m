%
%  honza_scatter(pts,C*0+pts_size,C,'filled');
%
% scatter data motivated by mukta_plot
% (c) 2012

function t = honza_scatter(arr, varargin)


	if(nargin == 1)
        if( size(arr,1) == 2)
            t = scatter(arr(1,:), arr(2,:), 'b.');
		else
			t = scatter3(arr(1,:), arr(2,:), arr(3,:), 'b.');
		end
    else
        if( size(arr,1) == 2)
			t = scatter(arr(1,:), arr(2,:), varargin{:});
		else
			t = scatter3(arr(1,:), arr(2,:), arr(3,:), varargin{:});
		end
    end

    colormap('jet');

end
