function [x0,a] = PlaneIntersection( n1, n2, p1, p2 )

    % Assume that 2 planes are not parallel
    
    % Assume that both planes contain points for which x0(2) = 0
    
    a = cross(n1,n2);
    a = a/norm(a);
    
    % z = 0
%     y_coord = (p1*n2(1) - p2*n1(1))/(n2(2)*n1(1)-n1(2)*n2(1));
%     x_coord = (p2*n1(2) - p1*n2(2))/(n2(2)*n1(1)-n1(2)*n2(1));
%     z_coord = 0;
    
    % y = 0
    y_coord = 0;
    x_coord = (p2*n1(3) - p1*n2(3))/(n2(3)*n1(1)-n1(3)*n2(1));
    z_coord = (p1*n2(1) - p2*n1(1))/(n2(3)*n1(1)-n1(3)*n2(1));
    
    x0 = [x_coord y_coord z_coord];


end

