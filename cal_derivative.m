function [dx,dydx,ddx,ddy_ddx] = cal_derivative(x,y)

    
    % this is the "finite difference" derivative. Note it is  one element shorter than y and x
    dydx = diff(y)./diff(x);
    % this is to assign yd an abscissa midway between two subsequent x
    dx = (x(2:end)+x(1:(end-1)))/2;
    % this should be a rough plot of  your derivative
    ddy_ddx = diff(dydx)./diff(dx);
    ddx = (dx(2:end)+dx(1:(end-1)))/2;


    
end


