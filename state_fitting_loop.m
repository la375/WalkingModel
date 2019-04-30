function [support_fitting_parameters_1,support_fitting_parameters_2 ...
    walk_fitting_1, walk_fitting_2] = state_fitting_loop(x,y)

%   supporting period 
%     s_end = find( x == min(x));
    s_end = length(x)/2;
    %% % fitting supporting
    xx = x(1:s_end);
    yy = y(1:s_end);
    [dx,dydx,ddx,ddy_ddx] = cal_derivative(xx,yy);
    mid = find(dydx == max(dydx) );
     
    figure
    hold on
    xx1 = xx(1:mid);
    yy1 = yy(1:mid);
    [support_fitting_parameters_1,gof1] = fit(xx1,yy1,'poly4');
    plot(support_fitting_parameters_1,xx1,yy1);  
    legend('fitting','data');

    xx2 = xx(mid:end);
    yy2 = yy(mid:end);
    [support_fitting_parameters_2,gof2] = fit(xx2,yy2,'poly4');
    plot(support_fitting_parameters_2,xx2,yy2);
    legend('fitting','data');
    %% % fitting walking
    xx = x(s_end:end);
    yy = y(s_end:end);
    [dx,dydx,ddx,ddy_ddx] = cal_derivative(xx,yy);
    mid = find(dydx == max(dydx) );
     
    xx1 = xx(1:mid);
    yy1 = yy(1:mid);
    [walk_fitting_1,gof1] = fit(xx1,yy1,'poly4');
    plot(walk_fitting_1,xx1,yy1);  
    legend('fitting','data');

    xx2 = xx(mid:end);
    yy2 = yy(mid:end);
    [walk_fitting_2,gof2] = fit(xx2,yy2,'poly4');
    plot(walk_fitting_2,xx2,yy2);
    legend('fitting','data');

end