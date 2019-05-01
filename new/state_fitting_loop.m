function [support_fitting_parameters_1,support_fitting_parameters_2 ...
    walk_fitting_1, walk_fitting_2,mid1,mid2] = state_fitting_loop(x,y)

%   supporting period 
%     s_end = find( x == min(x));
    s_end = length(x)/2;
    %% % fitting supporting
    xx = x(1:s_end);
    yy = y(1:s_end);
    [dx,dydx,ddx,ddy_ddx] = cal_derivative(xx,yy);
    mid1 = find(dydx == max(dydx) );
     
    figure
    hold on
    xx1 = xx(1:mid1);
    yy1 = yy(1:mid1);
    [support_fitting_parameters_1,gof1] = fit(xx1,yy1,'poly4');
%     plot(support_fitting_parameters_1,xx1,yy1);  

    xx2 = xx(mid1:end);
    yy2 = yy(mid1:end);
    [support_fitting_parameters_2,gof2] = fit(xx2,yy2,'poly4');
%     plot(support_fitting_parameters_2,xx2,yy2);

    %% % fitting walking
    xx = x(s_end:end);
    yy = y(s_end:end);
    [dx,dydx,ddx,ddy_ddx] = cal_derivative(xx,yy);
    mid2 = find(dydx == max(dydx) );
     
    xx1 = xx(1:mid2);
    yy1 = yy(1:mid2);
    [walk_fitting_1,gof1] = fit(xx1,yy1,'poly4');
%     plot(walk_fitting_1,xx1,yy1);  

    xx2 = xx(mid2:end);
    yy2 = yy(mid2:end);
    [walk_fitting_2,gof2] = fit(xx2,yy2,'poly4');
%     plot(walk_fitting_2,xx2,yy2);
%    legend('data','fitting');
end