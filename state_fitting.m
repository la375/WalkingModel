function [state_fitting_parameters_1,state_fitting_parameters_2 ] = state_fitting(x,y)
%     figure
%     plot(x,y)
%     xlabel(x)
%     ylabel(y)
%     title('input data')
    
    [dx,dydx,ddx,ddy_ddx] = cal_derivative(x,y);
%     figure; hold on
%     plot(dx,dydx)
%     xlabel(x)
%     ylabel('derivative of',x)
%     yyaxis right
%     plot(ddx,ddy_ddx)
    
    mid = find(dydx == max(dydx) );
     
    figure
    hold on
    xx1 = x(1:mid);
    yy1 = y(1:mid);
    [state_fitting_parameters_1,gof1] = fit(xx1,yy1,'poly3');
    plot(state_fitting_parameters_1,xx1,yy1);

    
    legend('fitting','data');

    xx2 = x(mid:end);
    yy2 = y(mid:end);
    [state_fitting_parameters_2,gof2] = fit(xx2,yy2,'poly3');
    
    plot(state_fitting_parameters_2,xx2,yy2);
    legend('fitting','data');
    
%     figure(11), hold on
%     
%     plot(xx1,yy1,'o')
%     plot(xx2,yy2,'o')
%     plot(state_fitting_parameters_1)
%     plot(state_fitting_parameters_2)

end