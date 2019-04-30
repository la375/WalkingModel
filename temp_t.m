close all
robotParametersInvKin;

%% Plot the foot trajectory
gaitPeriod = 1;
stepLength = 0.1;%input <0.3
stepHeight = 0.05;%input (<0.3
numPoints  = 150; 
footLength = 0.02;

tVec = linspace(0,gaitPeriod,numPoints);
foot_height_offset = sqrt( (lower_leg_length+upper_leg_length)^2 ...
                         - ((stepLength/2)*100)^2 ) - 1e-3;

figure(1), hold on
x = zeros(numPoints,1);
y = zeros(numPoints,1); 
for idx = 1:numPoints   
   [x(idx),y(idx)] = evalFootGait(tVec(idx),stepLength,stepHeight,gaitPeriod);
end

plot(x,y,'.-');
axis equal
title('Foot Gait');
xlabel('x [m]')
ylabel('y [m]')

%% Calculate joint angles
theta_hip = zeros(numPoints,1);
theta_knee = zeros(numPoints,1);
theta_ankle = zeros(numPoints,1);



for idx = 1:numPoints
    
    pitch = 0; % Assume zero body pitch
    
    % Calculate inverse kinematics
    theta = legInvKin(upper_leg_length/100, lower_leg_length/100 , ... 
                      x(idx), y(idx) - (foot_height_offset/100));

    % Address multiple solutions by preventing knee bending backwards
    if size(theta,1) == 2
       if theta(1,2) > 0
          t1 = theta(2,1);
          t2 = theta(2,2);
       else
          t1 = theta(1,1); 
          t2 = theta(1,2);
       end
    else
        t1 = theta(1);
        t2 = theta(2);
    end
    
    % Pack the results. Ensure the ankle angle is set so the foot always
    % lands flat on the ground
    theta_hip(idx) = t1;
    theta_knee(idx) = t2;
    theta_ankle(idx) = -(t1+t2);
    
end

%% relation between joints
hip = rad2deg(theta_hip);
knee = rad2deg(theta_knee);
ankle = rad2deg(theta_ankle);

middle = round( numPoints/2 );

step_support = [tVec(1:middle)]';
hip_s = rad2deg(theta_hip(1:middle));
knee_s = rad2deg(theta_knee(1:middle));
ankle_s = rad2deg(theta_ankle(1:middle));

step_walk= [tVec(middle+1:end)]';
hip_w = rad2deg(theta_hip(middle+1:end));
knee_w = rad2deg(theta_knee(middle+1:end));
ankle_w = rad2deg(theta_ankle(middle+1:end));

% % Display joint angles
% figure(2), hold on
% subplot(311),hold on
% plot(step_support,hip_s)
% plot(step_walk,hip_w)
% title('Hip Angle [deg]');
% subplot(312),hold on
% plot(step_support,knee_s)
% plot(step_walk,knee_w)
% title('Knee Angle [deg]');
% subplot(313),hold on
% plot(step_support,ankle_s)
% plot(step_walk,ankle_w)
% xlabel('time')
% title('Ankle Angle [deg]');
% legend('walking','stand')
% 
% figure(3), hold on
% plot(hip_s, knee_s, hip_w, knee_w)
% xlabel('hip angle')
% ylabel('knee angle')
% legend("walking","stand")
% figure(4), hold on
% plot( knee_s, ankle_s, knee_w, ankle_w)     
% legend("stand","walking")
% xlabel('knee angle')
% ylabel('ankle angle')

% curve gitting

[h2n_s1, h2n_s2,h2n_w1,h2n_w2] = state_fitting_loop(hip,knee);
[ knee_fit ] = state_fit_plot(hip, h2n_s1, h2n_s2,h2n_w1,h2n_w2 );
figure; hold on
plot(hip,knee_fit,'o')
plot(hip,knee)

[n2a_s1, n2a_s2, n2a_w1, n2a_w2] = state_fitting_loop(knee,ankle);
[ ankle_fit ] = state_fit_plot( knee_fit, n2a_s1, n2a_s2, n2a_w1, n2a_w2 );
figure; hold on
plot(knee_fit,ankle_fit,'o')
plot(knee,ankle)




 
%% Animate the walking gait
% figure, clf, hold on
% 
% % Initialize plot
% % plot(x,y-foot_height_offset/100,'k:','LineWidth',1);
% numOfPairsLegs = 1;
% h(numOfPairsLegs,6) = nan;
% for i=1:numOfPairsLegs
%     h(i,1) = plot([0 0],[0 0],'r-','LineWidth',4);
%     h(i,2) = plot([0 0],[0 0],'r-','LineWidth',4);
%     h(i,3) = plot([0 0],[0 0],'b-','LineWidth',4);
%     h(i,4) = plot([0 0],[0 0],'b-','LineWidth',4);
%     h(i,5) = plot([0 0],[0 0],'g-','LineWidth',4);
%     h(i,6) = plot([0 0],[0 0],'g-','LineWidth',4);
% end
% 
% % Calculate knee and ankle (x,y) positions
% xKnee =  sin(theta_hip)*upper_leg_length/100;
% yKnee = -cos(theta_hip)*upper_leg_length/100;
% xAnkle = xKnee + sin(theta_hip+theta_knee)*lower_leg_length/100;
% yAnkle = yKnee - cos(theta_hip+theta_knee)*lower_leg_length/100;
% 
% % Define axis limits
% xMin = min([xKnee;xAnkle]) -0.025; 
% xMax = max([xKnee;xAnkle]) +0.025;
% yMin = min([yKnee;yAnkle]) -0.025; 
% yMax = max([0;yKnee;yAnkle]) +0.025;

% % Animate the walking gait
% numAnimations = 5;
% delta = 0.1;
% set(plot([0 0],[0 0],'g-','LineWidth',2),'xdata',[-(numOfPairsLegs)*delta -delta],'ydata',[0 0]);
% for anim = 1:numAnimations
%     for idx = 1:numPoints
%         for leg = 1:numOfPairsLegs
%             index = (75+idx);
%             if index > 150
%                 index = index - 150;
%             end
%             posOffset = leg * delta;
%             if rem(leg,2) ~= 0
%                 set(h(leg,1),'xdata',[0-posOffset xKnee(index)-posOffset],'ydata',[0 yKnee(index)]);
%                 set(h(leg,2),'xdata',[xKnee(index)-posOffset xAnkle(index)-posOffset],'ydata',[yKnee(index) yAnkle(index)]);
%                 set(h(leg,3),'xdata',[0-posOffset xKnee(idx)-posOffset],'ydata',[0 yKnee(idx)]);
%                 set(h(leg,4),'xdata',[xKnee(idx)-posOffset xAnkle(idx)-posOffset],'ydata',[yKnee(idx) yAnkle(idx)]);
%                 set(h(leg,6),'xdata',[xAnkle(index)-posOffset xAnkle(index)-posOffset + footLength*cos(theta_ankle(index))],'ydata',[yAnkle(index) yAnkle(index)-footLength*sin(theta_ankle(index))]);
%                 set(h(leg,5),'xdata',[xAnkle(idx)-posOffset xAnkle(idx)-posOffset + footLength*cos(theta_ankle(idx))],'ydata',[yAnkle(idx) yAnkle(idx)-footLength*sin(theta_ankle(idx))]);
%             else
%                 set(h(leg,3),'xdata',[0-posOffset xKnee(index)-posOffset],'ydata',[0 yKnee(index)]);
%                 set(h(leg,4),'xdata',[xKnee(index)-posOffset xAnkle(index)-posOffset],'ydata',[yKnee(index) yAnkle(index)]);
%                 set(h(leg,1),'xdata',[0-posOffset xKnee(idx)-posOffset],'ydata',[0 yKnee(idx)]);
%                 set(h(leg,2),'xdata',[xKnee(idx)-posOffset xAnkle(idx)-posOffset],'ydata',[yKnee(idx) yAnkle(idx)]);
%                 set(h(leg,5),'xdata',[xAnkle(index)-posOffset xAnkle(index)-posOffset + footLength*cos(theta_ankle(index))],'ydata',[yAnkle(index) yAnkle(index)-footLength*sin(theta_ankle(index))]);
%                 set(h(leg,6),'xdata',[xAnkle(idx)-posOffset xAnkle(idx)-posOffset + footLength*cos(theta_ankle(idx))],'ydata',[yAnkle(idx) yAnkle(idx)-footLength*sin(theta_ankle(idx))]);
%             end
%         end
%         xlim([xMin xMax]), ylim([yMin yMax]);
%         title('Multi-legged Walking (Red is Right leg)');
%         axis equal
%         drawnow
%     end
% end