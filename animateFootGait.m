close all
% robotParametersInvKin;

%% Plot the foot trajectory
gaitPeriod = 1;
stepLength = 0.1;
stepHeight = 0.025;
numPoints  = 150; 
footLength = 0.02;

tVec = linspace(0,gaitPeriod,numPoints);
foot_height_offset = sqrt( (lower_leg_length+upper_leg_length)^2 ...
                         - ((stepLength/2)*100)^2 ) - 1e-3;

% figure, hold on
x = zeros(numPoints,1);
y = zeros(numPoints,1); 
for idx = 1:numPoints   
   [x(idx),y(idx)] = evalFootGait(tVec(idx),stepLength,stepHeight,gaitPeriod);
end

% plot(x,y,'.-');
% axis equal
% title('Foot Gait');
% xlabel('x [m]')
% ylabel('y [m]')

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

% Display joint angles
% figure
% subplot(311)
% plot(tVec,rad2deg(theta_hip))
% title('Hip Angle [deg]');
% subplot(312)
% plot(tVec,rad2deg(theta_knee))
% title('Knee Angle [deg]');
% subplot(313)
% plot(tVec,rad2deg(theta_ankle))
% title('Ankle Angle [deg]');
                     
%% Animate the walking gait
figure(3), clf, hold on

% Initialize plot
% plot(x,y-foot_height_offset/100,'k:','LineWidth',1);
numOfPairsLegs = 1;
h(numOfPairsLegs,6) = nan;
for i=1:numOfPairsLegs
    h(i,1) = plot([0 0],[0 0],'r-','LineWidth',4);
    h(i,2) = plot([0 0],[0 0],'r-','LineWidth',4);
    h(i,3) = plot([0 0],[0 0],'b-','LineWidth',4);
    h(i,4) = plot([0 0],[0 0],'b-','LineWidth',4);
    h(i,5) = plot([0 0],[0 0],'g-','LineWidth',4);
    h(i,6) = plot([0 0],[0 0],'g-','LineWidth',4);
end

% Calculate knee and ankle (x,y) positions
xKnee =  sin(theta_hip)*upper_leg_length/100;
yKnee = -cos(theta_hip)*upper_leg_length/100;
xAnkle = xKnee + sin(theta_hip+theta_knee)*lower_leg_length/100;
yAnkle = yKnee - cos(theta_hip+theta_knee)*lower_leg_length/100;

% Define axis limits
xMin = min([xKnee;xAnkle]) -0.025; 
xMax = max([xKnee;xAnkle]) +0.025;
yMin = min([yKnee;yAnkle]) -0.025; 
yMax = max([0;yKnee;yAnkle]) +0.025;

% Animate the walking gait
numAnimations = 5;
delta = 0.1;
set(plot([0 0],[0 0],'g-','LineWidth',2),'xdata',[-(numOfPairsLegs)*delta -delta],'ydata',[0 0]);
for anim = 1:numAnimations
    for idx = 1:numPoints
        for leg = 1:numOfPairsLegs
            index = (75+idx);
            if index > 150
                index = index - 150;
            end
            posOffset = leg * delta;
            if rem(leg,2) ~= 0
                set(h(leg,1),'xdata',[0-posOffset xKnee(index)-posOffset],'ydata',[0 yKnee(index)]);
                set(h(leg,2),'xdata',[xKnee(index)-posOffset xAnkle(index)-posOffset],'ydata',[yKnee(index) yAnkle(index)]);
                set(h(leg,3),'xdata',[0-posOffset xKnee(idx)-posOffset],'ydata',[0 yKnee(idx)]);
                set(h(leg,4),'xdata',[xKnee(idx)-posOffset xAnkle(idx)-posOffset],'ydata',[yKnee(idx) yAnkle(idx)]);
                set(h(leg,6),'xdata',[xAnkle(index)-posOffset xAnkle(index)-posOffset + footLength*cos(theta_ankle(index))],'ydata',[yAnkle(index) yAnkle(index)-footLength*sin(theta_ankle(index))]);
                set(h(leg,5),'xdata',[xAnkle(idx)-posOffset xAnkle(idx)-posOffset + footLength*cos(theta_ankle(idx))],'ydata',[yAnkle(idx) yAnkle(idx)-footLength*sin(theta_ankle(idx))]);
            else
                set(h(leg,3),'xdata',[0-posOffset xKnee(index)-posOffset],'ydata',[0 yKnee(index)]);
                set(h(leg,4),'xdata',[xKnee(index)-posOffset xAnkle(index)-posOffset],'ydata',[yKnee(index) yAnkle(index)]);
                set(h(leg,1),'xdata',[0-posOffset xKnee(idx)-posOffset],'ydata',[0 yKnee(idx)]);
                set(h(leg,2),'xdata',[xKnee(idx)-posOffset xAnkle(idx)-posOffset],'ydata',[yKnee(idx) yAnkle(idx)]);
                set(h(leg,5),'xdata',[xAnkle(index)-posOffset xAnkle(index)-posOffset + footLength*cos(theta_ankle(index))],'ydata',[yAnkle(index) yAnkle(index)-footLength*sin(theta_ankle(index))]);
                set(h(leg,6),'xdata',[xAnkle(idx)-posOffset xAnkle(idx)-posOffset + footLength*cos(theta_ankle(idx))],'ydata',[yAnkle(idx) yAnkle(idx)-footLength*sin(theta_ankle(idx))]);
            end
        end
        xlim([xMin xMax]), ylim([yMin yMax]);
        title('Multi-legged Walking (Red is Right leg)');
        axis equal
        drawnow
    end
end