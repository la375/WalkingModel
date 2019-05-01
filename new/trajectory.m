close all
robotParametersInvKin;

%% Plot the preset foot trajectory 
gaitPeriod = 1;
stepLength = 0.1;%input <0.3
stepHeight = 0.025;%input (<0.3
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
plot(x,y-foot_height_offset/100)
%% Use inverse kinematics
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

hip = (theta_hip);
knee=(theta_knee);
ankle=(theta_ankle);
% curve gitting
[h2n_s1, h2n_s2,h2n_w1,h2n_w2,mid1,mid2] = state_fitting_loop(hip,knee);
[ knee_fit ] = state_fit_plot(hip, h2n_s1, h2n_s2,h2n_w1,h2n_w2,mid1,mid2 );
figure; hold on
plot(hip,knee_fit,'o')
plot(hip,knee)

[n2a_s1, n2a_s2, n2a_w1, n2a_w2,mid1,mid2] = state_fitting_loop(knee,ankle);
[ ankle_fit ] = state_fit_plot( knee_fit, n2a_s1, n2a_s2, n2a_w1, n2a_w2,mid1,mid2 );
figure; hold on
plot(knee_fit,ankle_fit,'o')
plot(knee,ankle)


%% Plot fitted trajectory

theta_hip_1 = (hip);
theta_knee_1 =(knee_fit);
theta_ankle_1 =(ankle_fit);
% Calculate knee and ankle (x,y) positions
xKnee =  sin(theta_hip_1)*upper_leg_length/100;
yKnee = -cos(theta_hip_1)*upper_leg_length/100;
xAnkle = xKnee + sin(theta_hip_1+theta_knee_1)*lower_leg_length/100;
yAnkle = yKnee - cos(theta_hip_1+theta_knee_1)*lower_leg_length/100;

figure(1)
plot(xAnkle,yAnkle)
