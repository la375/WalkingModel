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
% figure; hold on
% plot(hip,knee_fit,'o')
% plot(hip,knee)

[n2a_s1, n2a_s2, n2a_w1, n2a_w2,mid1,mid2] = state_fitting_loop(knee,ankle);
[ ankle_fit ] = state_fit_plot( knee_fit, n2a_s1, n2a_s2, n2a_w1, n2a_w2,mid1,mid2 );
% figure; hold on
% plot(knee_fit,ankle_fit,'o')
% plot(knee,ankle)


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

%% Plot fitted noisy trajectory
% noise = (rand( numPoints,1)*2-1)*max(knee_fit)*1;
knee_fit2 = zeros(numPoints,1);
max_in_knee = max(knee_fit);
er_array = zeros(1, numPoints);

%rand*max_in_knee
for i=1:numPoints
    er_array(i) = er_array(i) + rand*max_in_knee/15 ;
end

for i=1:numPoints
    total = 0;
    for j = 1:i
        total = total+er_array(j);
    end
    knee_fit2(i) = total + knee_fit(i);
end

theta_hip_1 = (hip);
theta_knee_2 =(knee_fit2);
theta_ankle_1 =(ankle_fit);
% Calculate knee and ankle (x,y) positions
xKnee =  sin(theta_hip_1)*upper_leg_length/100;
yKnee = -cos(theta_hip_1)*upper_leg_length/100;
xAnkle2 = xKnee + sin(theta_hip_1+theta_knee_2)*lower_leg_length/100;
yAnkle2 = yKnee - cos(theta_hip_1+theta_knee_2)*lower_leg_length/100;

figure(1)
plot(xAnkle2,yAnkle2,'--')

%% minus noise



% theta_hip_1 = (hip);
% theta_knee_2 =(knee_fit2);
% theta_ankle_1 =(ankle_fit);
% % Calculate knee and ankle (x,y) positions
% xKnee =  sin(theta_hip_1)*upper_leg_length/100;
% yKnee = -cos(theta_hip_1)*upper_leg_length/100;
% xAnkle3 = xKnee + sin(theta_hip_1+theta_knee_2)*lower_leg_length/100;
% yAnkle3 = yKnee - cos(theta_hip_1+theta_knee_2)*lower_leg_length/100;
yAnkle3 = yAnkle2;
for i = 2:numPoints
   yAnkle3(i) = yAnkle3(i) - mean( yAnkle3(1:i)-yAnkle(1:i));
%    yAnkle3(i) = yAnkle3(i) - ( yAnkle3(i-1)-yAnkle(i-1))*0.5;
end

figure(1)
plot(xAnkle2,yAnkle3)
legend('1','2','noise','noise adaptive')


%follow sequence
Kp = -1.5;
Ki = 0;
Kd = 0.075;
P_error = [0;0];
I_error = [0;0];
D_error = [0;0];
drive = [0;0];
curr_pos = [0;0];
prev_pos = [0;0];
error = [0;0];
prev_e = [0;0];
ideal = [xAnkle';yAnkle'];
noise = [er_array;er_array];
startpos = [stepLength/2;0];
curr_pos = startpos;
linar= zeros(2,numPoints);
linar(:,1) = startpos;
for i = 2: numPoints
    prev_e = error;
    error = curr_pos-ideal(:,i);
    D_error = error-prev_e;
    P_error = error;
    I_error = I_error + error;
    drive = Kp*P_error + Ki*I_error + Kd*D_error;
    
    prev_pos = curr_pos;
    curr_pos = prev_pos+drive+noise(:,i);
    linar(:,i) = curr_pos;
end

close all 
plot(linar(1,:), linar(2,:));
hold on;
plot(ideal(1,:), ideal(2,:));

