%% TEST_PUMA_IK_TEAM00  Tests the full inverse kinematics for the PUMA 260.
%
% This Matlab file provides the starter code for the PUMA 260 inverse
% kinematics testing script of project 1 in MEAM 520 at the University of
% Pennsylvania.  The original was written by Professor Katherine J.
% Kuchenbecker in October of 2012. Students will work in teams modify this
% code to create their own script. Post questions on the class's Piazza
% forum. 
%
% This script runs thorough tests on the inverse kinematics function the
% designated team has written for the PUMA 260.  At a minimum, it
% calculates the following two scores:
%
% score_without_thnow
% The score for the inverse kinematics solution when called without the
% current configuration of the robot (th1now ... th6now).  The ik function
% is free to pick any valid solution.  It should return NaN for all six
% joint angles if the requested configuration is not reachable or is
% outside the robot's joint limits.  The score should range from 0 (worst
% performance) to 100 (perfect performance).
%
% score_with_thnow
% The score for the inverse kinematics solution when called with the
% current configuration of the robot (th1now ... th6now).  The ik function 
% should pick the valid solution closest to the current joint angles.  The
% function should return NaN for all six joint angles if the requested
% configuration is not reachable or is outside the robot's joint limits.
% The score should range from 0 (worst performance) to 100 (perfect
% performance). 
%
% Please change the name of this file to include your team number rather
% than 00.  Also list your neam number and the full names of your three
% team members below.
%
% Team Number:
% Team Members: 

%% CLEAR
% Clear the workspace and the console to make it easier to find outputs and errors.
clear
clc

%% SET IK FUNCTION NAME
% Define the name of your the inverse kinematics function you want to test.
% The @ symbol in front of the function name creates a function handle, so
% that we can call the function indirectly.  We do this so that you can
% quickly change the IK function that you want to test, without having to
% change it in many places down below in the code.  This feature will also
% let the grader test your test code on IK that we know to work well or
% poorly, to see what score your test function assigns it.
puma_ik = @puma_ik_team06;
disp(['Testing function ' func2str(puma_ik) '.m'])

%% ROBOT PARAMETERS
% This problem is about the PUMA 260 robot, a 6-DOF manipulator.

% Define the robot's measurements.  These correspond to the diagram in the
% homework and are constant.  One inch exactly equals 0.0254 meters.
a = 13.0 * 0.0254; % meters
b =  2.5 * 0.0254; % meters
c =  8.0 * 0.0254; % meters
d =  2.5 * 0.0254; % meters
e =  8.0 * 0.0254; % meters
f =  2.5 * 0.0254; % meters

LEDVector = [0 0.125*0.0254 1.25*0.0254 1]';

%% Define the Points it Should Draw
% basically I will use the points from a circle to to test the
% inverse kinematic function
pointOnCircle = 100;
tStart = 0;
tEnd = 2*pi;
tStep = 2*pi/(pointOnCircle-1);
t = (tStart:tStep:tEnd)';

% Define the radius of the circle.
radius = .15; % meters

% Define the z-value for the plane that contains the circle.
x_offset = .15; % meters

% Define the x and y coordinates for the center of the circle.
z_center = .3; % meters
y_center = 0; % meters

% Set the desired x, y, and z positions over time given the circle parameters.
ox_history = x_offset*ones(size(t));
oy_history = y_center + radius * cos(t);
oz_history = z_center + radius * sin(t);
%define a point as reference to make the tip point to
%in the realistic situation it is a camera
Xoc = 0.5;
Yoc = 0;
Zoc = 0.3;

%% CALCULATE IK SCORE WITHOUT CURRENT CONFIGURATION
% Calculate a score that reflects the performance of your team's inverse
% kinematics function when called without th1now to th6now.  You need to
% design a test protocol you think is reasonable here.  The score should
% vary from 0 to 100 based on the level of performance, with 0 being worst
% and 100 being perfect.
dmax = sqrt((c+e)^2+(b+d)^2);
errosin1 = zeros(100,1);
errosin2 = zeros(100,1);
Draw1His = NaN(3,100);
score1 = zeros(100,1);
doWeHaveASolution =[0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0];
SolutionNotNaN =1;
th1History0 = zeros(pointOnCircle,1);
th2History0 = zeros(pointOnCircle,1);
th3History0 = zeros(pointOnCircle,1);
th4History0 = zeros(pointOnCircle,1);
th5History0 = zeros(pointOnCircle,1);
th6History0 = zeros(pointOnCircle,1);
for i = 1:length(score1)
    % Choose a random target.  You will want to change this.
    x = ox_history(i);
    y = oy_history(i);
    z = oz_history(i);
    tipIdeal = [x y z]';
    [phi, theta, psi]=test_puma_Find_EularAngle_team06(x,y,z,Xoc,Yoc,Zoc);
    directionIdeal= test_puma_Find_Direction_team06(phi, theta, psi);
    directionIdeal = directionIdeal / norm(directionIdeal); %unit direction vector
    
    % Run the puma_ik inverse kinematics function being tested.
    limitationEffector = 1;
    try
        [theta1 theta2 theta3 theta4 theta5 theta6] = puma_ik(x, y, z, phi, theta, psi);
    catch err 
%         limitationEffector = 0;
        errosin1(i)=1;
        if (strcmp(err.identifier,'MATLAB:atan2:complexArgument'))
            limitationEffector = 0;
        else
            rethrow(err);
            %display(err.identifier);
        end
    end
    score4direction = 1;
    score4position = 1;
    %[theta1 theta2 theta3 theta4 theta5 theta6] = puma_ik(x, y, z, phi, theta, psi);
    if limitationEffector ~=0
        if (~isnan(theta1))&&(~isnan(theta2))&&(~isnan(theta3))&&(~isnan(theta4))&&(~isnan(theta5))&&(~isnan(theta6))
            SolutionNotNaN=SolutionNotNaN+1;
            th1History0(SolutionNotNaN) = theta1;
            th2History0(SolutionNotNaN) = theta2;
            th3History0(SolutionNotNaN) = theta3;
            th4History0(SolutionNotNaN) = theta4;
            th5History0(SolutionNotNaN) = theta5;
            th6History0(SolutionNotNaN) = theta6;
              %     ?1 (waist) range = 290 deg , lowerlimit = -180 deg , upperlimit = 110 deg
            if theta1<(-180/180*pi)
                limitationEffector = limitationEffector/(exp(20*abs(theta1-(-180/180*pi)))); %allow a little cumulation erro
            end
            if theta1>(110/180*pi)
                limitationEffector = limitationEffector/(exp(20*abs(theta1-(110/180*pi))));
            end
        %     ?2 (shoulder) range = 315 deg , lowerlimit = -75 deg , upperlimit = 240 deg
            if theta2<(-75/180*pi)
                limitationEffector = limitationEffector/(exp(20*abs(theta2-(-75/180*pi)))); %allow a little cumulation erro
            end
            if theta2>(240/180*pi)
                limitationEffector = limitationEffector/(exp(20*abs(theta2-(-75/180*pi))));
            end
            %     ?3 (elbow) range = 295 deg , lowerlimit = -235 deg , upperlimit = 60 deg
            if theta3<(-235/180*pi)
                limitationEffector = limitationEffector/(exp(20*abs(theta3-(-235/180*pi)))); %allow a little cumulation erro
            end
            if theta3>(60/180*pi)
                limitationEffector = limitationEffector/(exp(20*abs(theta3-(60/180*pi))));
            end
        %     ?4 (wrist) range = 620 deg , lowerlimit = -580 deg , upperlimit = 40 deg
            if theta4<(-580/180*pi)
                limitationEffector = limitationEffector/(exp(20*abs(theta4-(-580/180*pi)))); %allow a little cumulation erro
            end
            if theta4>(40/180*pi)
                limitationEffector = limitationEffector/(exp(20*abs(theta4-(40/180*pi))));
            end
            %     ?5 (bend) range = 230 deg , lowerlimit = -120 deg , upperlimit = 110 deg
            if theta5<(-120/180*pi)
                limitationEffector = limitationEffector/(exp(20*abs(theta5-(-120/180*pi)))); %allow a little cumulation erro
            end
            if theta5>(110/180*pi)
                limitationEffector = limitationEffector/(exp(20*abs(theta5-(110/180*pi))));
            end
        %     ?6 (flange) range = 510 deg , lowerlimit = -215 deg , upperlimit = 295 deg 
            if theta6<(-215/180*pi)
                limitationEffector = limitationEffector/(exp(20*abs(theta6-(-215/180*pi)))); %allow a little cumulation erro
            end
            if theta6>(295/180*pi)
                limitationEffector = limitationEffector/(exp(20*abs(theta6-(295/180*pi))));
            end
            if limitationEffector<0
                limitationEffector=0;
            end
            %use the forward kinematics and theta angle to calculate the tip
            %point and director vector of end effector
            A1 = test_puma_dh_team06(0,  pi/2,  a, theta1);
            A2 = test_puma_dh_team06(c,     0, -b, theta2);
            A3 = test_puma_dh_team06(0, -pi/2, -d, theta3);
            A4 = test_puma_dh_team06(0,  pi/2,  e, theta4);
            A5 = test_puma_dh_team06(0, -pi/2,  0, theta5);
            A6 = test_puma_dh_team06(0,     0,  f, theta6);
            % Define the homogeneous representation of the origin of any frame.
            o = [0 0 0 1]';
            o5 = A1*A2*A3*A4*A5*o;
            o6 = A1*A2*A3*A4*A5*A6*o;
            otip = A1*A2*A3*A4*A5*A6*LEDVector;
            Draw1His(:,i) = otip(1:3);
            directionCal = (otip(1:3)-o6(1:3))/norm(otip(1:3)-o6(1:3));
            %evaluate the accuracy of the deriction of the end effector
            score4direction = dot(directionCal,directionIdeal);
            if score4direction<0
                score4direction=0;
            end
            tipCal = otip(1:3);
            %evaluate the accuracy 
            score4position = dot(tipCal/norm(tipCal),tipIdeal/norm(tipIdeal));
            if score4position<0
                score4position=0;
            end  
        else
            if doWeHaveASolution(i)==1
                limitationEffector = 1;
            else
                limitationEffector = 0;
            end
            
        end
    end
    
    % Inspect the solution and update the score.
    % For now run a dumb test on whether theta1 is greater than theta2
    score1(i) = limitationEffector*score4direction*score4position;
end
% Calculate the total score.
score_without_thnow = 100 * sum(score1) / length(score1);
disp(['Score without current configuration = ' num2str(score_without_thnow)])
if (score_without_thnow == 100)
    disp(' Good job!')
end

%% CALCULATE IK SCORE WITH CURRENT CONFIGURATION
% Calculate a score that reflects the performance of your team's inverse
% kinematics function when called with th1now to th6now.  You need to
% design a test protocol you think is reasonable here.  The score should
% vary from 0 to 100 based on the level of performance, with 0 being worst
% and 100 being perfect.

score2 = zeros(100,1);
th1History = zeros(pointOnCircle,1);
th2History = zeros(pointOnCircle,1);
th3History = zeros(pointOnCircle,1);
th4History = zeros(pointOnCircle,1);
th5History = zeros(pointOnCircle,1);
th6History = zeros(pointOnCircle,1);
differenceTh1His = zeros(100,1);
differenceTh2His = zeros(100,1);
differenceTh3His = zeros(100,1);
differenceTh4His = zeros(100,1);
differenceTh5His = zeros(100,1);
differenceTh6His = zeros(100,1);
SolutionNotNaN =1;
Draw2His = NaN(3,100);

SensitiveOfDisContin = 0.618;

for i = 1:length(score2)
    % Choose a random target.  You will want to change this.
    x = ox_history(i);
    y = oy_history(i);
    z = oz_history(i);
    tipIdeal = [x y z]';
    [phi, theta, psi]=test_puma_Find_EularAngle_team06(x,y,z,Xoc,Yoc,Zoc);
    directionIdeal= test_puma_Find_Direction_team06(phi, theta, psi);
    directionIdeal = directionIdeal / norm(directionIdeal); %unit direction vector
    
    
    % Choose random joint angles to start from.
    th1now = th1History(SolutionNotNaN);
    th2now = th2History(SolutionNotNaN);
    th3now = th3History(SolutionNotNaN);
    th4now = th4History(SolutionNotNaN);
    th5now = th5History(SolutionNotNaN);
    th6now = th6History(SolutionNotNaN);
    
    % Run the puma_ik inverse kinematics function being tested.
    limitationEffector = 1;
    try
        [theta1 theta2 theta3 theta4 theta5 theta6] = puma_ik(x, y, z, phi, theta, psi, th1now, th2now, th3now, th4now, th5now, th6now);
    catch err 
        %limitationEffector = 0;
        errosin1(i)=1;
        if (strcmp(err.identifier,'MATLAB:atan2:complexArgument'))
            limitationEffector = 0;
        else
            rethrow(err);
            %display(err.identifier);
        end
    end
    score4direction = 1;
    score4position = 1;
    score4smooth = 1;
    %[theta1 theta2 theta3 theta4 theta5 theta6] = puma_ik(x, y, z, phi, theta, psi, th1now, th2now, th3now, th4now, th5now, th6now);
    
    if limitationEffector ~=0
        if (~isnan(theta1))&&(~isnan(theta2))&&(~isnan(theta3))&&(~isnan(theta4))&&(~isnan(theta5))&&(~isnan(theta6))
            SolutionNotNaN=SolutionNotNaN+1;
            th1History(SolutionNotNaN) = theta1;
            th2History(SolutionNotNaN) = theta2;
            th3History(SolutionNotNaN) = theta3;
            th4History(SolutionNotNaN) = theta4;
            th5History(SolutionNotNaN) = theta5;
            th6History(SolutionNotNaN) = theta6;
            %     ?1 (waist) range = 290 deg , lowerlimit = -180 deg , upperlimit = 110 deg
            if theta1<(-180/180*pi)
                limitationEffector = limitationEffector/(exp(20*abs(theta1-(-180/180*pi)))); %allow a little cumulation erro
            end
            if theta1>(110/180*pi)
                limitationEffector = limitationEffector/(exp(20*abs(theta1-(110/180*pi))));
            end
        %     ?2 (shoulder) range = 315 deg , lowerlimit = -75 deg , upperlimit = 240 deg
            if theta2<(-75/180*pi)
                limitationEffector = limitationEffector/(exp(20*abs(theta2-(-75/180*pi)))); %allow a little cumulation erro
            end
            if theta2>(240/180*pi)
                limitationEffector = limitationEffector/(exp(20*abs(theta2-(-75/180*pi))));
            end
            %     ?3 (elbow) range = 295 deg , lowerlimit = -235 deg , upperlimit = 60 deg
            if theta3<(-235/180*pi)
                limitationEffector = limitationEffector/(exp(20*abs(theta3-(-235/180*pi)))); %allow a little cumulation erro
            end
            if theta3>(60/180*pi)
                limitationEffector = limitationEffector/(exp(20*abs(theta3-(60/180*pi))));
            end
        %     ?4 (wrist) range = 620 deg , lowerlimit = -580 deg , upperlimit = 40 deg
            if theta4<(-580/180*pi)
                limitationEffector = limitationEffector/(exp(20*abs(theta4-(-580/180*pi)))); %allow a little cumulation erro
            end
            if theta4>(40/180*pi)
                limitationEffector = limitationEffector/(exp(20*abs(theta4-(40/180*pi))));
            end
            %     ?5 (bend) range = 230 deg , lowerlimit = -120 deg , upperlimit = 110 deg
            if theta5<(-120/180*pi)
                limitationEffector = limitationEffector/(exp(20*abs(theta5-(-120/180*pi)))); %allow a little cumulation erro
            end
            if theta5>(110/180*pi)
                limitationEffector = limitationEffector/(exp(20*abs(theta5-(110/180*pi))));
            end
        %     ?6 (flange) range = 510 deg , lowerlimit = -215 deg , upperlimit = 295 deg 
            if theta6<(-215/180*pi)
                limitationEffector = limitationEffector/(exp(20*abs(theta6-(-215/180*pi)))); %allow a little cumulation erro
            end
            if theta6>(295/180*pi)
                limitationEffector = limitationEffector/(exp(20*abs(theta6-(295/180*pi))));
            end
            if limitationEffector<0
                limitationEffector=0;
            end
            %use the forward kinematics and theta angle to calculate the tip
            %point and director vector of end effector
            A1 = test_puma_dh_team06(0,  pi/2,  a, theta1);
            A2 = test_puma_dh_team06(c,     0, -b, theta2);
            A3 = test_puma_dh_team06(0, -pi/2, -d, theta3);
            A4 = test_puma_dh_team06(0,  pi/2,  e, theta4);
            A5 = test_puma_dh_team06(0, -pi/2,  0, theta5);
            A6 = test_puma_dh_team06(0,     0,  f, theta6);
            % Define the homogeneous representation of the origin of any frame.
            o = [0 0 0 1]';
            o5 = A1*A2*A3*A4*A5*o;
            o6 = A1*A2*A3*A4*A5*A6*o;
            otip = A1*A2*A3*A4*A5*A6*LEDVector;
            Draw2His(:,i) = otip(1:3);
            directionCal = (otip(1:3)-o6(1:3))/norm(otip(1:3)-o6(1:3));
            %evaluate the accuracy of the deriction of the end effector
            score4direction = dot(directionCal,directionIdeal);
            if score4direction<0
                score4direction=0;
            end
            tipCal = otip(1:3);
            %evaluate the accuracy 
            score4position = dot(tipCal/norm(tipCal),tipIdeal/norm(tipIdeal));
            if score4position<0
                score4position=0;
            end
            %use the displacement, velocity and accelerattion continuity to
            %judge wether the solution is a good one, which also means it
            %is unpossible to get a score of 100
            if SolutionNotNaN<3
                if SolutionNotNaN>1
                     differenceTh1His(SolutionNotNaN)= th1History(SolutionNotNaN)-th1History(SolutionNotNaN-1);
                     differenceTh2His(SolutionNotNaN)= th2History(SolutionNotNaN)-th2History(SolutionNotNaN-1);
                     differenceTh3His(SolutionNotNaN)= th3History(SolutionNotNaN)-th3History(SolutionNotNaN-1);
                     differenceTh4His(SolutionNotNaN)= th4History(SolutionNotNaN)-th4History(SolutionNotNaN-1);
                     differenceTh5His(SolutionNotNaN)= th5History(SolutionNotNaN)-th5History(SolutionNotNaN-1);
                     differenceTh6His(SolutionNotNaN)= th6History(SolutionNotNaN)-th6History(SolutionNotNaN-1);
                end
            else
                differenceTh1His(SolutionNotNaN)= th1History(SolutionNotNaN)-th1History(SolutionNotNaN-1);
                differenceTh2His(SolutionNotNaN)= th2History(SolutionNotNaN)-th2History(SolutionNotNaN-1);
                differenceTh3His(SolutionNotNaN)= th3History(SolutionNotNaN)-th3History(SolutionNotNaN-1);
                differenceTh4His(SolutionNotNaN)= th4History(SolutionNotNaN)-th4History(SolutionNotNaN-1);
                differenceTh5His(SolutionNotNaN)= th5History(SolutionNotNaN)-th5History(SolutionNotNaN-1);
                differenceTh6His(SolutionNotNaN)= th6History(SolutionNotNaN)-th6History(SolutionNotNaN-1);
                if abs(differenceTh1His(SolutionNotNaN)-differenceTh1His(SolutionNotNaN-1))>differenceTh1His(SolutionNotNaN-1)
                    tmp0=abs(differenceTh1His(SolutionNotNaN)-differenceTh1His(SolutionNotNaN-1))-differenceTh1His(SolutionNotNaN-1);
                    score4smooth=score4smooth/(exp(SensitiveOfDisContin*tmp0));
                end
                if abs(differenceTh2His(SolutionNotNaN)-differenceTh2His(SolutionNotNaN-1))>differenceTh2His(SolutionNotNaN-1)
                    tmp0=abs(differenceTh2His(SolutionNotNaN)-differenceTh2His(SolutionNotNaN-1))-differenceTh2His(SolutionNotNaN-1);
                    score4smooth=score4smooth/(exp(SensitiveOfDisContin*tmp0));
                end
                if abs(differenceTh3His(SolutionNotNaN)-differenceTh3His(SolutionNotNaN-1))>differenceTh3His(SolutionNotNaN-1)
                    tmp0=abs(differenceTh3His(SolutionNotNaN)-differenceTh3His(SolutionNotNaN-1))-differenceTh3His(SolutionNotNaN-1);
                    score4smooth=score4smooth/(exp(SensitiveOfDisContin*tmp0));
                end
                if abs(differenceTh4His(SolutionNotNaN)-differenceTh4His(SolutionNotNaN-1))>differenceTh4His(SolutionNotNaN-1)
                    tmp0=abs(differenceTh4His(SolutionNotNaN)-differenceTh4His(SolutionNotNaN-1))-differenceTh4His(SolutionNotNaN-1);
                    score4smooth=score4smooth/(exp(SensitiveOfDisContin*tmp0));
                end
                if abs(differenceTh5His(SolutionNotNaN)-differenceTh5His(SolutionNotNaN-1))>differenceTh5His(SolutionNotNaN-1)
                    tmp0=abs(differenceTh5His(SolutionNotNaN)-differenceTh5His(SolutionNotNaN-1))-differenceTh5His(SolutionNotNaN-1);
                    score4smooth=score4smooth/(exp(SensitiveOfDisContin*tmp0));
                end
                if abs(differenceTh6His(SolutionNotNaN)-differenceTh6His(SolutionNotNaN-1))>differenceTh6His(SolutionNotNaN-1)
                    tmp0=abs(differenceTh6His(SolutionNotNaN)-differenceTh6His(SolutionNotNaN-1))-differenceTh6His(SolutionNotNaN-1);
                    score4smooth=score4smooth/(exp(SensitiveOfDisContin*tmp0));
                end
            end
            if score4smooth<0
                score4smooth=0;
            end
        else
            if doWeHaveASolution(i)==1
                limitationEffector = 1;
            else
                limitationEffector = 0;
            end
        end
    end
    % Inspect the solution and update the score.
    % For now run a dumb test on whether theta3 is greater than theta4
    score2(i) = limitationEffector*score4direction*score4position*score4smooth;
end
% Calculate the total score.
score_with_thnow = 100 * sum(score2) / length(score2);
disp(['Score with current configuration = ' num2str(score_with_thnow)])
if (score_with_thnow == 100)
    disp(' Good job!')
end
fg1=figure(1);
set(fg1, 'position', get(0,'ScreenSize'));
subplot(3,2,1);
plot(1:SolutionNotNaN,110/180*pi*ones(SolutionNotNaN,1),'m');
hold on
plot(1:SolutionNotNaN,th1History0(1:SolutionNotNaN),'g');
plot(1:SolutionNotNaN,th1History(1:SolutionNotNaN),'b');
plot(1:SolutionNotNaN,differenceTh1His(1:SolutionNotNaN),'r');
plot(1:SolutionNotNaN,-180/180*pi*ones(SolutionNotNaN,1),'m');
legend('limitation of theta','theta1 with 6 input','theta1 with 12 input','velocity of theta1 with 12 input');
title('theta1  history');
grid on
subplot(3,2,2);
plot(1:SolutionNotNaN,240/180*pi*ones(SolutionNotNaN,1),'m');
hold on
plot(1:SolutionNotNaN,th2History0(1:SolutionNotNaN),'g');
plot(1:SolutionNotNaN,th2History(1:SolutionNotNaN),'b');
plot(1:SolutionNotNaN,differenceTh2His(1:SolutionNotNaN),'r');
plot(1:SolutionNotNaN,-75/180*pi*ones(SolutionNotNaN,1),'m');
legend('limitation of theta','theta2 with 6 input','theta2 with 12 input','velocity of theta2 with 12 input');
title('theta2  history');
grid on
subplot(3,2,3);
plot(1:SolutionNotNaN,60/180*pi*ones(SolutionNotNaN,1),'m');
hold on
plot(1:SolutionNotNaN,th3History0(1:SolutionNotNaN),'g');
plot(1:SolutionNotNaN,th3History(1:SolutionNotNaN),'b');
plot(1:SolutionNotNaN,differenceTh3His(1:SolutionNotNaN),'r');
plot(1:SolutionNotNaN,-235/180*pi*ones(SolutionNotNaN,1),'m');
legend('limitation of theta','theta3 with 6 input','theta3 with 12 input','velocity of theta3 with 12 input');
title('theta3  history');
grid on
subplot(3,2,4);
plot(1:SolutionNotNaN,40/180*pi*ones(SolutionNotNaN,1),'m');
hold on
plot(1:SolutionNotNaN,th4History0(1:SolutionNotNaN),'g');
plot(1:SolutionNotNaN,th4History(1:SolutionNotNaN),'b');
plot(1:SolutionNotNaN,differenceTh4His(1:SolutionNotNaN),'r');
plot(1:SolutionNotNaN,-580/180*pi*ones(SolutionNotNaN,1),'m');
legend('limitation of theta','theta4 with 6 input','theta4 with 12 input','velocity of theta4 with 12 input');
title('theta4  history');
grid on
subplot(3,2,5);
plot(1:SolutionNotNaN,110/180*pi*ones(SolutionNotNaN,1),'m');
hold on
plot(1:SolutionNotNaN,th5History0(1:SolutionNotNaN),'g');
plot(1:SolutionNotNaN,th5History(1:SolutionNotNaN),'b');
plot(1:SolutionNotNaN,differenceTh5His(1:SolutionNotNaN),'r');
plot(1:SolutionNotNaN,-120/180*pi*ones(SolutionNotNaN,1),'m');
legend('limitation of theta','theta5 with 6 input','theta5 with 12 input','velocity of theta5 with 12 input');
title('theta5  history');
grid on
subplot(3,2,6);
plot(1:SolutionNotNaN,295/180*pi*ones(SolutionNotNaN,1),'m');
hold on
plot(1:SolutionNotNaN,th6History0(1:SolutionNotNaN),'g');
plot(1:SolutionNotNaN,th6History(1:SolutionNotNaN),'b');
plot(1:SolutionNotNaN,differenceTh6His(1:SolutionNotNaN),'r');
plot(1:SolutionNotNaN,-215/180*pi*ones(SolutionNotNaN,1),'m');
legend('limitation of theta','theta6 with 6 input','theta6 with 12 input','velocity of theta6 with 12 input');
title('theta6  history');
grid on
hold off
    
for i = 1:pointOnCircle
    if i==1
        % Open figure 1.
        figure(2);
        
        % Plot the pattern we set to make the robot draw
        subplot(2,2,1);
        IdealTrace = plot3(ox_history(i),oy_history(i),oz_history(i),'b.');
        
        title('The input points');
        % Label the axes.
        xlabel('X (meter.)');
        ylabel('Y (meter.)');
        zlabel('Z (meter.)');
        
        grid on;
        box on;
        % Set the axis limits.
        axis([-20*0.0254 20*0.0254 -20*0.0254 20*0.0254 0 40*0.0254])

        % Set the axis properties for 3D visualization, which makes one
        % unit the same in every direction, and enables rotation.
        axis vis3d;
        
        subplot(2,2,2);
        PointWthoutCon = plot3(Draw1His(1,i),Draw1His(2,i),Draw1His(3,i),'r.');
        
        title('The solution point with 6 input');
        % Label the axes.
        xlabel('X (meter.)');
        ylabel('Y (meter.)');
        zlabel('Z (meter.)');
        
        grid on;
        box on;
        % Set the axis limits.
        axis([-20*0.0254 20*0.0254 -20*0.0254 20*0.0254 0 40*0.0254])

        % Set the axis properties for 3D visualization, which makes one
        % unit the same in every direction, and enables rotation.
        axis vis3d;
        
        text(-10*0.0254,10*0.0254,0,sprintf('Score without current configuration: %.4f',score_without_thnow),'horizontalAlignment','center');
        
        subplot(2,2,3);
        PointWthCon = plot3(Draw2His(1,i),Draw2His(2,i),Draw2His(3,i),'g.');
        
        title('The solution point with 12 input');
        % Label the axes.
        xlabel('X (meter.)');
        ylabel('Y (meter.)');
        zlabel('Z (meter.)');
        
        grid on;
        box on;
        % Set the axis limits.
        axis([-20*0.0254 20*0.0254 -20*0.0254 20*0.0254 0 40*0.0254])

        % Set the axis properties for 3D visualization, which makes one
        % unit the same in every direction, and enables rotation.
        axis vis3d;
        
        text(-10*0.0254,10*0.0254,-2*0.0254,sprintf('Score with current configuration: %.4f',score_with_thnow),'horizontalAlignment','center');
        
        subplot(2,2,4);
        IdealTrace4 = plot3(ox_history(i),oy_history(i),oz_history(i),'b.');
        hold on
        PointWthoutCon4 = plot3(Draw1His(1,i),Draw1His(2,i),Draw1His(3,i),'r.');
        PointWthCon4 = plot3(Draw2His(1,i),Draw2His(2,i),Draw2His(3,i),'g.');
        plot3(Xoc,Yoc,Zoc,'y.','markersize',25);
        hold off
        %legend('The input points','The solution point with 6 input','The solution point with 12 input');
        % Label the axes.
        xlabel('X (meter.)');
        ylabel('Y (meter.)');
        zlabel('Z (meter.)');

        % Turn on the grid and the box.
        grid on;
        box on;
        % Set the axis limits.
        axis([-20*0.0254 20*0.0254 -20*0.0254 20*0.0254 0 40*0.0254])

        % Set the axis properties for 3D visualization, which makes one
        % unit the same in every direction, and enables rotation.
        axis vis3d;
        
        % Put text on the plot to show how much time has elapsed.  The text
        % is centered.
        text(Xoc-0.01,Yoc-0.01,Zoc-0.01,sprintf('Camera'),'horizontalAlignment','center');
        ScoreForConfig1 = text(-10*0.0254,10*0.0254,0,sprintf('Score without current configuration: %.4f',score_without_thnow),'horizontalAlignment','center');
        ScoreForConfig2 = text(-10*0.0254,10*0.0254,-2*0.0254,sprintf('Score with current configuration: %.4f',score_with_thnow),'horizontalAlignment','center');
        
        % Add a title including the student's name.
        title('PUMA 360 Robot Test Code by Team06'); 
           
    else
        % Once the animation has been set up, we don't need to reformat the
        % whole plot.  We just set the data to the correct new values for
        % the robot animation, the tip history, and the text showing the
        % elapsed time.
        set(IdealTrace,'xdata',ox_history(1:i),'ydata',oy_history(1:i),'zdata',oz_history(1:i))
        set(PointWthoutCon,'xdata',Draw1His(1,1:i),'ydata',Draw1His(2,1:i),'zdata',Draw1His(3,1:i))
        set(PointWthCon,'xdata',Draw2His(1,1:i),'ydata',Draw2His(2,1:i),'zdata',Draw2His(3,1:i))
        set(IdealTrace4,'xdata',ox_history(1:i),'ydata',oy_history(1:i),'zdata',oz_history(1:i))
        set(PointWthoutCon4,'xdata',Draw1His(1,1:i),'ydata',Draw1His(2,1:i),'zdata',Draw1His(3,1:i))
        set(PointWthCon4,'xdata',Draw2His(1,1:i),'ydata',Draw2His(2,1:i),'zdata',Draw2His(3,1:i))
        %set(ScoreForConfig1,'string', (sprintf('Score without current configuration: %.4f',score_without_thnow)));
        %set(ScoreForConfig2,'string', (sprintf('Score with current configuration: %.4f',score_with_thnow)));
    end
end
