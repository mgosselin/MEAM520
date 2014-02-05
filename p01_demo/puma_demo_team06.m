% Demo Team06
%% Puma Simulator Demo

% Initializiation
% Close all figure windows.
close all

% Clear all global variables.  You should do this before calling any PUMA
% functions.
clear all

% Move the cursor to the top of the command window so new text is easily
% seen.
home

%% Demo 2
th1now = 0;
th2now = 0;
th3now = 0;
th4now = 0;
th5now = 0;
th6now = 0;
% Initialize the PUMA simulation to plot every 20th location of the PUMA
% and use '.' for the LED markers.  Decreasing 'PlotEveryNFrames' will
% result in smoother motions of the PUMA but will take longer to simulate.
pumaStart('Delay',10,'PlotEveryNFrames', 2, 'LEDMarkerStyle', '.');
% Turn on the LED
pumaLEDOn;

disp(' ')
disp('Demo 2: Use pumaServo for smaller motions')
disp('You may also pass in optional arguments to pumaStart to adjust the')
disp('visualization. See the documentation for a list of available options')
disp('Press ENTER to make the robot paint with light');
title('Press ENTER to make the robot paint with light');
pause

load theta_hist_xy.mat
pumaAngles % see the joint angles
% pumaMove(theta_history(:,1));

thetaStart = [0 0 0 0 -pi/2 0];
for j = 1:6
    theta_drift(j,:) = linspace(thetaStart(j), theta_history(j,1),200);
end

for i = 1:200
    pumaLEDSet(0.8, 0.5, 0.2); % set the LED color
    pumaServo(theta_drift(1,i),theta_drift(2,i),theta_drift(3,i),...
        theta_drift(4,i),theta_drift(5,i),theta_drift(6,i)); % successive joint angle commands must be less than 0.1 radians apart
end

for i = 1:length(theta_history)
    pumaLEDSet(0.8, 0.5, 0.2); % set the LED color
    pumaServo(theta_history(1,i),theta_history(2,i),theta_history(3,i),...
        theta_history(4,i),theta_history(5,i),theta_history(6,i)); % successive joint angle commands must be less than 0.1 radians apart
end
load theta_hist_yz.mat
pumaAngles % see the joint angles
for i = 1:length(theta_history)
    pumaLEDSet(0.8, 0.5, 0.2); % set the LED color
     pumaServo(theta_history(1,i),theta_history(2,i),theta_history(3,i),...
        theta_history(4,i),theta_history(5,i),theta_history(6,i)); % successive joint angle commands must be less than 0.1 radians apart
end
load theta_hist_xz.mat
pumaAngles % see the joint angles
% pumaMove(theta_history(:,1));
for i = 1:length(theta_history)
    pumaLEDSet(0.8, 0.5, 0.2); % set the LED color
     pumaServo(theta_history(1,i),theta_history(2,i),theta_history(3,i),...
        theta_history(4,i),theta_history(5,i),theta_history(6,i)); % successive joint angle commands must be less than 0.1 radians apart
end
pumaAngles % see the joint angles

disp('Press ENTER to make the robot move with the LED off');
title('Press ENTER to make the robot move with the LED off');
pause

pumaLEDOff; % turn the LED off
pumaAngles % see the joint angles
disp('The demo is done')
title('The demo is done')
