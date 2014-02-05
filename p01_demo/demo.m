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


%% Demo 1

% Open figure 1 and clear it.
figure(1); clf

% Initialize the PUMA simulation
pumaStart;

% turn on the LED
pumaLEDOn;

disp('Demo 1: Use pumaMove to make the arm jump to any desired location')
disp('Note, this works only in simulation')
disp('Press ENTER to start');
title('Press ENTER to start');
pause;

pumaLEDSet(1,0,0); % set the LED to red
pumaMove(0, 0, 0, 0, 0, 0);
disp('Press ENTER to move the robot');
title('Press ENTER to move the robot');
pause;

pumaLEDSet(0,0,1); % set the LED to blue
pumaMove(0, pi/2, -pi/2, 0, 0, 0);
disp('Press ENTER to move the robot again');
title('Press ENTER to move the robot again');
pause;

pumaLEDSet(0,1,0); % set the LED to green
pumaMove(0, -pi/4, pi/4, 0, pi/2, 0);
disp('Press ENTER to continue');
title('Press ENTER to continue');
pause;

pumaLEDOff;
pumaStop;

%% Demo 2

% Initialize the PUMA simulation to plot every 20th location of the PUMA
% and use '.' for the LED markers.  Decreasing 'PlotEveryNFrames' will
% result in smoother motions of the PUMA but will take longer to simulate.
pumaStart('PlotEveryNFrames', 20, 'LEDMarkerStyle', '.');
% Turn on the LED
pumaLEDOn;

disp(' ')
disp('Demo 2: Use pumaServo for smaller motions')
disp('You may also pass in optional arguments to pumaStart to adjust the')
disp('visualization. See the documentation for a list of available options')
disp('Press ENTER to make the robot paint with light');
title('Press ENTER to make the robot paint with light');
pause

pumaAngles % see the joint angles
for k = 0:0.01:2
    pumaLEDSet(k/2, 0, k/2); % set the LED color
    pumaServo(-k, 0, -k, -k, -pi/2, 0); % successive joint angle commands must be less than 0.1 radians apart
end
pumaAngles % see the joint angles

disp('Press ENTER to make the robot move with the LED off');
title('Press ENTER to make the robot move with the LED off');
pause

pumaLEDOff; % turn the LED off
for k = 0:0.005:1
    pumaServo(-2, k, -2, -2, -pi/2, k); % successive joint angle commands must be less than 0.1 radians apart
end
pumaAngles % see the joint angles
disp('The demo is done')
title('The demo is done')
