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

%% Light Painting Simulation
% Initialize the PUMA simulation to plot every 20th location of the PUMA
% and use '.' for the LED markers.  Decreasing 'PlotEveryNFrames' will
% result in smoother motions of the PUMA but will take longer to simulate.
load puma_light_painting_team06.mat
pumaStart('PlotEveryNFrames', 2, 'LEDMarkerStyle', '.');
% Turn on the LED

pause
theta_start = [0 0 0 0 -pi/2 0];
theta_end_last = theta_start;
pumaLEDOn;
for i = 1:length(solar.theta)
    theta_history = solar.theta{i};
    pumaLEDSet(0,0,0);
    for d = 1:6
        theta_drift(d,:) = linspace(theta_end_last(d), theta_history(d,1),157);
    end
    for l = 1:length(theta_drift)
        pumaServo(theta_drift(:,l));
    end
    led_rgb_start = solar.rgb(i).start;
    led_rgb_end = solar.rgb(i).end;
    for j = 1:3
        led_rgb(j,:) = linspace(led_rgb_start(j),led_rgb_end(j),length(theta_history));
    end
    for k = 1:2:length(theta_history)
        pumaLEDSet(led_rgb(:,k));
        pumaServo(theta_history(:,k));
    end
    theta_end_last = theta_history(:,end);
    clear led_rgb
end
% Back to zero configuration
pumaLEDSet(0,0,0);
for d = 1:6
    theta_drift(d,:) = linspace(theta_end_last(d),theta_start(d),157);
end
for l = 1:length(theta_drift)
    pumaServo(theta_drift(:,l));
end
pause

pumaLEDOff; % turn the LED off
pumaAngles % see the joint angles
disp('The demo is done')
title('The demo is done')
