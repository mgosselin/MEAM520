%% puma_robot_starter.m
% 
% This Matlab script provides the starter code for the PUMA 260 robot
% problem on Homework 3 in MEAM 520 at the University of Pennsylvania.
% The original was written by Professor Katherine J. Kuchenbecker in
% September of 2012. Students will modify this code to create their own
% script. Post questions on the class's Piazza forum.
% 
% Change the name of this file to replace "starter" with your PennKey.

%% SETUP

% Clear all variables from the workspace.
clear all

% Clear the console, so you can more easily find any errors that may occur.
clc

% Input your name as a string.
student_name = 'Team06';

% Define our time vector.
tStart = 0;   % The time at which the simulation starts, in seconds.
tStep = 0.04; % The simulation's time step, in seconds.
tEnd = 4*pi;     % The time at which the simulation ends, in seconds.
t = (tStart:tStep:tEnd)';  % The time vector (a column vector).

% Set whether to animate the robot's movement and how much to slow it down.
pause on;  % Set this to off if you don't want to watch the animation.
GraphingTimeDelay = 0.001; % The length of time that Matlab should pause between positions when graphing, if at all, in seconds.

%% DEFINE CIRCULAR MOTION

% We want the SCARA to draw a vertical circle parallel to the x-z plane.

% Define the radius of the circle.
radius = 0.1; % meters

% Define the y-value for the plane that contains the circle.
y_offset = 0.2; % meters

% Define the x and z coordinates for the center of the circle.
x_center = 0.3; % meters
z_center = 0.3; % meters

% Set the desired x, y, and z positions over time given the circle parameters.
ox_history = x_center + radius * sin(t);
oy_history = y_offset * ones(size(t));
oz_history = z_center + radius * cos(t);

%% ROBOT PARAMETERS

% This problem is about the PUMA 260 robot, a 6-DOF manipulator.

% Define the robot's measurements.  These correspond to the diagram in the
% homework and are constant.
a = 13.0 * 0.0254; % meters
b =  3.5 * 0.0254; % meters
c =  8.0 * 0.0254; % meters
d =  3.0 * 0.0254; % meters
e =  8.0 * 0.0254; % meters
f =  2.5 * 0.0254; % meters

phi = 0;
theta = pi/2;
psi = 0;

%% SIMULATION

% Notify the user that we're starting the animation.
disp('Starting the animation.')

% Show a message to explain how to cancel the simulation and graphing.
disp('Click in this window and press control-c to stop the code.')

% Initialize a matrix to hold the position of the robot's tip over time.
% The first row is the x-coordinate, second is y, and third is z, all in
% the base frame. It has the same number of columns as t has rows, one tip
% position for every time step in the simulation.  We keep track of this
% history so we can trace out the trajectory of where the robot's tip has been.
tip_history = zeros(3,length(t));

% Step through the time vector to animate the robot.
for i = 1:length(t)
    
    % Pull the current values of the six joint angles from their histories.
    x = ox_history(i);
    y = oy_history(i);
    z = oz_history(i);
    % Do your calculations, using your dh function as needed.
    % All of your code should be between the two lines of stars.  
    % *******************************************************************
    [theta1, theta2, theta3, theta4, theta5, theta6] = ...
        puma_ik_team06(x, y, z, phi, theta, psi);
    % Calculate the six A matrices using DH function.
    A1 = puma_dh_team06(0,  pi/2, a, theta1);
    A2 = puma_dh_team06(c,  0, -b, theta2);
    A3 = puma_dh_team06(0,  -pi/2, -d, theta3);
    A4 = puma_dh_team06(0,  pi/2, e, theta4);
    A5 = puma_dh_team06(0,  -pi/2, 0, theta5);
    A6 = puma_dh_team06(0,  0, f, theta6);
    
    % Define the homogeneous representation of the origin of any frame.
    o = [0 0 0 1]';

    % Calculate the position of the origin of each frame in the base frame.
    o0 = o;
    o1 = A1*o;
    o2 = A1*A2*o;
    o3 = A1*A2*A3*o;
    o4 = A1*A2*A3*A4*o;
    o5 = A1*A2*A3*A4*A5*o;
    o6 = A1*A2*A3*A4*A5*A6*o;
    
    % Put the seven origin points together for plotting.
    % For now we are just plotting the origin of the base frame seven times.
    points_to_plot = [o0 o1 o2 o3 o4 o5 o6];
    
    % *******************************************************************
    % All your calculations should be done by here.  You should have
    % calculated all six of the A matrices (A1 through A6) and put the
    % seven frame origins into points_to_plot in order, so that the last
    % one is the tip of the robot (frame 6).

    % Grab the final plotted point for the trajectory graph.
    tip_history(:,i) = points_to_plot(1:3,end);

    % Check if this is the first time we are plotting.
    if (i == 1)
        % Open figure 1.
        figure(1);
        
        % The first time, plot the robot points and keep a handle to the plot.
        % This is a 3D plot with dots at the points and lines connecting
        % neighboring points, made thicker, with big dots, in almost black.
        hrobot = plot3(points_to_plot(1,:),points_to_plot(2,:),points_to_plot(3,:),'.-','linewidth',7,'markersize',25,'color',[.25 .25 .125]);
        
        % Also plot the tip position of the robot, using hold on and hold
        % off, also keeping a handle to the plot so we can update the data
        % points later.
        hold on;
        htip = plot3(tip_history(1,i),tip_history(2,i),tip_history(3,i),'b.');
        hold off;

        % Label the axes.
        xlabel('X (in.)');
        ylabel('Y (in.)');
        zlabel('Z (in.)');

        % Turn on the grid and the box.
        grid on;
        box on;

        % Set the axis limits.
        axis([-20 20 -20 20 0 40]*0.0254)

        % Set the axis properties for 3D visualization, which makes one
        % unit the same in every direction, and enables rotation.
        axis vis3d;

        % Put text on the plot to show how much time has elapsed.  The text
        % is centered.
        htime = text(-10,10,0,sprintf('t = %.2f s',t(i)),'horizontalAlignment','center');

        % Add a title including the student's name.
        title(['PUMA 260 Robot by ' student_name]);
    else
        % Once the animation has been set up, we don't need to reformat the
        % whole plot.  We just set the data to the correct new values for
        % the robot animation, the tip history, and the text showing the
        % elapsed time.
        set(hrobot,'xdata',points_to_plot(1,:),'ydata',points_to_plot(2,:),'zdata',points_to_plot(3,:))
        set(htip,'xdata',tip_history(1,1:i),'ydata',tip_history(2,1:i),'zdata',tip_history(3,1:i))
        set(htime,'string', (sprintf('t = %.2f s',t(i))));
    end
    
    % Pause for a short duration so that the viewer can watch animation evolve over time.
    pause(GraphingTimeDelay)
    
end

disp('Done with the animation.')
