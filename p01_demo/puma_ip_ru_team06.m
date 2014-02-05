function [ th1, th2, th3 ] = puma_ip_ru_team06( x_c, y_c, z_c )
%PUAM_IP_RU_TEAM06 calculates inverse position solution for right arm elbow up

%% ROBOT PARAMETERS
% This problem is about the PUMA 260 robot, a 6-DOF manipulator.

% Define the robot's measurements.
a = 13.0 * 0.0254; % meters
b =  2.5 * 0.0254; % meters
c =  8.0 * 0.0254; % meters
d =  2.5 * 0.0254; % meters
e =  8.0 * 0.0254; % meters
f =  2.5 * 0.0254; % meters

%% INVERSE POSITION
% Calcutate th1 for right arm configuration
alpha = atan2(y_c, x_c);
r = sqrt(x_c^2 + y_c^2 - (b+d)^2);
beta = atan2(-(b+d),-r);
th1 = alpha + beta; % theta1

% Calculate th2 and th3 for right arm, elbow up configuration
l = r;
s = z_c-a;
D = (l^2 + s^2 - c^2 - e^2)/(2*c*e);
phi = atan2(-sqrt(1-D^2), D);
th3 = 3*pi/2 - phi; % theta3
th2 = pi - atan2(s, l) + atan2(e*sin(phi), c+e*cos(phi)); % theta2
end

