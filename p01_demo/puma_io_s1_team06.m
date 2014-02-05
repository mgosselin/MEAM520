function [th4 th5 th6]=puma_io_s1_team06(theta1, theta2, theta3, phi, theta, psi)
%% function configuration
%this code is wrote by Ynkai Cui
%the basic method in this function is in the SHV, from page 106 to 107 and
%from page 54 to 55
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
%% Calcute the theta4 theta5 theta6
A1 = puma_dh_team06(0,  pi/2, a, theta1);
A2 = puma_dh_team06(c,  0,   -b, theta2);
A3 = puma_dh_team06(0, -pi/2,-d, theta3);

%claculate the transifermation matrix from frame 3 to frame 0
A03 = A1*A2*A3;
R03 = A03(1:3,1:3);
%calculate the Eular transformation on the base frame
R = puma_euler_team06(phi, theta, psi);
%Calculate the transformation matrix from frame 3 to 3, which is also know
%is the Eular transformation on frame 3
R36 = (R03')*R;
r33 = R36(3,3);
th5 = atan2(-sqrt(1-r33^2),r33);
if sin(th5) == 0
    th4 = atan2(R36(2,1),R36(1,1));
    th6 = 0;
else
    th4 = atan2(R36(2,3),R36(1,3));
    th6 = atan2(R36(3,2),-R36(3,1));
end
