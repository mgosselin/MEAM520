function T06 = puma_fk_team06(th1, th2, th3, th4, th5, th6)
%% PUMA_FK_TEAM06  Calculates the forward kinematics for the PUMA 260.
%
% This Matlab file provides the starter code for the PUMA 260 forward
% kinematics function of project 1 in MEAM 520 at the University of
% Pennsylvania.  The original was written by Professor Katherine J.
% Kuchenbecker in October of 2012. Students will work in teams modify this
% code to create their own script. Post questions on the class's Piazza
% forum.
%
% The six inputs (th1 ... th6) are the PUMA's current joint angles in
% radians, specified according to the order and sign conventions described
% in the documentation.
%
% The one output is the homogeneous transformation representing the pose of
% frame 6 in frame 0.  The position part of this transformation is in
% meters (changed from initial version).
%
% Please change the name of this file and the function declaration on the
% first line above to include your team number rather than 00.  Also list
% your neam number and the full names of your three team members below.
%
% Team Number:
% Team Members:


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

%% CALCULATE
% Calculate the homogenous transformation representing the pose of frame 6
% in frame 0, given the current values of th1 through th6, which are all in
% radians.  The position part of this transformation should be in meters.

A1 = puma_dh_team06(0,  pi/2,  a, th1);
A2 = puma_dh_team06(c,     0, -b, th2);
A3 = puma_dh_team06(0, -pi/2, -d, th3);
A4 = puma_dh_team06(0,  pi/2,  e, th4);
A5 = puma_dh_team06(0, -pi/2,  0, th5);
A6 = puma_dh_team06(0,     0,  f, th6);

T06 = A1*A2*A3*A4*A5*A6;