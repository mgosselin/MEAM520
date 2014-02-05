function [th1 th2 th3 th4 th5 th6] = puma_ik_team06(x, y, z, phi, theta, psi, th1now, th2now, th3now, th4now, th5now, th6now)
%% PUMA_IK_TEAM00  Calculates the full inverse kinematics for the PUMA 260.
%
% This Matlab file provides the starter code for the PUMA 260 inverse
% kinematics function of project 1 in MEAM 520 at the University of
% Pennsylvania.  The original was written by Professor Katherine J.
% Kuchenbecker in October of 2012. Students will work in teams modify this
% code to create their own script. Post questions on the class's Piazza
% forum. 
%
% The first three input arguments (x, y, z) are the desired coordinates of
% the PUMA's end-effector tip in meters (changed from initial release),
% specified in the base frame.  The origin of the base frame is where the
% first joint axis (waist) intersects the table. The z0 axis points up, and
% the x0 axis points out away from the robot, perpendicular to the front
% edge of the table.  These arguments are mandatory.
%
% The fourth through sixth input arguments (phi, theta, psi) represent the
% desired orientation of the PUMA's end-effector in the base frame using
% ZYZ Euler angles in radians.  These arguments are mandatory.
%
% The seventh through twelfth input arguments (th1now ... th6now) are the
% current joint angles of the PUMA.  These arguments are optional, but you
% must supply all of them if you supply any of them.  Passing in the
% robot's current joint angles enables this function to find an IK solution
% close to the robot's current configuration, to avoid large jumps in the
% robot's movement.  If these values are not passed in, the function may
% select from the possible solutions in any manner.
%
% The six outputs (th1 ... th6) are the joint angles needed to place the
% PUMA's end-effector at the desired position and in the desired
% orientation. These joint angles are specified in radians according to the
% order and sign conventions described in the documentation.  If this
% function cannot find a solution to the inverse kinematics problem, it
% will pass back NaN (not a number) for all of the thetas.
%
% Please change the name of this file and the function declaration on the
% first line above to include your team number rather than 00.  Also list
% your neam number and the full names of your three team members below.
%
% Team Number: 06
% Team Members:  Edward Cui, Michael Gosselin, Chao Qu


%% CHECK INPUTS

% Look at the number of arguments the user has passed in to make sure this
% function is being called correctly.
if (nargin < 6)
    error('Not enough input arguments.  You need six or twelve.')
elseif (nargin == 6)
    % They have passed in only the first six arguments, so the robot's
    % current joint angles are not defined.  There is nothing special to do
    % in this case.
elseif ((nargin > 6) && (nargin < 12))
    error('Incorrect number of input arguments.  You need six or twelve.')
elseif (nargin == 12)
    % There is nothing special to do in this case.
elseif (nargin > 12)
    error('Too many input arguments.  You need six or twelve.')
end

%% DEFINE JOINT ANGLE LIMITS
lower = 1;
upper = 2;
range = 3;

theta_range = [-180 110 290;
    -75  240 315;
    -235 60 295;
    -580 40  620;
    -120 110 230;
    -215 295 510];

theta_range = theta_range * pi / 180;

%% CALCULATE INVERSE KINEMATICS SOLUTION(S)

% For now, just set each theta to NaN (not a number).  This is what you
% should output if there is no solution to the inverse kinematics problem
% for the position and orientation that were passed in.  For example, this
% would be the correct output if the desired position for the end-effector
% was outside the robot's reachable workspace.  We use this sentinel value
% of NaN to be sure that the code calling this function can tell that
% something is wrong and shut down the PUMA.
%
% You should update this section of the code with your IK solution.
% Please comment your code to explain what you are doing at each step.

[x_c, y_c, z_c] = puma_fw_team06(x, y, z, phi, theta, psi); % Find coordinates for wrist center
puma_ip = {@puma_ip_ld_team06, @puma_ip_lu_team06, @puma_ip_rd_team06, @puma_ip_ru_team06};
puma_io = {@puma_io_s1_team06, @puma_io_s2_team06};

% Generate all 8 solutions for 6 joint angles
theta_all = zeros(8,6);
k = 1;
point_exceeded = 0;
for i = 1:4
    try
        [th1, th2, th3] = puma_ip{i}(x_c, y_c, z_c);
    catch err
        th1 = NaN;
        th2 = NaN;
        th3 = NaN;
        if point_exceeded == 0
            disp(['Point (' num2str(x) ', ' num2str(y) ', ' num2str(z) ') out of workspace. Solution not evaluated.']);
            point_exceeded = 1;
        end
    end
    for j = 1:2
        [th4, th5, th6] = puma_io{j}(th1, th2, th3, phi, theta, psi);
        if th4 > theta_range(4,upper)
            th4 = th4 - 2*pi;
        end
        theta_all(k,:) = [th1, th2, th3, th4, th5, th6];
        k = k + 1;
    end
end

% Consider joint angle limitations
t = ones(8,1);

for i = 1:6
    t = t .* (theta_all(:,i) < theta_range(i,upper)) .* (theta_all(:,i) > theta_range(i,lower));
end

s = find(t == 1);

angle_diff = zeros(8,1);

if isempty(s)
    th1 = NaN;
    th2 = NaN;
    th3 = NaN;
    th4 = NaN;
    th5 = NaN;
    th6 = NaN;
    
    t = ones(8,1);
    joint_exceeded = 0;
    for i = 1:6
        t = t .* (theta_all(:,i) < theta_range(i,upper)) .* (theta_all(:,i) > theta_range(i,lower));
        if sum(t) < 8
            if joint_exceeded == 0
                disp(['No full solutions in range. At least joint ' num2str(i) ' out of range.']);
                joint_exceeded = 1;
            end
        end
    end
else
    if (exist('th1now','var'))
        th_all_now = [th1now th2now th3now th4now th5now th6now];
        for n = 1:length(s)
            angle_diff(s(n),1) = sum(abs(th_all_now - theta_all(s(n),:)));
        end
        
        angle_diff = angle_diff .* t;
        angle_diff(~angle_diff) = NaN;
        
        [g, h] = min(angle_diff);
    else
        h = s(1);
    end
    th1 = theta_all(h,1);
    th2 = theta_all(h,2);
    th3 = theta_all(h,3);
    th4 = theta_all(h,4);
    th5 = theta_all(h,5);
    th6 = theta_all(h,6);
end

%% HANDLE THE ROBOT'S CURRENT CONFIGURATION, IF SUPPLIED

% If the user passed in the PUMA's current joint angles, handle the
% constraint that the returned solution should be close to the current
% joint angles.  This feature is needed because there are multiple
% solutions to the PUMA's inverse kinematics.

    % They did pass in the current joint angles.
    % Do something smart with the solution you calculated above and the
    % PUMA's current joint angles to select the solution closest to the
    % current configuration.
    
    % For now, we don't do anything special.  You will definitely want to change this.

% By the very end, th1 through th6 should hold the values of the joint
% angles that will put the PUMA's end-effector in the desired
% configuration.  If that configuration is not reachable, set all of the
% joint angles to NaN.