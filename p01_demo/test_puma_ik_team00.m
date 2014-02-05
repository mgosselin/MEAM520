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
puma_ik = @puma_ik_team00;
disp(['Testing function ' func2str(puma_ik) '.m'])


%% CALCULATE IK SCORE WITHOUT CURRENT CONFIGURATION
% Calculate a score that reflects the performance of your team's inverse
% kinematics function when called without th1now to th6now.  You need to
% design a test protocol you think is reasonable here.  The score should
% vary from 0 to 100 based on the level of performance, with 0 being worst
% and 100 being perfect.

score1 = zeros(100,1);
for i = 1:length(score1)
    % Choose a random target.  You will want to change this.
    x = rand;
    y = rand;
    z = rand;
    phi = rand;
    theta = rand;
    psi = rand;
    
    % Run the puma_ik inverse kinematics function being tested.
    [theta1 theta2 theta3 theta4 theta5 theta6] = puma_ik(x, y, z, phi, theta, psi);

    % Inspect the solution and update the score.
    % For now run a dumb test on whether theta1 is greater than theta2
    score1(i) = theta1 > theta2;
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
for i = 1:length(score2)
    % Choose a random target.  You will want to change this.
    x = rand;
    y = rand;
    z = rand;
    phi = rand;
    theta = rand;
    psi = rand;
    
    % Choose random joint angles to start from.
    th1now = rand;
    th2now = rand;
    th3now = rand;
    th4now = rand;
    th5now = rand;
    th6now = rand;
    
    % Run the puma_ik inverse kinematics function being tested.
    [theta1 theta2 theta3 theta4 theta5 theta6] = puma_ik(x, y, z, phi, theta, psi, th1now, th2now, th3now, th4now, th5now, th6now);

    % Inspect the solution and update the score.
    % For now run a dumb test on whether theta3 is greater than theta4
    score2(i) = theta3 > theta4;
end
% Calculate the total score.
score_with_thnow = 100 * sum(score2) / length(score2);
disp(['Score with current configuration = ' num2str(score_with_thnow)])
if (score_with_thnow == 100)
    disp(' Good job!')
end
