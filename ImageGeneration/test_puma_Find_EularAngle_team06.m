function [phi, theta, psi]=test_puma_Find_EularAngle_team06(Xw, Yw, Zw, Xc, Yc, Zc)
%% Function Configuration
% This file is created by Cui, Yunkai at 10/14/2012
% This function is used to find the Eular angle of the end effector in the 
% base frame to make it point to the camera when we draw the picture we want.
% The input is the coordinates of the points we want to draw and the
% coordinates of the camera

initialConfig = [0 0 1 1]'; % initial configuration of the effector in the base frame
%Vled = [0 0.125*0.0254 1.25*0.0254]';
%% Calculatetion Process
vectorOfEffector = [Xc-Xw, Yc-Yw, Zc-Zw]';

    Ctheta = vectorOfEffector(3)/norm(vectorOfEffector);
    %theta = acos(Ctheta);
    theta = atan2(sqrt(1-Ctheta^2),Ctheta);
    phi = atan2(vectorOfEffector(2)/norm(vectorOfEffector),vectorOfEffector(1)/norm(vectorOfEffector));
    psi = 0;
