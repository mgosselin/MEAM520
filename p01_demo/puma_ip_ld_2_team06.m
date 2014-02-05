function [th1 th2 th3] = puma_ip_ld_2_team06(x, y, z)
%PUAM_IP_LD_TEAM06 calculates inverse position solution for left arm elbow down
%
% this function is used to calculate the th1 th2 and th3 for the inverse
% kinematics, the input is the desired coordinates of the wrist center,
% which is also the outline of the picture we want to draw. And the output
% is the th1 th2 and th3 angle for the first three links

a = 13.0 * 0.0254; % meters
b =  2.5 * 0.0254; % meters
c =  8.0 * 0.0254; % meters
d =  2.5 * 0.0254; % meters
e =  8.0 * 0.0254; % meters
%% Wrist Center Calculation
try
    alpha = asin((b+d)/sqrt(x^2+y^2));
    % alpha = atan2((b+d),sqrt(x^2+y^2-(b+d)^2));
    phi = atan2(y,x);
    th1 = phi - alpha;
    % the legth of projection of link 2 and link3
    n = sqrt(x^2+y^2-(b+d)^2);
    m = sqrt(n^2+(z-a)^2);
    
    % p = (c^2+e^2-m^2)/(2*c*e);
    th3 = acos((c^2+e^2-m^2)/(2*c*e))-pi*3/2;
    % th3 = pi/2-atan2(sqrt(1-p^2),p);
    th2 = atan2((z-a),n) - atan2(e*sin(th3+pi/2),(c+e*cos(th3+pi/2)));
catch err
    
end
