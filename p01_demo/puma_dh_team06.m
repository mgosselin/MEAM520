function A = puma_dh_team06( a, alpha, d, theta )
%PUMA_DH_TEAM06 calculates transormation matrix for DH parameters
% This function calculates the four-by-four transformation matrix A for a
% given set of DH parameters: a, alpha, d, and theta.  The angles are in
% radius.

Rot_z = [cos(theta) -sin(theta) 0   0;
         sin(theta) cos(theta)  0   0;
         0          0           1   0;
         0          0           0   1];
     
Trans_z = [1    0   0   0;
           0    1   0   0;
           0    0   1   d;
           0    0   0   1];
       
Trans_x = [1    0   0   a;
           0    1   0   0;
           0    0   1   0;
           0    0   0   1];
       
Rot_x = [1  0           0           0;
         0  cos(alpha) -sin(alpha)  0;
         0  sin(alpha) cos(alpha)   0;
         0  0           0           1];
     
A = Rot_z * Trans_z * Trans_x * Rot_x;
