function R = puma_euler_team06(phi, theta, psi)
% Use The Given Eular Angle To Get The Eular Transformation
R_z_phi = [cos(phi) -sin(phi)   0;
           sin(phi) cos(phi)    0;
           0        0           1];
R_y_theta = [cos(theta)     0   sin(theta);
             0              1   0;
             -sin(theta)    0   cos(theta)];
R_z_psi = [cos(psi) -sin(psi)   0;
           sin(psi) cos(psi)    0;
           0        0           1];
     
R = R_z_phi * R_y_theta * R_z_psi;