function V = test_puma_Find_Direction_team06(phi, theta, psi)
%% Calculation Process
R_z_phi = [cos(phi) -sin(phi)   0;
           sin(phi) cos(phi)    0;
           0        0           1];
R_y_theta = [cos(theta)     0   sin(theta);
             0              1   0         ;
             -sin(theta)    0   cos(theta)];
R_z_psi = [cos(psi) -sin(psi)   0 ;
           sin(psi) cos(psi)    0 ;
           0        0           1 ];
R = R_z_phi * R_y_theta * R_z_psi;
Vled = [0 0.125*0.0254 1.25*0.0254]';
V = R*Vled;
