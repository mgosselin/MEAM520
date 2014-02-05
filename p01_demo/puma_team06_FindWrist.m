function [Xc Yc Zc] = puma_team06_FindWrist(x, y, z, phi, theta, psi)
%% Function Configuration

f =  2.5*0.0254; % meters
%% Calculate the Wrist Center Point
%Eular Angle Transformation
Rzphi = [ cos(phi) -sin(phi) 0 0;
          sin(phi)  cos(phi) 0 0;
          0         0        0 0;
          0         0        0 1];
Rytheta = [ cos(theta) 0 sin(theta) 0;
            0          1 0          0;
           -sin(theta) 0 cos(theta) 0;
            0          0 0          1];
Rzpsi = [cos(psi) -sin(psi) 0 0;
         sin(psi)  cos(psi) 0 0;
         0         0        1 0;
         0         0        0 1];
Rzyz = Rzphi*Rytheta*Rzpsi;
%the vector of the last link in the final frame(which is also the base frame)
Veff = [0 0 f 1]';
Veffbas = Rzyz*Veff;

Xc = x - Veffbas(1);
Yc = y - Veffbas(2);
Zc = z - Veffbas(3);
