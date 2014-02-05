function [x_c y_c z_c] = puma_fw_team06(x, y, z, phi, theta, psi)
%% Function Configuration

f =  2.5 * 0.0254; % meters
%% Calculate the Wrist Center Point
%Eular Angle Transformation
mode = 1;
R = puma_euler_team06(phi, theta, psi);
led = [0 0.125*0.0254 1.25*0.0254]';
Vtmp = R*led;

switch mode
    case 0
        x_c = x - f*R(1,3);
        y_c = y - f*R(2,3);
        z_c = z - f*R(3,3);
    case 1
        x_c = x - Vtmp(1)- f*R(1,3);
        y_c = y - Vtmp(2)- f*R(2,3);
        z_c = z - Vtmp(3)- f*R(3,3);
    otherwise
        error('Error!')
end

