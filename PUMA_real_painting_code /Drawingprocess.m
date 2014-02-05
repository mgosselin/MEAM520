%% Puma Simulator Demo

% Initializiation
% Close all figure windows.
close all

% Clear all global variables.  You should do this before calling any PUMA
% functions.
clear all

% Move the cursor to the top of the command window so new text is easily
% seen.
home

%% Picture Demo
% Open figure 1 and clear it.
figure(1); clf
fileName = 'PICfkData.mat';
load(fileName,'PICfkData');
initialStep = 50;
th1Ini = PICfkData(1,1);
th2Ini = PICfkData(1,2);
th3Ini = PICfkData(1,3);
th4Ini = PICfkData(1,4);
th5Ini = PICfkData(1,5);
th6Ini = PICfkData(1,6);
th1I = 0:(th1Ini/(initialStep-1)):th1Ini;
th2I = 0:(th2Ini/(initialStep-1)):th2Ini;
th3I = 0:(th3Ini/(initialStep-1)):th3Ini;
th4I = 0:(th4Ini/(initialStep-1)):th4Ini;
th5I = -1.5708:((th5Ini+1.5708)/(initialStep-1)):th5Ini;
th6I = 0:(th6Ini/(initialStep-1)):th6Ini;

% Initialize the PUMA simulation
pumaStart('PlotEveryNFrames', 20, 'LEDMarkerStyle', '.');

pumaAngles % see the joint angles
for k = 1:initialStep
    pumaServo(th1I(k),th2I(k),th3I(k),th4I(k),th5I(k),th6I(k));
end
pumaAngles % see the joint angles
% turn on the LED
pumaLEDOn;
for k=1:size(PICfkData,1)
    pumaLEDSet(PICfkData(k,7), PICfkData(k,8), PICfkData(k,9));
    pumaServo(PICfkData(k,1), PICfkData(k,2), PICfkData(k,3), PICfkData(k,4), PICfkData(k,5), PICfkData(k,6));
end
pumaLEDOff;
pumaStop;
