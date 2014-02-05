clear all
clc
%% Initial Status Configuration
resolution = [320 480];
plainSizeActual = [0.3 0.4];
centerPoint = [0.32 0 0.3]';
cameraPosition = [0.8 0 0.3]';  %the center point of the picture and camera and the origin point of frame 1 must be in the same line
picDirection = cameraPosition-centerPoint;
PICBox = zeros(resolution(1),resolution(2),6);  %the first three value of each point are the coordinates of the point in actual space and the last three values are RGB color

path = 'E:\academic\MEAM520 Robotics\PUMAmodel\Harry-Potter-032.jpg';
PIC = imread(path);
rescaleRatio = min((size(PIC,1)/resolution(1)),(size(PIC,2)/resolution(2)));


%% Calculate the Position of each point
disp('Calculate the Position of each point');
tic
theta0 = atan2(picDirection(2),picDirection(1));
FrameTrans = [cos(theta0) -sin(theta0);sin(theta0) cos(theta0)];
A = FrameTrans\picDirection(1:2);
B = FrameTrans*centerPoint(1:2);
z = sqrt((plainSizeActual(1)/2)^2/(picDirection(3)^2/(A(1)^2)+1))+centerPoint(3);
x = -(z-centerPoint(3))*picDirection(3)/A(1)+B(1);
point1 = [x B(2)+plainSizeActual(2)/2 z]';
point2 = [x B(2)-plainSizeActual(2)/2 z]';
point3 = [2*B(1)-x B(2)+plainSizeActual(2)/2 2*centerPoint(3)-z]';
point4 = [2*B(1)-x B(2)-plainSizeActual(2)/2 2*centerPoint(3)-z]';

deltaY = plainSizeActual(2)/(resolution(2)-1);
deltaX = (point1(1)-point3(1))/(resolution(1)-1);
deltaZ = (point1(3)-point3(3))/(resolution(1)-1);
IdealPICBox = zeros(resolution(1),resolution(2),3);
tmpVar0 = zeros(2,1); 
tmpVar1 = zeros(2,1);
for i=1:resolution(1)
    for j=1:resolution(2)
        if (i==1)&&(j==1)
            IdealPICBox(i,j,1:3)=point1;
        else
            if j==1
                IdealPICBox(i,j,1) = IdealPICBox(i-1,j,1)-deltaX;
                IdealPICBox(i,j,2) = IdealPICBox(i-1,j,2);
                IdealPICBox(i,j,3) = IdealPICBox(i-1,j,3)-deltaZ;
            else
                IdealPICBox(i,j,1) = IdealPICBox(i,j-1,1);
                IdealPICBox(i,j,2) = IdealPICBox(i,j-1,2)-deltaY;
                IdealPICBox(i,j,3) = IdealPICBox(i,j-1,3);
            end
        end
        tmpVar0(1) = IdealPICBox(i,j,1);
        tmpVar0(2) = IdealPICBox(i,j,2);
        tmpVar1 = FrameTrans*tmpVar0;
        PICBox(i,j,1) = tmpVar1(1);
        PICBox(i,j,2) = tmpVar1(2);
        PICBox(i,j,3) = IdealPICBox(i,j,3);
    end
end
toc
%% Calculate the Color of each point
disp('Calculate the Color of each point');
tic
for i=1:resolution(1)
    for j=1:resolution(2)
        PixelX = size(PIC,1)/2+(i-resolution(1)/2)*rescaleRatio;
        PixelY = size(PIC,2)/2+(j-resolution(2)/2)*rescaleRatio;
%         PICBox(i,j,4) = PIC(size(PIC,1)/2+(i-resolution(1)/2)*rescaleRatio,size(PIC,2)/2+(i-resolution(2)/2)*rescaleRatio,1);
%         PICBox(i,j,5) = PIC(size(PIC,1)/2+(i-resolution(1)/2)*rescaleRatio,size(PIC,2)/2+(i-resolution(2)/2)*rescaleRatio,2);
%         PICBox(i,j,6) = PIC(size(PIC,1)/2+(i-resolution(1)/2)*rescaleRatio,size(PIC,2)/2+(i-resolution(2)/2)*rescaleRatio,3);
        [PICBox(i,j,4) PICBox(i,j,5) PICBox(i,j,6)] = ColorDetermination(PixelX,PixelY,PIC,size(PIC,1),size(PIC,2));
    end
end
toc
%% Save the data
disp('Save the data in row array');
tic
PICData = zeros(resolution(1)*resolution(2),6);
i = 1;
numofdata = 1;
for j=1:resolution(2)
    while (i<=resolution(1))&&(i>=1)
        PICData(numofdata,1) = PICBox(i,j,1);
        PICData(numofdata,2) = PICBox(i,j,2);
        PICData(numofdata,3) = PICBox(i,j,3);
        PICData(numofdata,4) = PICBox(i,j,4);
        PICData(numofdata,5) = PICBox(i,j,5);
        PICData(numofdata,6) = PICBox(i,j,6);
        numofdata = numofdata+1;
        if mod(j,2)==1
            i=i+1;
        else
            i=i-1;
        end
    end
    if i<1
        i=1;
    end
    if i>resolution(1)
        i=resolution(1);
    end
end
toc

%% Using IK to find th1 to th6
disp('Calculate inverse kinematics');
tic
PICfkData = zeros(resolution(1)*resolution(2),9);
for i=1:resolution(1)*resolution(2)
    [phi, theta, psi]=test_puma_Find_EularAngle_team06(PICData(i,1),PICData(i,2),PICData(i,3),cameraPosition(1),cameraPosition(2),cameraPosition(3));
    if i==1
        [PICfkData(i,1) PICfkData(i,2) PICfkData(i,3) PICfkData(i,4) PICfkData(i,5) PICfkData(i,6)]=puma_ik_3_team06(PICData(i,1),PICData(i,2),PICData(i,3),phi, theta, psi);
    else
        [PICfkData(i,1) PICfkData(i,2) PICfkData(i,3) PICfkData(i,4) PICfkData(i,5) PICfkData(i,6)]=puma_ik_3_team06(PICData(i,1),PICData(i,2),PICData(i,3),phi, theta, psi,PICfkData(i-1,1),PICfkData(i-1,2),PICfkData(i-1,3),PICfkData(i-1,4),PICfkData(i-1,5),PICfkData(i-1,6));
    end
    PICfkData(i,7) = PICData(i,4);
    PICfkData(i,8) = PICData(i,5);
    PICfkData(i,9) = PICData(i,6);
end
toc
%% Draw the Picture in matlab
% disp('Draw the Picture')
% tic
% for i=1:resolution(1)
%     for j=1:resolution(2)
%         if (i==1)&&(j==1)
%             picture3D = plot3(PICBox(i,j,1),PICBox(i,j,2),PICBox(i,j,3),'.','markersize',10,'color',[PICBox(i,j,4) PICBox(i,j,5) PICBox(i,j,6)]);
%             % Label the axes.
%             xlabel('X (meter.)');
%             ylabel('Y (meter.)');
%             zlabel('Z (meter.)');
% 
%             % Turn on the grid and the box.
%             grid on;
%             box on;
%             % Set the axis limits.
%             axis([-20*0.0254 20*0.0254 -20*0.0254 20*0.0254 0 40*0.0254]);
%             axis vis3d;
%             hold on
%         else
%             plot3(PICBox(i,j,1),PICBox(i,j,2),PICBox(i,j,3),'.','markersize',15,'color',[PICBox(i,j,4) PICBox(i,j,5) PICBox(i,j,6)]);
%             %set(picture3D,'xdata',PICBox(i,j,1),'ydata',points_to_plot(2,:),'zdata',points_to_plot(3,:),'.','markersize',15,'color',[PICBox(i,j,4) PICBox(i,j,5) PICBox(i,j,6)]);
%         end
%     end
% end
% toc