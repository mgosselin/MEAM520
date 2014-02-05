function [R G B]=ColorDetermination(x,y,PIC,W,L)
x1 = floor(x);
x2 = ceil(x);
y1 = floor(y);
y2 = ceil(y);
x1 = max(x1,1);
x1 = min(x1,W);
x2 = max(x2,1);
x2 = min(x2,W);
y1 = max(y1,1);
y1 = min(y1,L);
y2 = max(y2,1);
y2 = min(y2,L);
Point1 = zeros(3,1);
Point2 = zeros(3,1);
Point3 = zeros(3,1);
Point4 = zeros(3,1);
Point1(1) = PIC(x1,y1,1);
Point1(2) = PIC(x1,y1,2);
Point1(3) = PIC(x1,y1,3);
Point2(1) = PIC(x2,y1,1);
Point2(2) = PIC(x2,y1,2);
Point2(3) = PIC(x2,y1,3);
Point3(1) = PIC(x1,y2,1);
Point3(2) = PIC(x1,y2,2);
Point3(3) = PIC(x1,y2,3);
Point4(1) = PIC(x2,y2,1);
Point4(2) = PIC(x2,y2,2);
Point4(3) = PIC(x2,y2,3);
if x1==x2
    Point12 = Point1;
    Point34 = Point3;
else
    Point12 = Point1*(x2-x)+Point2*(x-x1);
    Point34 = Point3*(x2-x)+Point4*(x-x1);
end
if y1==y2
    Point = Point12;
else
    Point = Point12*(y2-y)+Point34*(y-y1);
end
R = Point(1)/255;
G = Point(2)/255;
B = Point(3)/255;
