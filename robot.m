function [ robot ] = robot( Z,del,l,b,lstyle,color,out)
F_x=sign(out(9:12)).*min(abs(out(9:12))/2,6);
F_y=sign(out(13:16)).*min(abs(out(13:16))/5,5);
%function [ robot ] = robot( x,y,teta,l,b,lstyle,color)
%   Detailed explanation goes here
x=Z(1);y=Z(2);
teta=Z(3)+1*out(end);
v=l*Z(4)/5; vn=l*Z(5)/2;
F_y=l*F_y/5;
F_x=l*F_x/5;
r=l/3;
% l=10; b=3;
if abs(cos(teta)) > .1 && y>3 
    teta=teta*1.12;
end
if abs(cos(teta)) > .1 && y<-0.1 
    teta=teta*1.13;
end
CT=cos(teta); ST=sin(teta);
CTD=cos(teta+del);
STD=sin(teta+del);
p1=[x+CT*l-ST*b
    y+ST*l+CT*b];
p4=p1-2*l*[CT;ST];
p3=p4+2*b*[ST;-CT];
p2=p3+2*l*[CT;ST];

V=[[x;y] [x+v*CT;y+v*ST]];
V_n=[[x;y] [x-vn*ST;y+vn*CT]];
draw_arrow([x;y]',[x+v*CT;y+v*ST]',3)
draw_arrow([x;y]',[x-vn*ST;y+vn*CT]',2)
w1p1=[x+CT*(.8*l)+CTD*r-ST*b;y+ST*(.8*l)+STD*r+CT*b]; w1p2=[x-CT*(-.8*l)-CTD*r-ST*b;y-ST*(-.8*l)-STD*r+CT*b];
wheel1=[w1p1 w1p2];
w2p1=[x+CT*(.8*l)+CTD*r+ST*b;y+ST*(.8*l)+STD*r-CT*b]; w2p2=[x-CT*(-.8*l)-CTD*r+ST*b;y-ST*(-.8*l)-STD*r-CT*b];
wheel2=[w2p1 w2p2];
w3p1=[x+CT*(r-.8*l)-ST*b;y+ST*(r-.8*l)+CT*b]; w3p2=[x-CT*(r+.8*l)-ST*b;y-ST*(r+.8*l)+CT*b];
wheel3=[w3p1 w3p2];
w4p1=[x+CT*(r-.8*l)+ST*b;y+ST*(r-.8*l)-CT*b]; w4p2=[x-CT*(r+.8*l)+ST*b;y-ST*(r+.8*l)-CT*b];
wheel4=[w4p1 w4p2];

F_x1=[(w1p1+w1p2)/2 (w1p1+w1p2)/2+[F_x(1)*CTD;F_x(1)*STD]];
F_x2=[(w2p1+w2p2)/2 (w2p1+w2p2)/2+[F_x(2)*CTD;F_x(2)*STD]];
F_x3=[(w3p1+w3p2)/2 (w3p1+w3p2)/2+[F_x(3)*CT;F_x(3)*ST]];
F_x4=[(w4p1+w4p2)/2 (w4p1+w4p2)/2+[F_x(4)*CT;F_x(4)*ST]];

F_y1=[(w1p1+w1p2)/2 (w1p1+w1p2)/2+[-F_y(1)*STD;F_y(1)*CTD]];
F_y2=[(w2p1+w2p2)/2 (w2p1+w2p2)/2+[-F_y(2)*STD;F_y(2)*CTD]];
F_y3=[(w3p1+w3p2)/2 (w3p1+w3p2)/2+[-F_y(3)*ST;F_y(3)*CT]];
F_y4=[(w4p1+w4p2)/2 (w4p1+w4p2)/2+[-F_y(4)*ST;F_y(4)*CT]];

robot=[p1 p2 p3 p4 p1];
hold on
% Y=[x+.1*CT; y+.1*ST];
% plot(Y(1),Y(2),'Color',color,'LineStyle','o')
plot(wheel1(1,:),wheel1(2,:),'Color',color,'LineStyle',lstyle,'LineWidth',5)
plot(wheel2(1,:),wheel2(2,:),'Color',color,'LineStyle',lstyle,'LineWidth',5)
plot(wheel3(1,:),wheel3(2,:),'Color',color,'LineStyle',lstyle,'LineWidth',5)
plot(wheel4(1,:),wheel4(2,:),'Color',color,'LineStyle',lstyle,'LineWidth',5)
% plot(F_x1(1,:),F_x1(2,:),'Color','red','LineStyle',lstyle,'LineWidth',3)
% plot(F_x2(1,:),F_x2(2,:),'Color','red','LineStyle',lstyle,'LineWidth',3)
% plot(F_x3(1,:),F_x3(2,:),'Color','red','LineStyle',lstyle,'LineWidth',3)
% plot(F_x4(1,:),F_x4(2,:),'Color','red','LineStyle',lstyle,'LineWidth',3)
% plot(F_y1(1,:),F_y1(2,:),'Color','green','LineStyle',lstyle,'LineWidth',3)
% plot(F_y2(1,:),F_y2(2,:),'Color','green','LineStyle',lstyle,'LineWidth',3)
% plot(F_y3(1,:),F_y3(2,:),'Color','green','LineStyle',lstyle,'LineWidth',3)
% plot(F_y4(1,:),F_y4(2,:),'Color','green','LineStyle',lstyle,'LineWidth',3)
plot(V(1,:),V(2,:),'Color','Yellow','LineStyle',lstyle,'LineWidth',4)
plot(V_n(1,:),V_n(2,:),'Color','blue','LineStyle',lstyle,'LineWidth',4)
plot(robot(1,:),robot(2,:),'Color',color+.5,'LineStyle',lstyle,'LineWidth',1)

function draw_arrow(startpoint,endpoint,headsize) 
%by Ryan Molecke
v1 = headsize*(startpoint-endpoint)/2.5;
theta = 22.5*pi/180; 
theta1 = -1*22.5*pi/180; 
rotMatrix = [cos(theta) -sin(theta) ; sin(theta) cos(theta)]; 
rotMatrix1 = [cos(theta1) -sin(theta1) ; sin(theta1) cos(theta1)]; 
v2 = v1*rotMatrix; 
v3 = v1*rotMatrix1; 
x1 = endpoint; 
x2 = x1 + v2; 
x3 = x1 + v3; 
hold on; 
% below line fills the arrowhead (black) 
fill([x1(1) x2(1) x3(1)],[x1(2) x2(2) x3(2)],[.5 1 0]);
% below line draws line (black) 
plot([startpoint(1) endpoint(1)],[startpoint(2) endpoint(2)],'linewidth',2,'color',[0 0 0]);