function [T,z,u,out,Xd]=UturnNoSafe(vd,tf,ru)
% close all;
% vd=2; ru=1; tf=2;
Del_T=0.002;
T=0:Del_T:tf;
Xd=u_turn_path(T,vd,ru);
v0=norm(Xd(4:5,1),2);
% v0=5;
% Xd=[v0*T;0*sign(T);0*T;v0*sign(T);0*T;0*T];
z(:,1)=[Xd(1,1) -0.05 pi/2 .99*v0 .0*v0 0 .99*v0*ones(1,4)/.055]';
dz=z(:,1);

% R=40; wr=.25;
% Xd=[R*sin(wr*round(T));R*cos(wr*round(T))-R;wr*round(T);wr*R*cos(wr*round(T));-wr*R*sin(wr*round(T));wr*sign(T)];
 % % % % % % % % % % % Uturn
% Xd=[v0*T;20*sign(T);0*T;v0*sign(T);0*T;0*T];
DELTA=0*T;
% % outs=[alpha; lambda; F_x; F_y dZ \Beta];
lT=length(T);
z=[z zeros(length(z(:,1)),lT-1)];
out=zeros(27,lT);
u=zeros(5,lT);
PH=10;
max_time=0.02;
for i=1:lT-1
if rem(i,PH)==1
    tic        
      u(:,i+1)=NMPC(Xd(:,i+10),z(:,i),u(:,i),Del_T,out(:,i),1.1,.4,max_time);
      u(1,i+1)=PID_ST(u(1,i+1),.1);
      u(1,i+1)=satu(u(1,i+1)-u(1,i),.2)+u(1,i);
    a=[T(i) toc];
else
    u(:,i+1)=u(:,i);
end
[out(:,i+1),dz]=wheel_model(z(:,i),(u(:,i)),dz);
z(:,i+1)=z(:,i)+ dz*Del_T;
DELTA(i)=u(1,i);
end
% hold on
%  figure
%  plot(z(1,:),z(2,:),'r') %path?
%  hold on;
%  plot(Xd(1,:),Xd(2,:),'k') %path
%  xlabel('x');ylabel('yo')
%  axis('equal') 
%  for i=1:round(length(T)/11):length(T)
% % %     hold off;
%      robot(z(:,i),DELTA(1,i),max(z(1,:))/10,max(z(1,:))/15,'-',[0 0 0],out(:,i));
% %      robot(Xd(:,i),DELTA(1,i),max(z(1,:))/10,max(z(1,:))/15,'-.',[.3 .3 .3],out(:,i));
%  end
% %      figure
% %      plot(T,out(end,:),T,out(1:4,:)) %123
%    figure
%    plot(out(3,10:end),out(end,10:end)) %stablity bound
% %     figure
% % % %  hold on
% %     plot(T,DELTA,'b') %steering     
% %    plot(T,out(23,:),'r*') %ddpsi
% %    plot(T,z(6,:),'r*') %ddpsi
% %    xlabel('t');ylabel('Deg')   
% %    plot(T,z(3,:),'g')
% %  xlabel('t');ylabel('\phi')
% %  figure
% %  plot(T,z(1,:),T,z(2,:),'r*') %path
% %  xlabel('t');ylabel('m')
% %  figure
% %  plot(T,z(7:end,:),'r*') %Wv
% %  xlabel('t');ylabel('W V')
% %    figure
% %    plot(T,z(4,:),'r*') %Vel
% %    hold on;
% %   plot(T,sqrt(Xd(4,:).^2+Xd(5,:).^2),'r*') %Vel
% %   xlabel('t');ylabel('Vel')
% %  figure
% %  plot(T,out(9:12,:),'.') %fx
% %  xlabel('t');ylabel('fx')
% % figure
% % plot(T,out(13:16,:),'O') %fy
% % xlabel('t');ylabel('fy')
% %   figure
% %   plot(T,out(1:4,:),'-.')  %alpha
% %   xlabel('t');ylabel('\alpha')
% %  figure
% %  plot(T,out(5:8,:),'-.')  %lambda
% %  xlabel('t');ylabel('\Lambda')

function al = Alpha(z,u,Par)
if(z(4)==0)
    al=[0;0;0;0];
else
alf=atan((z(5)+Par.Car.L_f*z(6))/z(4))-u(1);
alr=atan((z(5)-Par.Car.L_r*z(6))/z(4));
al=[alf;alf;alr;alr];
end

function LWv = Wheel_velocity(z,u,Par,SC)
LWv=[SC.s_d*(z(5)+Par.Car.L_f*z(6))+z(4)*SC.c_d
     SC.s_d*(z(5)+Par.Car.L_f*z(6))+z(4)*SC.c_d
     z(4)
     z(4)];

function lam = Lambda(z,u,Par,WV)
dphi=z(7:10);
lam=(Par.tire.r*dphi-WV)./(max(Par.tire.r*dphi',WV')');

function fx=fricx(lambda,Fz)
persistent B C D E;
if isempty(B)
B=11.275; C=1.56; D=-.95; E=-1.999;
end
alpha=-lambda;
aB=asin(sin(alpha));
f=D*sin(C*atan(B*(1-E)*aB+E*atan(B*aB)));
fx=f.*Fz;
% for i=1:4
%     if fx(i)*alpha(i)>0
%         fx(i)=-fx(i);
%     end
% end
for i=1:2
    if fx(i)>0
        fx(i)=0;
    end
end

function fy=fricy(alpha,Fz)
persistent B C D E;
if isempty(B)
B=11.275; C=1.56; D=-.95; E=-1.999;
end
aB=asin(sin(alpha));
f=D*sin(C*atan(B*(1-E)*aB+E*atan(B*aB)));
fy=f.*Fz;
for i=1:4
    if fy(i)*alpha(i)>0
        fy(i)=-fy(i);
    end
end

function y=satu(x,sat)
y=sign(x)*min(sat,abs(x));

function [ robot ] = robot( Z,del,l,b,lstyle,color,out)
F_x=out(9:12)/4;
F_y=out(13:16)/4;
%function [ robot ] = robot( x,y,teta,l,b,lstyle,color)
%   Detailed explanation goes here
x=Z(1);y=Z(2);
teta=Z(3)+200*out(end);
v=l*Z(4)/25; vn=l*Z(5)/10;
F_y=l*F_y/5;
F_x=l*F_x/5;
r=l/3;
% l=10; b=3;
CT=cos(teta); ST=sin(teta);
CTD=cos(teta+del);
STD=sin(teta+del);
p1=[x+CT*l-ST*b
    y+ST*l+CT*b];
p4=p1-2*l*[CT;ST];
p3=p4+2*b*[ST;-CT];
p2=p3+2*l*[CT;ST];

% V=[[x;y] [x+v*CT;y+v*ST]];
% V_n=[[x;y] [x-vn*ST;y+vn*CT]];
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
plot(F_x1(1,:),F_x1(2,:),'Color','red','LineStyle',lstyle,'LineWidth',3)
plot(F_x2(1,:),F_x2(2,:),'Color','red','LineStyle',lstyle,'LineWidth',3)
plot(F_x3(1,:),F_x3(2,:),'Color','red','LineStyle',lstyle,'LineWidth',3)
plot(F_x4(1,:),F_x4(2,:),'Color','red','LineStyle',lstyle,'LineWidth',3)
plot(F_y1(1,:),F_y1(2,:),'Color','green','LineStyle',lstyle,'LineWidth',3)
plot(F_y2(1,:),F_y2(2,:),'Color','green','LineStyle',lstyle,'LineWidth',3)
plot(F_y3(1,:),F_y3(2,:),'Color','green','LineStyle',lstyle,'LineWidth',3)
plot(F_y4(1,:),F_y4(2,:),'Color','green','LineStyle',lstyle,'LineWidth',3)
% plot(V(1,:),V(2,:),'Color','Yellow','LineStyle',lstyle,'LineWidth',4)
% plot(V_n(1,:),V_n(2,:),'Color','blue','LineStyle',lstyle,'LineWidth',4)
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

function T=PID_vel(z,u,Xd)
vd=norm(Xd(4:5),2);
kp=4;
T=kp*(vd-z(4));
T=sign(T).*min(abs(T),4);

function u=PID_ST(u,DelT)
persistent uo kp kd ki
kp=.5; kd=.08; ki=.00;
if isempty(uo)
    uo=0;
end
der=(u-uo)/DelT;
u=satu(kp*u+kd*der+ki*uo,pi/6);
uo=u;

function  U=NMPC(Xd,z,u,DT,other,ke,kp1,max_time)
u(2:5)=Feedback_Linearizing(Xd,z,u);
% persistent delt NP Nopt opt_rate acc
% if isempty(delt)
    NP=15;   %% presiction steps NP*ST
    Nopt=20; %% Max Number of optimization
    opt_rate=1; %% learning rate of optimization method
    acc=1e-3;%% min accuracy of optimization method
%   PH=30;   %% predict horizon
% end
% DT=2*DT;
x=z(1);y=z(2); teta=z(3);
v=z(4);
% ke=.; kp1=0;kp2=.05; %%Uturn
tetad=Xd(3); yd=Xd(2); xd=Xd(1);
% ke=1.6; kp1=.1; 
kp2=.00;
%d=satu(sqrt((xd-x)^2+(yd-y)^2));
derr=atan2(yd-y,xd-x);
kp=kp1*exp(-kp2*norm(v));
delta=satu((kp*(1*sin(-teta+derr)))+ke*(sin(tetad-teta)),pi/6);
u(1)=delta*.9;%*pi/600;
u(2:5,1)=Feedback_Linearizing(Xd,z,u);%/0.1047;
U=optimize(z,Xd,u,DT*1,NP,Nopt,opt_rate,acc,other,max_time);
% delta=U(1);%satu(delt+(delta-delt)*.3);
% %delt=delta;
% U(1,1)=delta;%*600/pi;

% function y=satu(x,sat)
% y=sign(x)*min(sat,abs(x));

function U=optimize(x,xd,U_old,DT,NP,maxiteration,alfa,acc,other,max_time)
% U=optimize(z(1:6),Xd,u,DT,NP,Nopt,opt_rate,acc,other);
  Un=U_old;     n=1;%length(Un);
  maxiter=maxiteration;     dx=ones(size(U_old));     stop=0;
  fx=0;
  CG=0;
  CG_o=CG;
  bet=.5;
  tic
  MPC_time=0;
while stop<maxiter && norm(dx,2)>acc && MPC_time<max_time
    stop=stop+1;            
    fx = mainfun(xd,x,Un,DT,NP,other);
    dxo=dx;     CGo=CG;
    dx=dot_fun(Un,n,fx,xd,x,DT,NP,other);
    CG=-dx+bet*CG;
    UUU=Un-diag([1;.0;.0;.0;.0])*diag(CG)*dx;
%   UUU=Un-diag(alfa*[1;.01;.01;.01;.01])*dx*CG;
    UUU(1)=satu(UUU(1),pi/6);
    fxx=mainfun(xd,x,UUU,DT,NP,other);%%%%%%%f(x_new)
    if (fxx<fx)
    Un=UUU;
    alfa=alfa*1.8;
    else
    alfa=alfa*.4;
    end
    %%%%%%%%%%%%%%%%%%%%%%%
     if norm(CG)>.1
     bet=norm(CG)/norm(CGo);
     else
     bet=(norm(CG)/norm(CGo*dxo'));
     end
MPC_time=toc;
end   %%%while
  U=Un;
  
function J=mainfun(xd,x,u,DelT,NP,other)
% Xd=Xd';
J=0; %Np=15;
z=zeros(length(x),NP);
out=zeros(length(x)+17,NP);
z(:,1)=x;
dz0=z;
for i=1:NP
%   u(1)=PID_ST(Steering_con(Xd(:,i),z(:,i)),DelT);
%   u(2)=PID_vel(z(:,i),u,Xd(:,i));
    [out(:,i+1),dz]=wheel_model(z(:,i),u,dz0);
    z(:,i+1)=z(:,i)+ dz*DelT;
    x=z(:,i+1);
    J=J+1*sqrt((xd(1:2)-x(1:2))'*((xd(1:2)-x(1:2)) ) + .0*( (xd(4:6)-x(4:6))'*(xd(4:6)-x(4:6)) ) + 0*(u'*u) );
    dz0=dz;
%     J=J+sqrt((xd(1:2)-x(1:2))'*((xd(1:2)-x(1:2)) ) + .0*( (xd(4:6)-x(4:6))'*(xd(4:6)-x(4:6)) ) + 0*(u(2)*u(2)) );
end
    if abs(u(1))>pi/6
     J = J+1*abs(abs(u(1))-pi/6);
    end
% J=J+0.5*(xd(1:3)-x(1:3))'*((xd(1:3)-x(1:3)) );
J=J+1*(xd(1:2)-x(1:2))'*((xd(1:2)-x(1:2)));
J=1*J+.0002*stability_bound_cost(x,out(:,end),0);

function j = stability_bound_cost(x,other,bound)
beta=other(end); r=x(6);
j=beta^2+r^2;
beta_bound=.1; %shoud computed according to the speed, and friction feedback
beta_m=.2;
yaw_rate_bound=.8;
if (beta>(beta_m*r+beta_bound) || beta<(beta_m*r-beta_bound)) || (r>yaw_rate_bound || r<-yaw_rate_bound);
    j=1*j;
else
    j=0;
end
% j=min(j,1);
  
function dX=dot_fun(X,n,fx,xd,x,DT,NP,other)
E=zeros(size(X));
eps=10^-3;
dX=zeros(size(X));
for i=1:n
 E(i)=1;
 dX(i) = (1/eps)*(mainfun(xd,x,X+eps*E,DT,NP,other)-fx);
 E(i)=0;
end