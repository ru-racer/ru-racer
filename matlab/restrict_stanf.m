function test()
close all;
Del_T=0.001;
T=0:Del_T:1;
Xd=u_turn_path(T,5);
v0=norm(Xd(4:5,1),2);
% v0=5;
% Xd=[v0*T;0*sign(T);0*T;v0*sign(T);0*T;0*T];
z(:,1)=[Xd(1,1) -0.05 pi/2 1*v0 .0*v0 0 1*v0*ones(1,4)/.055]';
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
max_time=0.05;
for i=1:lT-1
if rem(i,PH)==1
    tic        
      u(:,i+1)=NMPC(Xd(:,i+10),z(:,i),u(:,i),Del_T,out(:,i),1.8,1,max_time);
      u(1,i+1)=PID_ST(u(1,i+1),0.1);
      u(1,i+1)=satu(u(1,i+1)-u(1,i),.2)+u(1,i);
%     u(:,i+1)=NMPC2(Xd(:,i),z(:,i),u(:,i),Del_T,out(:,i));
%     u(:,i)
    a=[T(i) toc];
else
    u(:,i+1)=u(:,i);
end
u(:,i+1)=[0;-2;2;-2;2];
[out(:,i+1),dz]=wheel_model1(z(:,i),u(:,i+1));
z(:,i+1)=z(:,i)+ dz*Del_T;
DELTA(i+1)=u(1,i+1);
end
% hold on
 figure
 plot(z(1,:),z(2,:),'r') %path?
 hold on;
%  plot(Xd(1,:),Xd(2,:),'k') %path
 xlabel('x');ylabel('yo')
 axis('equal') 
 for i=1:round(length(T)/4):length(T)
% %     hold off;
     robot(z(:,i),DELTA(1,i),max(z(1,:))/22,max(z(1,:))/16,'-',[0 0 0],out(:,i)/5);
%      robot(Xd(:,i),DELTA(1,i),max(z(1,:))/10,max(z(1,:))/15,'-.',[.3 .3 .3],out(:,i));
 end
%      figure
%      plot(T,out(end,:),T,out(1:4,:)) %123
   figure
   plot(out(3,10:end),out(end,10:end)) %stablity bound
%     figure
% % %  hold on
%     plot(T,DELTA,'b') %steering     
%    plot(T,out(23,:),'r*') %ddpsi
%    plot(T,z(6,:),'r*') %ddpsi
%    xlabel('t');ylabel('Deg')   
%    plot(T,z(3,:),'g')
%  xlabel('t');ylabel('\phi')
%  figure
%  plot(T,z(1,:),T,z(2,:),'r*') %path
%  xlabel('t');ylabel('m')
%  figure
%  plot(T,z(7:end,:),'r*') %Wv
%  xlabel('t');ylabel('W V')
%    figure
%    plot(T,z(4,:),'r*') %Vel
%    hold on;
%   plot(T,sqrt(Xd(4,:).^2+Xd(5,:).^2),'r*') %Vel
%   xlabel('t');ylabel('Vel')
%  figure
%  plot(T,out(9:12,:),'.') %fx
%  xlabel('t');ylabel('fx')
 figure
 plot(T,out(13:16,:),'O') %fy
 xlabel('t');ylabel('fy')
 hold on
 plot(T,out(1:4,:),'-.')  %alpha
%   figure
%   plot(T,out(1:4,:),'-.')  %alpha
%   xlabel('t');ylabel('\alpha')
%  figure
%  plot(T,out(5:8,:),'-.')  %lambda
%  xlabel('t');ylabel('\Lambda')

function [outs, dotz] = wheel_model1(z,u)
% % z=[x y psi v vn dotpsi dotphi_fl dotphi_fr dotphi_rl dotphi_rr]
% z=zeros(10,1);
% u=zeros(5,1);
persistent Par dotzo;
if isempty(Par)
    dotzo=0*z;  
    m=6.75; I=1;
    Tire=struct('mu',1,'r',.055,'alpha_max',.15);
    Car_size=struct('L_f',.108,'L_r',.300,'l_c',.2,'b',.290,'h',.2,'L',.408);
    Par=struct('M',diag([m m I]),'J',.1,'tire',Tire,'Car',Car_size,'g',9.8);
end
% T_m=u(2); T_b=u(3:end);
z1=z(1:3); z2=z(4:6); z3=z(7:10);
psi=z(3); c_p=cos(psi); s_p=sin(psi);
delta=u(1); c_d=cos(delta); s_d=sin(delta);
c_dp=cos(psi+delta); s_dp=sin(psi+delta);
SinCos=struct('s_p',s_p,'c_p',c_p,'s_d',s_d,'c_d',c_d,'s_dp',s_dp,'c_dp',c_dp);
J=[c_p -s_p 0;
   s_p c_p 0;
   0    0  1];
B_x=[c_d c_d 1 1;
    s_d s_d 0 0;    
    Par.Car.L_f*s_d-.5*Par.Car.b*c_d  Par.Car.L_f*s_d+.5*Par.Car.b*c_d -Par.Car.b Par.Car.b];
B_y=[s_d s_d 0  0;
     c_d c_d 1 1;
     Par.Car.L_f*c_d-.5*Par.Car.b*s_d  Par.Car.L_f*c_d+.5*Par.Car.b*s_d  -Par.Car.L_r -Par.Car.L_r];
C=[-z(5)*z(6)*Par.M(1,1) z(4)*z(6)*Par.M(1,1) 0]'*.1;
WVel=Wheel_velocity(z,u,Par,SinCos);
alpha=Alpha(z,u,Par);
lambda=Lambda(z,u,Par,WVel);

F_z=.5*Par.M(1,1)*[Par.g*Par.Car.L_r/Par.Car.L-Par.Car.h*(1*(dotzo(4)/Par.Car.L)+(1*dotzo(5)/Par.Car.b))
                   Par.g*Par.Car.L_r/Par.Car.L-Par.Car.h*(1*(dotzo(4)/Par.Car.L)-(1*dotzo(5)/Par.Car.b))
                   Par.g*Par.Car.L_f/Par.Car.L-Par.Car.h*(1*(dotzo(4)/Par.Car.L)-(1*dotzo(5)/Par.Car.b))
                   Par.g*Par.Car.L_f/Par.Car.L-Par.Car.h*(1*(dotzo(4)/Par.Car.L)+(1*dotzo(5)/Par.Car.b))];
               
F_x=fricx(lambda,F_z);
% ffx=abs(F_x./F_z)
% F_y=fricy(alpha,sqrt(1-(F_x./(F_z*.7)).^2));
F_y=fricy(alpha,F_z);
% ffy=abs(F_y./F_z)
dz1=J*z2;
dz2=(Par.M^-1)*(B_x*F_x+B_y*F_y-C);
dz3=(Par.J^-1)*(u(2:5)-.1*F_x-.00*z3);

dotz=[dz1
      dz2
      dz3];
dotzo=dotz;
outs=[alpha
      lambda
      F_x
      F_y      
      dotz
      atan(z(5)/z(4))];

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
for i=1:4
    if fx(i)*alpha(i)>0
        fx(i)=-fx(i);
    end
end
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
F_x=out(9:12);
F_y=out(13:16);
%function [ robot ] = robot( x,y,teta,l,b,lstyle,color)
%   Detailed explanation goes here
x=Z(1);y=Z(2);
teta=Z(3)+1*out(end);
v=l*Z(4)/5; vn=l*Z(5)/3;
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

function Xd=u_turn_path(t,vd)
 vs=vd; vu=2*vd;
 ls=2; ru=1.4;
 Tf=round(2*ls/vs+2*pi*ru/vu)+1;
 t1=ls/vs;
 t2=ls/vs+2*pi*ru/vu;
  wr=.5*vu/ru; R=ru;
%  teta1,teta2,tetadad1,tetadad2
for i=1:length(t)
%     [~,ind]=min(t(i)-tdata>0);
%     Xd(:,i)=[xdata(ind);ydata(ind);0;0;0;0];
 if t(i)<ls/vs
     Xd(:,i)=[ru;vs*t(i);pi/2;0;vs;0];
 elseif t(i)>=t1 && t(i)<t2
     Xd(:,i)=[ru*sin(pi/2-wr*(t(i)-t1));vs*t1+R*cos(pi/2-wr*(t(i)-t1));wr*(t(i)-t1)+pi/2;wr*R*cos(wr*(t(i)-t1));-wr*R*sin(wr*(t(i)-t1));wr*(t(i)-t1)];
     I2=i;
 elseif t(i)>=t2
     Xd(:,i)=[Xd(1,I2);Xd(2,I2)-vs*(t(i)-t2);-pi/2;0;-vs;0];
 elseif t(i)>=Tf
     Xd(:,i)=Xd(:,i-1);
 end
end

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
kp=.5; kd=.05; ki=.00;
if isempty(uo)
    uo=0;
end
der=(u-uo)/DelT;
u=satu(kp*u+kd*der+ki*uo,pi/4);
uo=u;

function  U=NMPC(Xd,z,u,DT,other,ke,kp1,max_time)
u(2:5)=Feedback_Linearizing(Xd,z,u);
% persistent delt NP Nopt opt_rate acc
% if isempty(delt)
    NP=11;   %% presiction steps NP*ST
    Nopt=25; %% Max Number of optimization
    opt_rate=.5; %% learning rate of optimization method
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
delta=satu((kp*(1*sin(-teta+derr)))+ke*(sin(tetad-teta)),pi/4);
u(1)=delta/4;%*pi/600;
u(2:5,1)=Feedback_Linearizing(Xd,z,u);%/0.1047;
U=optimize(z,Xd,u,DT*1,NP,Nopt,opt_rate,acc,other,max_time);
% delta=U(1);%satu(delt+(delta-delt)*.3);
% %delt=delta;
% U(1,1)=delta;%*600/pi;

function y=satu(x,sat)
y=sign(x)*min(sat,abs(x));

function U=optimize(x,xd,U_old,DT,NP,maxiteration,alfa,acc,other,max_time)
% U=optimize(z(1:6),Xd,u,DT,NP,Nopt,opt_rate,acc,other);
  Un=U_old;     n=length(Un);
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
    UUU(1)=satu(UUU(1),pi/4);
    fxx=mainfun(xd,x,UUU,DT,NP,other);%%%%%%%f(x_new)
    if (fxx<fx)
    Un=UUU;
    alfa=alfa*1.4;
    else
    alfa=alfa*.5;
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
    if abs(u(1))>pi/3
     J = J+1*abs(abs(u(1))-pi/3);
    end
% J=J+0.5*(xd(1:3)-x(1:3))'*((xd(1:3)-x(1:3)) );
J=J+1*(xd(1:2)-x(1:2))'*((xd(1:2)-x(1:2)));
J=1*J+20*stability_bound_cost(x,other,0);

function j = stability_bound_cost(x,other,bound)
beta=other(5); r=other(6);
j=beta^2+r^2;
beta_bound=.4; %shoud computed according to the speed, and friction feedback
beta_m=.2;
yaw_rate_bound=.5;
if (beta>(beta_m*r+beta_bound) || beta<(beta_m*r-beta_bound)) || (r>yaw_rate_bound || r<-yaw_rate_bound);
    j=1*j;
end

j=min(j,1);

function [outs, dotz] = wheel_model(z,u,dotzo)
% % z=[x y psi v vn dotpsi dotphi_fl dotphi_fr dotphi_rl dotphi_rr]
% z=zeros(10,1);
% u=zeros(5,1);
% persistent Par dotzo;
% if isempty(Par)
%     dotzo=0*z;  
    m=6.75; I=0.05;
    Tire.mu=1;
    Tire.r=0.055;
    Tire.alpha_max=0.15;
    Car_size.L_f=0.108;
    Car_size.L_r=0.300;
    Car_size.l_c=0.2;
    Car_size.b=0.29;
    Car_size.h=0.2;
    Car_size.L=0.108;Car_size.L_r+Car_size.L_f;
    Par.M=diag([m m I]);
    Par.tire=Tire;
    Par.Car=Car_size;
    Par.g=9.8;
    Par.J=0.02;
% end

z1=z(1:3); z2=z(4:6); z3=z(7:10)*0.1047;
psi=z(3); c_p=cos(psi); s_p=sin(psi);
delta=u(1); c_d=cos(delta); s_d=sin(delta);
c_dp=cos(psi+delta); s_dp=sin(psi+delta);
SinCos.s_p=s_p; SinCos.c_p=c_p;
SinCos.s_d=s_d; SinCos.c_d=c_d;
SinCos.s_dp=s_dp; SinCos.c_dp=c_dp;
J=[c_p -s_p 0;
   s_p c_p 0;
   0    0  1];
B_x=[c_d c_d 1 1;
    s_d s_d 0 0;    
    Par.Car.L_f*s_d-.5*Par.Car.b*c_d  Par.Car.L_f*s_d+.5*Par.Car.b*c_d -Par.Car.b Par.Car.b];
B_y=[s_d s_d 0  0;
     c_d c_d 1 1;
     Par.Car.L_f*c_d-.5*Par.Car.b*s_d  Par.Car.L_f*c_d+.5*Par.Car.b*s_d  -Par.Car.L_r -Par.Car.L_r];
C=[-z(5)*z(6)*Par.M(1,1) z(4)*z(6)*Par.M(1,1) 0]'*.2;
WVel=Wheel_velocity(z,u,Par,SinCos);
alpha=Alpha(z,u,Par);

lambda=Lambda(z,u,Par,WVel);

F_z=.5*Par.M(1,1)*[Par.g*Par.Car.L_r/Par.Car.L-Par.Car.h*((dotzo(4)/Par.Car.L)+(dotzo(5)/Par.Car.b))
                   Par.g*Par.Car.L_r/Par.Car.L-Par.Car.h*((dotzo(4)/Par.Car.L)-(dotzo(5)/Par.Car.b))
                   Par.g*Par.Car.L_f/Par.Car.L-Par.Car.h*((dotzo(4)/Par.Car.L)-(dotzo(5)/Par.Car.b))
                   Par.g*Par.Car.L_f/Par.Car.L-Par.Car.h*((dotzo(4)/Par.Car.L)+(dotzo(5)/Par.Car.b))];
F_x=fricx(lambda,F_z);
% ffx=abs(F_x./F_z)
% F_y=fricy(alpha,sqrt(1-(F_x./(F_z*.7)).^2));
F_y=fricy(alpha,F_z);
% ffy=abs(F_y./F_z)
dz1=J*z2;
dz2=(Par.M^-1)*(B_x*F_x+B_y*F_y-C);
dz3=(Par.J^-1)*(u(2:5)-0.1*F_x-0.01*z3);

dotz=[dz1
      dz2
      dz3];
dotzo=dotz;
outs=[alpha
      lambda
      F_x
      F_y      
      dotz
      atan(z(5)/z(4))];
  
  
function dX=dot_fun(X,n,fx,xd,x,DT,NP,other)
E=zeros(size(X));
eps=10^-3;
dX=zeros(size(X));
for i=1:n
 E(i)=1;
 dX(i) = (1/eps)*(mainfun(xd,x,X+eps*E,DT,NP,other)-fx);
 E(i)=0;
end