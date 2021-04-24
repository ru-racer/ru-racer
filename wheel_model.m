function [outs, dotz] = wheel_model(z,u,dotzo)
% % z=[x y psi v vn dotpsi dotphi_fl dotphi_fr dotphi_rl dotphi_rr]
% z=zeros(10,1);
% u=zeros(5,1);
% persistent Par dotzo;
% if isempty(Par)
%     dotzo=0*z;  
    m=6.75; I=3;
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
    Par.J=0.5;
% end

z1=z(1:3); z2=z(4:6); z3=z(7:10);%*0.1047;
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
B_y=[-s_d -s_d 0  0;
     c_d c_d 1 1;
     Par.Car.L_f*c_d-.5*Par.Car.b*s_d  Par.Car.L_f*c_d+.5*Par.Car.b*s_d  -Par.Car.L_r -Par.Car.L_r];

C=[-z(5)*z(6)*Par.M(1,1) -z(4)*z(6)*Par.M(1,1) 0]';
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
dotz(4:6)=sign(dotz(4:6)).*min(abs(dotz(4:6)),20);
% dotzo=dotz;
outs=[alpha
      lambda
      F_x
      F_y      
      dotz
      atan(z(5)/z(4))];  
  
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
 
 function al = Alpha(z,u,Par)
if(z(4)==0)
    al=[0;0;0;0];
else
alf=atan((z(5)+Par.Car.L_f*z(6))/z(4))-u(1);
alr=atan((z(5)-Par.Car.L_r*z(6))/z(4));
al=[alf;alf;alr;alr];
end