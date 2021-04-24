function u_x=Feedback_Linearizing(Xd,z,u)
% persistent Par;
% if isempty(Par)
% Xd=Xdo';
    dotzo=0*z;  
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
    g=9.8;
% end
% T_m=u(2); T_b=u(3:end);
z1=z(1:3); z2=z(4:6); z3=z(7:10);
psi=z(3); c_p=cos(psi); s_p=sin(psi);
delta=u(1); c_d=cos(delta); s_d=sin(delta);
c_dd=cos(Xd(3)); s_dd=sin(Xd(3));
% SinCos=struct('s_p',s_p,'c_p',c_p,'s_d',s_d,'c_d',c_d,'s_dp',s_dp,'c_dp',c_dp);
% J=[c_p -s_p 0;
%    s_p c_p 0;
%    0    0  1];
B_x=[c_d c_d 1 1;
    s_d s_d 0 0;
    Par.Car.L_f*s_d-.5*Par.Car.b*c_d  Par.Car.L_f*s_d+.5*Par.Car.b*c_d -Par.Car.b Par.Car.b];
A=[c_p -s_p -Par.Car.l_c*s_p;   s_p  c_p  Par.Car.l_c*c_p];
P=[z(1)+Par.Car.l_c*c_p;   z(2)-Par.Car.l_c*s_p];
Omega=A*(Par.M^-1)*B_x;
e=Xd(1:2)-P;
Ad=[c_dd -s_dd -Par.Car.l_c*s_dd;   s_dd  c_dd  Par.Car.l_c*c_dd];
% dot_P=A*z2;
% dot_e=Ad*Xd(4:6)-dot_P;
vd=norm(Xd(4:5),2);
dot_e=vd-z2(1);
kp=.01; kd=1;
u_x=Omega\(kp*e)+(kd*dot_e);
for i=1:4
 u_x(i)=sign(u_x(i))*min(abs(u_x(i)),10);
end
% u_x=(((u_x/(m*g)))+z(4))/Tire.r;