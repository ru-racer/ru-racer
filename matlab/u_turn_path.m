function Xd=u_turn_path(t,vd,ru)
 vs=vd; vu=1.8*vd;
 ls=3; %ru=4;
 Tf=round(2*ls/vs+2*pi*ru/vu)+.1;
 t1=ls/vs;
 t2=ls/vs+2*pi*ru/vu;
 t3=2*ls/vs+2*pi*ru/vu;
 t4=2*ls/vs+4*pi*ru/vu;

 wr=.5*vu/ru; R=ru;
%  teta1,teta2,tetadad1,tetadad2
for i=1:length(t)
 if t(i)>t4
     t(i)=rem( t(i) , t4 );
 end
%     [~,ind]=min(t(i)-tdata>0);
%     Xd(:,i)=[xdata(ind);ydata(ind);0;0;0;0];
 if t(i)<ls/vs
     Xd(:,i)=[ru;vs*t(i);pi/2;vs;0;0];
 elseif t(i)>=t1 && t(i)<t2
     Xd(:,i)=[ru*sin(pi/2-wr*(t(i)-t1));vs*t1+R*cos(pi/2-wr*(t(i)-t1));wr*(t(i)-t1)+pi/2;wr*R;0;wr*(t(i)-t1)];
     I2=i;
 elseif t(i)>=t2 && t(i)<t3
     Xd(:,i)=[Xd(1,I2);Xd(2,I2)-vs*(t(i)-t2);-pi/2;vs;0;0];
     I3=i;
 elseif t(i)>=t3 && t(i)<t4
     Xd(:,i)=[-ru*sin(pi/2-wr*(t(i)-t3));-R*cos(pi/2-wr*(t(i)-t3));wr*(t(i)-t3)+3*pi/2;wr*R;0;wr*(t(i)-t3)];
 end
end