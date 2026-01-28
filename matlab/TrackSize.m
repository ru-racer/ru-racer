length=2.1336*10^0;%m;
width=1.524*10^0;%m;
radiu_in=0.4572*10^0;%m
radiu_out=1.9812*10^0;%m                 
x1 = [0.381 0.381 -0.381 -0.381].*10^0;
x2=[-1.2954 -1.2954 -2.8194 -2.8194].*10^0;
y1 = [-1.5616428.*10^0 length+1.5616428.*10^0  length 0];
y2=[0 length length 0];
c1 = linspace(0,-pi,20);
c2 = linspace(0,pi,20);
c3 = linspace(asin(4/6.5)-pi/2,-pi,20);
c4 = linspace(pi/2-asin(4/6.5),pi,20);
% hold on
% plot(radiu_in*cos(c1)-0.8382*10^(3),radiu_in*sin(c1))
% plot(radiu_in*cos(c2)-0.8382*10^(3),radiu_in*sin(c2)+length)
% plot(radiu_out*cos(c3)-0.8382*10^(3),radiu_out*sin(c3))
% plot(radiu_out*cos(c4)-0.8382*10^(3),radiu_out*sin(c4)+length)
plot(x1,y1,'b',x2,y2,'b',radiu_out*cos(c4)-0.8382*10^(0),radiu_out*sin(c4)+length,'r'...
    ,radiu_out*cos(c3)-0.8382*10^(0),radiu_out*sin(c3),'r',...
    radiu_in*cos(c2)-0.8382*10^(0),radiu_in*sin(c2)+length,'b'...
    ,radiu_in*cos(c1)-0.8382*10^(0),radiu_in*sin(c1),'b')
   axis equal