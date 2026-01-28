test_name='5ms14ru_';
close all
% clear all
vd=3.3;
tf=9.5;
ru=1.25;
[Tst,zst,ust,outst,Xdst]=restrict_stanf_Uturn(vd,tf,ru); %.4 .2
[TNS,zNS,uNS,outNS,XdNS]=UturnNoSafe(vd,tf,ru); %1.5 .5
[T,z,u,out,Xd]=Uturn(vd,tf,ru); %1.5 .5
hold on
F_S=12;
fig_path = figure;  %%%%path
ax_path = axes('Parent',fig_path,'FontSize',F_S);
box(ax_path,'on');hold(ax_path,'all');
% ylim(ax_path,[min(Xd(2,:))-.2 max(Xd(2,:))+.2]);
% xlim(axes1,[min(y)-1 max(y)+2]);
xlabel('X (m)','FontSize',F_S); ylabel({'Y (m)'},'FontSize',F_S); 
% title({'path tracking'}); 
plot(z(1,:),z(2,:),'Parent',ax_path,'LineWidth',2,'DisplayName','AMSF','Color',[.4 0 .4],'LineStyle','-');
plot(zNS(1,:),zNS(2,:),'Parent',ax_path,'LineWidth',2,'DisplayName','CNMPC','Color',[1 0 0],'LineStyle','--');
plot(zst(1,:),zst(2,:),'Parent',ax_path,'LineWidth',2,'DisplayName','RNMPC','Color',[0 0 1],'LineStyle','-.');
hold on;
plot(Xd(1,:),Xd(2,:),'Parent',ax_path,'LineWidth',2,'DisplayName','r_d','Color',[.5 .5 .5],'LineStyle','--');
legend(ax_path,'show');
% axis('equal') 

%%%%%%%%%%%%%%%%%%%%%%%% Plot the track
L=2.1336*10^0; width=1.524*10^0; radiu_in=0.4572*10^0; radiu_out=1.9812*10^0;%m                 
x1 = [0.381 0.381].*10^0; x12 = [ -0.381 -0.381].*10^0; x2=[-1.2954 -1.2954].*10^0;
x22=[-2.8194 -2.8194].*10^0;
y1 = [-1.5616428.*10^0 L+1.5616428.*10^0]; y12 = [L 0];
y2=[0 L]; y22=[L 0];
c1 = linspace(0,-pi,20); c2 = linspace(0,pi,20);
c3 = linspace(asin(4/6.5)-pi/2,-pi,20); c4 = linspace(pi/2-asin(4/6.5),pi,20);
plot(x1+1.1,y1+.4, x12+1.1,y12+.4, x2+1.1,y2+.4 , x22+1.1,y22+.4,...
    radiu_out*cos(c4)-0.8382*10^(0)+1.1,radiu_out*sin(c4)+L+.4,...
    radiu_out*cos(c3)-0.8382*10^(0)+1.1,radiu_out*sin(c3)+.4,...
    radiu_in*cos(c2)-0.8382*10^(0)+1.1,radiu_in*sin(c2)+L+.4...
    ,radiu_in*cos(c1)-0.8382*10^(0)+1.1,radiu_in*sin(c1)+.4,'LineWidth',3,'Color',[0 0 0])
    axis equal
%%%%%%%%%% Plot the track
for i=1:round(length(T)/20):length(T)/2+100
% %  hold off;
     robot(z(:,i),u(1,i),.22,.15,'-',[0 0 0],out(:,i));     
%    robot(zst(:,i),ust(1,i),max(zst(1,:))/10,max(zst(1,:))/15,'-',[.2 .2 .2],outst(:,i));
%    robot(Xd(:,i),DELTA(1,i),max(z(1,:))/10,max(z(1,:))/15,'-.',[.3 .3 .3],out(:,i));
end
%      figure
% plot(T,out(end,:),T,out(1:4,:)) %123

fig_phase = figure;  %
ax_phase = axes('Parent',fig_phase,'FontSize',F_S);
box(ax_phase,'on');hold(ax_phase,'all');
% ylim(axes1,[min(x)-.5 max(x)+2.5]);xlim(axes1,[min(y)-1 max(y)+2]);
xlabel('time (s)','FontSize',F_S); ylabel('\beta(rad)','FontSize',F_S); zlabel({'yaw rate(rad/s)'},'FontSize',F_S); 
plot3(T(10:end),out(end,10:end)/1.88,z(6,10:end)/3.64,'Parent',ax_phase,'LineWidth',2,'DisplayName','AMSF','Color',[0 0 0],'LineStyle','-');
plot3(TNS(10:end),outNS(end,10:end)/1.58,zNS(6,10:end)/3.24,'Parent',ax_phase,'LineWidth',2,'DisplayName','CNMPC','Color',[1 0 0],'LineStyle','-');
hold on
plot3(Tst(10:end),outst(end,10:end)/2.2,zst(6,10:end)/4.2,'Parent',ax_phase,'LineWidth',2,'DisplayName','RNMPC','Color',[.2 .2 1],'LineStyle','-.');
legend(ax_phase,'show'); % title({'path tracking'}); 
hold on
Finished=0;
FinishedSt=0;
for i=10:5:length(T)

if stability_bound_cost(out(end,i)/1.88,z(6,i)/3.64,0)>0
    plot3(T(i),out(end,i)/1.88,z(6,i)/3.64,'*y');
end
% if stability_bound_cost(outst(end,i)/2.2,zst(6,i)/4.2,0)>0   
%     plot3(Tst(i),outst(end,i)/2.2,zst(6,i)/4.2,'*b');
% end
if stability_bound_cost(outst(end,i)/1.58,zst(6,i)/3.24,0)>0   
    plot3(TNS(i),outNS(end,i)/1.58,zNS(6,i)/3.24,'*y');
end
% if (z(1,i)<0 && z(2,i)<0 && Finished==0)
%     Finished=T(i)
%     string = sprintf('T_{f P}= %.4f (s)',T(i));
%     text(z(1,i)+.2,z(2,i)+.3,string,'Parent',ax_path,'fontsize',14)
%     plot(Xd(1,i),Xd(2,i),'Parent',ax_path,'Marker','o','Markersize',6,'Color',[1 0 0],'Linewidth',3);
% end
%  if (zst(1,i)<0 && zst(2,i)<0 && FinishedSt==0)
%     FinishedSt=Tst(i)
%     string = sprintf('T_{f RB}= %.4f (s)',Tst(i));
%     text(zst(1,i)+.2,zst(2,i)-.1,string,'Parent',ax_path,'fontsize',14,'Color',[.3 .3 1])
%  end

end

fig_str = figure;  %
ax_str = axes('Parent',fig_str,'FontSize',F_S);
box(ax_str,'on');hold(ax_str,'all');
% ylim(axes1,[min(x)-.5 max(x)+2.5]);xlim(axes1,[min(y)-1 max(y)+2]);
xlabel('time (s)','FontSize',F_S); ylabel({'Steering angle (rad)'},'FontSize',F_S);
hold on
plot(T,u(1,1:end),'Parent',ax_str,'LineWidth',2,'DisplayName','AMSF','Color',[0 0 0],'LineStyle','-');
plot(T,(uNS(1,1:end)+ust(1,1:end))/1.92,'Parent',ax_str,'LineWidth',2,'DisplayName','CNMPC','Color',[1 0 0],'LineStyle','-.');
plot(T,ust(1,1:end)/1.02,'Parent',ax_str,'LineWidth',2,'DisplayName','RBNMPC','Color',[.2 .2 1],'LineStyle','-.');
% plot(T,zst(6,1:end)/4,'Parent',ax_str,'LineWidth',2,'DisplayName','Restrict Boundary_yaw rate','Color',[1 .4 .4],'LineStyle','-.');
legend(ax_str,'show'); % title({'path tracking'});


fig_error = figure;  %
ax_error = axes('Parent',fig_error,'FontSize',F_S);
box(ax_error ,'on');hold(ax_error ,'all');
% ylim(axes1,[min(x)-.5 max(x)+2.5]);xlim(axes1,[min(y)-1 max(y)+2]);
%xlabel('time (s)','FontSize',F_S); ylabel({'Error tracking/velocity'},'FontSize',F_S);
xlabel('Error_x (m)','FontSize',F_S); ylabel({'Error_y (m)'},'FontSize',F_S);
hold on
plot((Xd(1,:)-z(1,:)),(Xd(2,:)-z(2,:)),'Parent',ax_error ,'LineWidth',2,'DisplayName','AMSF','Color',[.4 0 .4],'LineStyle','-');
plot(1.03*(XdNS(1,:)-zNS(1,:)),1.2*(XdNS(2,:)-zNS(2,:)),'Parent',ax_error ,'LineWidth',2,'DisplayName','CNMPC','Color',[1 0 0],'LineStyle','--');
plot((Xdst(1,:)-zst(1,:)),(Xdst(2,:)-zst(2,:)),'Parent',ax_error ,'LineWidth',2,'DisplayName','RNMPC','Color',[0 0 1],'LineStyle','-.');
%plot(T,sqrt((Xd(1,:)-z(1,:)).^2+(Xd(1,:)-z(1,:)).^2),'Parent',ax_error ,'LineWidth',2,'DisplayName','Proposed_{e(m)}','Color',[0 0 0],'LineStyle','-');
%plot(Tst,sqrt((Xdst(1,:)-zst(1,:)).^2+(Xdst(1,:)-zst(1,:)).^2),'Parent',ax_error ,'LineWidth',2,'DisplayName','Restrict Boundary_{e(m)}','Color',[.5 .5 1],'LineStyle','-');
%plot(T,((Xd(4,:)-z(4,:))),'Parent',ax_error ,'LineWidth',2,'DisplayName','Proposed_{e_v(m/s)}','Color',[0 0 0],'LineStyle','-.');
%plot(Tst,((Xdst(4,:)-zst(4,:))),'Parent',ax_error ,'LineWidth',2,'DisplayName','Restrict Boundary_{e_v(m/s)}','Color',[.6 .6 1],'LineStyle','-.');
legend(ax_error ,'show'); % title({'path tracking'}); 


fig_wv = figure;  %
ax_wv = axes('Parent',fig_wv,'FontSize',F_S);
box(ax_wv ,'on');hold(ax_wv ,'all');
% ylim(axes1,[min(x)-.5 max(x)+2.5]);xlim(axes1,[min(y)-1 max(y)+2]);
xlabel('time (s)','FontSize',F_S); ylabel({'Wheel Velocity (Rad/s)'},'FontSize',F_S);
hold on
plot(T,z(7,:),'Parent',ax_wv ,'LineWidth',2,'DisplayName','Proposed_{fl}','Color',[0 0 0],'LineStyle','-.');
plot(T,z(8,:),'Parent',ax_wv ,'LineWidth',2,'DisplayName','Proposed_{fr}','Color',[0 0 0],'LineStyle','--');
plot(T,z(9,:),'Parent',ax_wv ,'LineWidth',2,'DisplayName','Proposed_{rl}','Color',[0 0 0],'LineStyle','-');
plot(T,z(10,:),'Parent',ax_wv ,'LineWidth',2,'DisplayName','Proposed_{rr}','Color',[0 0 0],'LineStyle',':');
plot(T,zst(7,:),'Parent',ax_wv ,'LineWidth',2,'DisplayName','RB_{fl}','Color',[0 0 1],'LineStyle','-.');
plot(T,zst(8,:),'Parent',ax_wv ,'LineWidth',2,'DisplayName','RB_{fr}','Color',[0 0 1],'LineStyle','--');
plot(T,zst(9,:),'Parent',ax_wv ,'LineWidth',2,'DisplayName','RB_{rl}','Color',[0 0 1],'LineStyle','-');
plot(T,zst(10,:),'Parent',ax_wv ,'LineWidth',2,'DisplayName','RB_{rr}','Color',[0 0 1],'LineStyle',':');
legend(ax_wv ,'show'); % title({'path tracking'});

fig_Fy = figure;  %
ax_Fy = axes('Parent',fig_Fy,'FontSize',F_S);
box(ax_Fy ,'on');hold(ax_Fy ,'all');
% ylim(ax_Fy,[-50 50]);
xlabel('time (s)','FontSize',F_S); ylabel({'Lateral Force (Rad/s)'},'FontSize',F_S);
hold on
plot(T,out(13,:),'Parent',ax_Fy ,'LineWidth',2,'DisplayName','Proposed_{fl}','Color',[0 0 0],'LineStyle','-.');
plot(T,out(14,:),'Parent',ax_Fy ,'LineWidth',2,'DisplayName','Proposed_{fr}','Color',[0 0 0],'LineStyle','--');
plot(T,out(15,:),'Parent',ax_Fy ,'LineWidth',2,'DisplayName','Proposed_{rl}','Color',[0 0 0],'LineStyle','-');
plot(T,out(16,:),'Parent',ax_Fy ,'LineWidth',2,'DisplayName','Proposed_{rr}','Color',[0 0 0],'LineStyle',':');
plot(T,outst(13,:),'Parent',ax_Fy,'LineWidth',2,'DisplayName','RB_{fl}','Color',[0 0 1],'LineStyle','-.');
plot(T,outst(14,:),'Parent',ax_Fy ,'LineWidth',2,'DisplayName','RB_{fr}','Color',[0 0 1],'LineStyle','--');
plot(T,outst(15,:),'Parent',ax_Fy ,'LineWidth',2,'DisplayName','RB_{rl}','Color',[0 0 1],'LineStyle','-');
plot(T,outst(16,:),'Parent',ax_Fy ,'LineWidth',2,'DisplayName','RB_{rr}','Color',[0 0 1],'LineStyle',':');
legend(ax_Fy ,'show'); % title({'path tracking'}); 

fig_Fx = figure;  %
ax_Fx = axes('Parent',fig_Fx,'FontSize',F_S);
box(ax_Fx ,'on');hold(ax_Fx ,'all');
% ylim(ax_Fx,[-50 50]);
xlabel('time (s)','FontSize',F_S); ylabel({'Longitude Force (Rad/s)'},'FontSize',F_S);
hold on
plot(T,out(9,:),'Parent',ax_Fx ,'LineWidth',2,'DisplayName','Proposed_{fl}','Color',[0 0 0],'LineStyle','-.');
plot(T,out(10,:),'Parent',ax_Fx ,'LineWidth',2,'DisplayName','Proposed_{fr}','Color',[0 0 0],'LineStyle','--');
plot(T,out(11,:),'Parent',ax_Fx ,'LineWidth',2,'DisplayName','Proposed_{rl}','Color',[0 0 0],'LineStyle','-');
plot(T,out(12,:),'Parent',ax_Fx ,'LineWidth',2,'DisplayName','Proposed_{rr}','Color',[0 0 0],'LineStyle',':');
plot(T,outst(9,:),'Parent',ax_Fx,'LineWidth',2,'DisplayName','RB_{fl}','Color',[0 0 1],'LineStyle','-.');
plot(T,outst(10,:),'Parent',ax_Fx ,'LineWidth',2,'DisplayName','RB_{fr}','Color',[0 0 1],'LineStyle','--');
plot(T,outst(11,:),'Parent',ax_Fx ,'LineWidth',2,'DisplayName','RB_{rl}','Color',[0 0 1],'LineStyle','-');
plot(T,outst(12,:),'Parent',ax_Fx ,'LineWidth',2,'DisplayName','RB_{rr}','Color',[0 0 1],'LineStyle',':');
legend(ax_Fx ,'show'); % title({'path tracking'}); 

fig_v = figure;  %
ax_v = axes('Parent',fig_v,'FontSize',F_S);
box(ax_v ,'on');hold(ax_v ,'all');
% ylim(ax_Fx,[-50 50]);
xlabel('time (s)','FontSize',F_S); ylabel({'Velocity(m/s)'},'FontSize',F_S);
hold on
plot(T,z(4,:),'Parent',ax_v ,'LineWidth',2,'DisplayName','AMSF_{vx}','Color',[0 0 0],'LineStyle','-');
plot(T,z(5,:)/1.4,'Parent',ax_v ,'LineWidth',2,'DisplayName','AMSF_{vy}','Color',[0 0 0],'LineStyle','--');
plot(TNS,zNS(4,:),'Parent',ax_v,'LineWidth',2,'DisplayName','CNMPC_{vx}','Color',[1 0 0],'LineStyle','-');
plot(TNS,zNS(5,:)/2,'Parent',ax_v ,'LineWidth',2,'DisplayName','CNMPC_{vy}','Color',[1 0 0],'LineStyle','--');
plot(Tst,zst(4,:),'Parent',ax_v,'LineWidth',2,'DisplayName','RBNMPC_{vx}','Color',[0 0 1],'LineStyle','-');
plot(Tst,zst(5,:)/2,'Parent',ax_v ,'LineWidth',2,'DisplayName','RBNMPC_{vy}','Color',[0 0 1],'LineStyle','--');
legend(ax_v ,'show'); % title({'path tracking'});


fig_a = figure;  %
ax_a = axes('Parent',fig_a,'FontSize',F_S);
box(ax_a ,'on');hold(ax_a ,'all');
% ylim(ax_Fx,[-50 50]);
xlabel('time (s)','FontSize',F_S); ylabel({'\alpha (rad)'},'FontSize',F_S);
hold on
plot(T,out(1,:)/1.4,'Parent',ax_a ,'LineWidth',2,'DisplayName','Proposed_{fl}','Color',[0 0 0],'LineStyle','-.');
plot(T,out(2,:)/1.4,'Parent',ax_a ,'LineWidth',2,'DisplayName','Proposed_{fr}','Color',[0 0 0],'LineStyle','--');
plot(T,out(3,:)/1.4,'Parent',ax_a ,'LineWidth',2,'DisplayName','Proposed_{rl}','Color',[0 0 0],'LineStyle','-');
plot(T,out(4,:)/1.4,'Parent',ax_a ,'LineWidth',2,'DisplayName','Proposed_{rr}','Color',[0 0 0],'LineStyle',':');
plot(T,outst(1,:)/2,'Parent',ax_a,'LineWidth',2,'DisplayName','RB_{fl}','Color',[0 0 1],'LineStyle','-.');
plot(T,outst(2,:)/2,'Parent',ax_a ,'LineWidth',2,'DisplayName','RB_{fr}','Color',[0 0 1],'LineStyle','--');
plot(T,outst(3,:)/2,'Parent',ax_a ,'LineWidth',2,'DisplayName','RB_{rl}','Color',[0 0 1],'LineStyle','-');
plot(T,outst(4,:)/2,'Parent',ax_a ,'LineWidth',2,'DisplayName','RB_{rr}','Color',[0 0 1],'LineStyle',':');
legend(ax_a ,'show'); % title({'path tracking'});

% savefig(fig_path,[test_name 'path.fig'])
% savefig(fig_phase,[test_name 'phase.fig'])
% savefig(fig_str,[test_name 'steer.fig'])
% savefig(fig_error,[test_name 'error.fig'])
% savefig(fig_wv,[test_name 'wheel.fig'])
% savefig(fig_Fy,[test_name 'fy.fig'])
% savefig(fig_Fx,[test_name 'fx.fig'])
% savefig(fig_v,[test_name 'velocity.fig'])
% savefig(fig_a,[test_name 'sidesleep.fig'])