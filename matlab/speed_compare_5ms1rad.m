test_name='5ms1ru_';
close all
% clear all
vd=5;
tf=2.42;
ru=1;
% [Tst,zst,ust,outst,Xdst]=restrict_stanf_Uturn(vd,tf,ru); %.4 .2
% [T,z,u,out,Xd]=Uturn(vd,tf,ru); %1.5 .5
 % hold on
F_S=12;
fig_path = figure;  %%%%path
ax_path = axes('Parent',fig_path,'FontSize',F_S);
box(ax_path,'on');hold(ax_path,'all');
ylim(ax_path,[-.2 max(Xd(2,:))+1]);
% xlim(axes1,[min(y)-1 max(y)+2]);
xlabel('X (m)','FontSize',F_S); ylabel({'Y (m)'},'FontSize',F_S); 
% title({'path tracking'}); 
plot(z(1,:),z(2,:),'Parent',ax_path,'LineWidth',2,'DisplayName','Proposed','Color',[0 0 0],'LineStyle','-');
plot(zst(1,:),zst(2,:),'Parent',ax_path,'LineWidth',2,'DisplayName','Restrict Boundary','Color',[0 0 1],'LineStyle','-.');
hold on;
plot(Xd(1,:),Xd(2,:),'Parent',ax_path,'LineWidth',2,'DisplayName','Desierd trajectory','Color',[.5 .5 .5],'LineStyle','--');
legend(ax_path,'show');
axis('equal') 
for i=1:round(length(T)/11):length(T)
% %  hold off;
     robot(z(:,i),u(1,i),max(z(1,:))/10,max(z(1,:))/15,'-',[0 0 0],out(:,i));     
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
plot3(T(10:end),out(end,10:end)/1.88,z(6,10:end)/3.64,'Parent',ax_phase,'LineWidth',2,'DisplayName','Proposed','Color',[0 0 0],'LineStyle','-');
hold on
plot3(Tst(10:end),outst(end,10:end)/2,zst(6,10:end)/4,'Parent',ax_phase,'LineWidth',2,'DisplayName','Restrict Boundary','Color',[.2 .2 1],'LineStyle','-.');
legend(ax_phase,'show'); % title({'path tracking'}); 
hold on
Finished=0;
FinishedSt=0;
for i=10:5:length(T)

if stability_bound_cost(out(end,i)/1.88,z(6,i)/3.64,0)>0
    plot3(T(i),out(end,i)/1.88,z(6,i)/3.64,'*r');
end
if stability_bound_cost(outst(end,i)/2,zst(6,i)/4,0)>0   
    plot3(Tst(i),outst(end,i)/2,zst(6,i)/4,'*b');
end
if (z(1,i)<0 && z(2,i)<0 && Finished==0)
    Finished=T(i)
    string = sprintf('T_{f P}= %.4f (s)',T(i));
    text(z(1,i)+.2,z(2,i)+.4,string,'Parent',ax_path,'fontsize',14)
    plot(Xd(1,i),Xd(2,i),'Parent',ax_path,'Marker','o','Markersize',6,'Color',[1 0 0],'Linewidth',3);
end
 if (zst(1,i)<0 && zst(2,i)<0 && FinishedSt==0)
    FinishedSt=Tst(i)
    string = sprintf('T_{f RB}= %.4f (s)',Tst(i));
    text(zst(1,i)-.2,zst(2,i),string,'Parent',ax_path,'fontsize',14,'Color',[.3 .3 1])
 end

end

fig_str = figure;  %
ax_str = axes('Parent',fig_str,'FontSize',F_S);
box(ax_str,'on');hold(ax_str,'all');
% ylim(axes1,[min(x)-.5 max(x)+2.5]);xlim(axes1,[min(y)-1 max(y)+2]);
xlabel('time (s)','FontSize',F_S); ylabel({'Steering angle (rad)'},'FontSize',F_S);
hold on
plot(T,u(1,1:end),'Parent',ax_str,'LineWidth',2,'DisplayName','Proposed','Color',[0 0 0],'LineStyle','-');
% plot(T,z(6,1:end)/2,'Parent',ax_str,'LineWidth',2,'DisplayName','Proposed_yaw rate','Color',[1 0 0],'LineStyle','-');
plot(T,ust(1,1:end)/1.02,'Parent',ax_str,'LineWidth',2,'DisplayName','Restrict Boundary','Color',[.4 .4 1],'LineStyle','-.');
% plot(T,zst(6,1:end)/4,'Parent',ax_str,'LineWidth',2,'DisplayName','Restrict Boundary_yaw rate','Color',[1 .4 .4],'LineStyle','-.');
legend(ax_str,'show'); % title({'path tracking'});


fig_error = figure;  %
ax_error = axes('Parent',fig_error,'FontSize',F_S);
box(ax_error ,'on');hold(ax_error ,'all');
% ylim(axes1,[min(x)-.5 max(x)+2.5]);xlim(axes1,[min(y)-1 max(y)+2]);
xlabel('time (s)','FontSize',F_S); ylabel({'Error tracking/velocity'},'FontSize',F_S);
hold on
plot(T,sqrt((Xd(1,:)-z(1,:)).^2+(Xd(1,:)-z(1,:)).^2),'Parent',ax_error ,'LineWidth',2,'DisplayName','Proposed_{e(m)}','Color',[0 0 0],'LineStyle','-');
plot(Tst,sqrt((Xdst(1,:)-zst(1,:)).^2+(Xdst(1,:)-zst(1,:)).^2),'Parent',ax_error ,'LineWidth',2,'DisplayName','Restrict Boundary_{e(m)}','Color',[.5 .5 1],'LineStyle','-');
plot(T,((Xd(4,:)-z(4,:))),'Parent',ax_error ,'LineWidth',2,'DisplayName','Proposed_{e_v(m/s)}','Color',[0 0 0],'LineStyle','-.');
plot(Tst,((Xdst(4,:)-zst(4,:))),'Parent',ax_error ,'LineWidth',2,'DisplayName','Restrict Boundary_{e_v(m/s)}','Color',[.6 .6 1],'LineStyle','-.');
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
xlabel('time (s)','FontSize',F_S); ylabel({'Longitude Force (Rad/s)'},'FontSize',F_S);
hold on
plot(T,z(4,:),'Parent',ax_v ,'LineWidth',2,'DisplayName','Proposed_{vx}','Color',[0 0 0],'LineStyle','-');
plot(T,z(5,:)/1.4,'Parent',ax_v ,'LineWidth',2,'DisplayName','Proposed_{vy}','Color',[0 0 0],'LineStyle','--');
plot(Tst,zst(4,:),'Parent',ax_v,'LineWidth',2,'DisplayName','RB_{vx}','Color',[0 0 1],'LineStyle','-');
plot(Tst,zst(5,:)/2,'Parent',ax_v ,'LineWidth',2,'DisplayName','RB_{vy}','Color',[0 0 1],'LineStyle','--');
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

savefig(fig_path,[test_name 'path.fig'])
savefig(fig_phase,[test_name 'phase.fig'])
savefig(fig_str,[test_name 'steer.fig'])
savefig(fig_error,[test_name 'error.fig'])
savefig(fig_wv,[test_name 'wheel.fig'])
savefig(fig_Fy,[test_name 'fy.fig'])
savefig(fig_Fx,[test_name 'fx.fig'])
savefig(fig_v,[test_name 'velocity.fig'])
savefig(fig_a,[test_name 'sidesleep.fig'])

%      figure
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
% figure
% % plot(T,out(9:12,:),'.') %fx
% hold on
% plot(Tst,outst(9:12,:),'.') %fx
% xlabel('t');ylabel('fx')
% figure
% plot(T,out(13:16,:),'O') %fy
% hold on
% plot(Tst,outst(13:16,:),'O') %fy
% xlabel('t');ylabel('fy')
% %   figure
% %   plot(T,out(1:4,:),'-.')  %alpha
% %   xlabel('t');ylabel('\alpha')
% %  figure
% %  plot(T,out(5:8,:),'-.')  %lambda
% %  xlabel('t');ylabel('\Lambda')
% figure 
% plot(T,z(4:5,:),'-',T,zst(4:5,:),'-.')  %lambda
