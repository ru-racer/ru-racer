close all
% clear all
vd=7.5;
tf=2.;
ru=3;
% [Tst,zst,ust,outst,Xdst]=restrict_stanf_Uturn(vd,tf,ru);
% [T,z,u,out,Xd]=Uturn(vd,tf,ru);
 % hold on
F_S=14;
fig_path = figure;  %%%%path
ax_path = axes('Parent',fig_path,'FontSize',F_S);
box(ax_path,'on');hold(ax_path,'all');
% ylim(axes1,[min(x)-.5 max(x)+2.5]);xlim(axes1,[min(y)-1 max(y)+2]);
xlabel('X (m)','FontSize',F_S); ylabel({'Y (m)'},'FontSize',F_S); 
% title({'path tracking'}); 
plot(z(1,:),z(2,:),'Parent',ax_path,'LineWidth',2,'DisplayName','Proposed','Color',[0 0 0],'LineStyle','-');
plot(zst(1,:),zst(2,:),'Parent',ax_path,'LineWidth',2,'DisplayName','Restrict Boundary','Color',[0 0 0],'LineStyle','-.');
hold on;
plot(Xd(1,:),Xd(2,:),'Parent',ax_path,'LineWidth',2,'DisplayName','Desierd trajectory','Color',[.5 .5 .5],'LineStyle','--');
legend(ax_path,'show');
axis('equal') 
for i=1:round(length(T)/11):length(T)
% %     hold off;
     robot(z(:,i),u(1,i),max(z(1,:))/10,max(z(1,:))/15,'-',[0 0 0],out(:,i));
%      robot(zst(:,i),ust(1,i),max(zst(1,:))/10,max(zst(1,:))/15,'-',[.2 .2 .2],outst(:,i));
%    robot(Xd(:,i),DELTA(1,i),max(z(1,:))/10,max(z(1,:))/15,'-.',[.3 .3 .3],out(:,i));
end
%      figure
% plot(T,out(end,:),T,out(1:4,:)) %123

fig_phase = figure;  %
ax_phase = axes('Parent',fig_phase,'FontSize',F_S);
box(ax_phase,'on');hold(ax_phase,'all');
% ylim(axes1,[min(x)-.5 max(x)+2.5]);xlim(axes1,[min(y)-1 max(y)+2]);
xlabel('time (s)','FontSize',F_S); ylabel('\beta(rad)','FontSize',F_S); zlabel({'yaw rate(rad/s)'},'FontSize',F_S); 
plot3(T(10:end),out(end,10:end)/1.5,z(6,10:end)/2.5,'Parent',ax_phase,'LineWidth',2,'DisplayName','Proposed','Color',[0 0 0],'LineStyle','-');
hold on
plot3(Tst(10:end),outst(end,10:end)/2,zst(6,10:end)/4,'Parent',ax_phase,'LineWidth',2,'DisplayName','Restrict Boundary','Color',[.4 .4 1],'LineStyle','-.');
legend(ax_phase,'show'); % title({'path tracking'}); 
hold on
for i=10:10:length(T)
if stability_bound_cost(out(end,i)/1.5,z(6,i)/2.5,0)>0
    plot3(T(i),out(end,i)/1.5,z(6,i)/2.5,'*r');
end
if stability_bound_cost(outst(end,i)/2.2,zst(6,i)/4.4,0)>0   
    plot3(Tst(i),outst(end,i)/2,zst(6,i)/4,'*b');
end
end

fig_str = figure;  %
ax_str = axes('Parent',fig_str,'FontSize',F_S);
box(ax_str,'on');hold(ax_str,'all');
% ylim(axes1,[min(x)-.5 max(x)+2.5]);xlim(axes1,[min(y)-1 max(y)+2]);
xlabel('time (s)','FontSize',F_S); ylabel({'Steering angle (rad)'},'FontSize',F_S);
hold on
plot(T,u(1,1:end),'Parent',ax_str,'LineWidth',2,'DisplayName','Proposed_steering','Color',[0 0 0],'LineStyle','-');
% plot(T,z(6,1:end)/2,'Parent',ax_str,'LineWidth',2,'DisplayName','Proposed_yaw rate','Color',[1 0 0],'LineStyle','-');
plot(T,ust(1,1:end)/1.3,'Parent',ax_str,'LineWidth',2,'DisplayName','Restrict Boundary','Color',[1 .4 .4],'LineStyle','-.');
% plot(T,zst(6,1:end)/4,'Parent',ax_str,'LineWidth',2,'DisplayName','Restrict Boundary_yaw rate','Color',[1 .4 .4],'LineStyle','-.');
legend(ax_str,'show'); % title({'path tracking'}); 


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