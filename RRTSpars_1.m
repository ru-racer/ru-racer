% function RRT()
clear all
close all
max_itter=50000;
map='map3.bmp';
% R1=eval('RRT1(max_itter,[-.85 0 .5;-.85 .5 .5;-.85 1 .5;-.85 1.5 .5;-.85 2.05 .5])');
% R1=eval('RRT1(max_itter,[0 0 0])');
% R1=eval('RRTBike(max_itter,[-.85 0 .5;-.85 .5 .5;-.85 1 .5;-.85 1.5 .5;-.85 2.05 .5])');
R1=eval('RRTBike(max_itter,map,"o")');
% R1=eval('RRTBike(max_itter,[-.85 0 .5;-.85 .5 .5;-.85 1 .5;-.85 1.5 .5;-.85 2.05 .5],"o")');
% R1=eval('RRTBike_PC(max_itter,[-.85 0 .5;-.85 .5 .5;-.85 1 .5;-.85 1.5 .5;-.85 2.05 .5])');
% R1=eval('RRT1(max_itter,[1 1 1])');
tic
% hold on;
lastCost=[];
for i=1:max_itter
        R1.delta_near =1+(.5-rand);
        newNode=R1.sample(R1.goal);
        NearestInd=R1.Nearest(newNode);
        Neighbors = R1.Neighbors(newNode,NearestInd);
        min_ind = R1.find_NewParent(Neighbors,newNode,NearestInd);
        [isFeasible,newNode] = R1.Steer(min_ind,newNode);
        if (isFeasible)
            new_ind = R1.add_Node(min_ind,newNode);
%             if new_ind > 0 && ~isempty(R1.goal_reached)% && R1.delta_near<.1
%                R1.delta_near =.1*rand;
%                R1.rewire(new_ind, Neighbors, min_ind)
%             end
            if new_ind > 0
                R1.Drain(new_ind,min_ind);
            end
        end
    if(mod(i,100) == 0)
        disp([num2str(i) ' iterations ' num2str(R1.nodes_added) ' nodes in ' num2str(toc) ' Distance ' num2str(R1.distance(R1.best_node)) ...
            ' Cost ' num2str(min(R1.cumcost(R1.goal_reached)))   ' Path ' num2str(length(R1.goal_reached)) ...
              '  Rewired ' num2str(R1.num_rewire) ]);          
%           R1.plot2d()
%           pause(.001)
%           clf('reset')
    end
end
    F_S=12;
%     fig = figure;  %
    A=imread('track.jpg');%trackhighresolution,tracklowresolution
    fig=imshow(A);hold on;
%     ax = axes('Parent',fig,'FontSize',F_S,'YTick',[],'XTick',[]);
%     box(ax,'off');hold(ax,'all');    
    %xlabel('Spars-RRT: 3584 nodes in 10000 iterations in 254.16 s, Cost 8.3603','FontSize',F_S);    
    %title('\Delta_{drain}=.1, \Delta_{near}=.5')
    hold on
    R1.Implot2d()
    if ~isempty(R1.best_path)
    mm=R1.FollowTrajectory(R1.tree_input(:,R1.best_path),R1.tree(:,R1.best_path),R1.time(R1.best_path));
%     plot(370*mm.All_States(2,:)+687,390*mm.All_States(1,:)+1125,'b.')
    surface([360*mm.All_States(2,:)+687-360*(mm.All_States(2,:)<0).*mm.All_States(2,:);370*mm.All_States(2,:)+687],[390*mm.All_States(1,:)+1125;390*mm.All_States(1,:)+1125],[mm.All_States(4,:);mm.All_States(4,:)],[mm.All_States(4,:);mm.All_States(4,:)],...
        'facecol','no',...
        'edgecol','interp',...
        'linew',2);
    % % % % % % % % % % % % % % % % % % % % % % % % % % %
%     hold on
%     plot(1*[-3.5; -2.5],1*[-1; -1],'LineWidth',2,'Parent',ax,'Color',[0 0 0],'LineStyle','-')
%     plot(1*[-3.5; -3.5],1*[-1; 0],'LineWidth',2,'Parent',ax,'Color',[0 0 0],'LineStyle','-')
%     text(-2.700,-1.200, '1 m', 'HorizontalAlignment','right')
%     text(-3.750,-.500, '1 m', 'HorizontalAlignment','center')
    
%     F_S=12;
%     fig2 = figure;  %
%     ax2 = axes('Parent',fig2,'FontSize',F_S)
%     box(ax2,'on');
%     hold(ax2,'all');    
%     ylabel('Velocity(m/s)','FontSize',F_S);    
%     xlabel('Time(s)','FontSize',F_S);    
%     %title('\Delta_{drain}=.1, \Delta_{near}=.5')
%     hold on
%     plot(mm.All_Time,mm.All_States(4,:),'Parent',ax2,'LineWidth',2,'Color',[1 0 0],'LineStyle',':','DisplayName','v_x');
%     plot(mm.All_Time,mm.All_States(5,:),'Parent',ax2,'LineWidth',2,'Color',[0 0 0],'LineStyle','-','DisplayName','v_y');
%     plot(mm.All_Time,mm.All_States(6,:),'Parent',ax2,'LineWidth',2,'Color',[0 0 1],'LineStyle','-.','DisplayName','\psi^.');
end
Ttime=toc