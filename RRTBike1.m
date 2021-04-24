% 500000 iterations 4351 nodes in 2256.2026 Distance 0.36539 Cost 2.5801 Path 39  Rewired 2090

classdef RRTBike < handle
    
    properties
        tree                % Array stores position information of states
        tree_input
        parent              % Array stores relations of nodes
        children            % Number of children of each node
        time                % Cost between 2 connected states
        cumcost             % Cost from the root of the tree to the given node
        nodes_added         % Keeps count of added nodes
        obstacle            % Obstacle information
        best_node      % The index of last node of the best path
        goal_reached
        model               % Model and trajectory data for each node
        max_nodes
        Model_Size
        Input_Size
        XY_BOUNDARY
        Input_Boundary
        goal
        model_history
        distance
        num_rewire
        best_path
        score
        success
        SparsMarked
        ReConnect
        removed
        delta_near
        type
    end
    
    properties (Constant)
    delta_goal=.5;
%   delta_near =1;
    delta_drain = .3;
    max_step = .2;
    end
    
    methods
        
    function this = RRTBike(max_nodes,map,type) %Initil state and others
    this.type=type;
	this.delta_near =.3;
    this.Model_Size=6;
    this.Input_Size=2;
    this.model = Bike([0;.01;pi/2;1;0;0],[0;0],map);
    this.goal = [-2;0;3*pi/2;.5;0;0];%
%   this.goal = [0;2.5;pi/2;1;0;0];%
    this.model.goal = this.goal;%
	this.max_nodes=max_nodes;
	this.tree = zeros(this.Model_Size, max_nodes);
    this.tree_input = zeros(this.Input_Size, max_nodes);
	this.parent = zeros(1, max_nodes);
    this.score = zeros(1, max_nodes);
    this.success = zeros(1, max_nodes);
	this.children = zeros(1, max_nodes);
	this.time = zeros(1, max_nodes);
	this.cumcost = zeros(1,max_nodes);
	this.best_node = 1;
	this.goal_reached = [];
	this.tree(:, 1) = this.model.Last_State;
    this.nodes_added=1;
%   this.semigoal = [-2;-1;-pi/2;1;0;0];%    
%   this.XY_BOUNDARY = [[-2.6;-2;-2*pi;.5;-8;-25] [.3;3.5;2*pi;5;3;25]];
    this.XY_BOUNDARY = [[-2.6;-.02;-2*pi;.5;-.5;-5] [.3;3.5;2*pi;3.5;.5;5]];
    this.model.bounds.x=this.XY_BOUNDARY;
    this.Input_Boundary = [[-pi/9;pi/9] [-.1;.1]];
    this.obstacle=map;
    this.model_history = {};
    this.distance = zeros(1,max_nodes);
    this.ReConnect = zeros(1,max_nodes);
    this.distance(1) = 100;%norm(this.goal - this.tree(:, 1));
    this.num_rewire = 0;
    this.removed = 0;
%     for i=1:max_nodes
%     this.model_history{i}={};
%     end
% 	this.load_map(map.name);
    end
    
    function Node = sample(this,goal)
        w = 0;%.2+.8*(this.nodes_added/this.max_nodes);
        Node=(1-w)*(this.XY_BOUNDARY(:,1) + rand(this.Model_Size,1).*(this.XY_BOUNDARY(:,2)-this.XY_BOUNDARY(:,1))) ...
            + w * goal;
    end
    
    function Nearest_ind=Nearest(this,Node)
        dist_vec=zeros(1, this.nodes_added);
        for i=1:4 %length(Node)  % add weight for each state
            if i==3
            dist_vec = dist_vec + 1*abs(atan2(Node(2)-this.tree(2,1:this.nodes_added),Node(1)-this.tree(1,1:this.nodes_added)));
            else
            dist_vec = dist_vec + (this.tree(i,1:this.nodes_added)-Node(i)).^2; %Common Euq distance % maybe some thing else
            end
        end
        dist_vec=sqrt(dist_vec);
        if length(this.goal_reached)>1
        dist_vec(this.goal_reached)=dist_vec(this.goal_reached)+100;
        end
        [~,Nearest_ind]=min(dist_vec);
    end
    
    function [IsFeasible,New_node]=Steer(this,Nearest_ind,Random_Node)
        this.model=this.model.reset_states(this.tree(:,Nearest_ind),[0;0]);
        a=rand;
        if a<.5 || this.type=='o'
%         if (this.distance(this.best_node)>2 || this.distance(this.best_node)<.4) && a<.9
          raninput= this.Input_Boundary(1,:)+( this.Input_Boundary(2,:)- this.Input_Boundary(1,:) ).*(rand(1,2));
          this.model=this.model.run_Dyn(raninput',this.max_step);
        elseif a>.7
          this.model=this.model.run_Dyn(this.model.Controller(Random_Node),this.max_step);
        elseif this.distance(this.best_node)>2.4
          Random_Node=[-.1;2.7;pi*.1;5;0;0];
          this.model=this.model.run_Dyn((.5+.5*(rand))*this.model.Controller(Random_Node),this.max_step);
        else
            this.model=this.model.run_Dyn((.5+.5*(rand))*this.model.Controller(this.goal),this.max_step);
        end
        %;Random_Node(4)-this.tree(4,Nearest_ind)],this.Input_Boundary) ...
        %,this.max_step);
        IsFeasible = this.model.IsFeasible;
        if IsFeasible        
            this.success(Nearest_ind) = this.success(Nearest_ind)+1;
        else
            this.success(Nearest_ind) = this.success(Nearest_ind)-1;
        end
        New_node = this.model.Last_State;        
    end
    
    function Neighbors_Nodes=Neighbors(this,New_Node,Nearest_Ind)
        dist_vec=zeros(1, this.nodes_added);
        for i=1:4 %length(New_Node)  %% this part need to be changed
            if i==3
                dist_vec = dist_vec + 1*abs(atan2(New_Node(2)-this.tree(2,1:this.nodes_added),New_Node(1)-this.tree(1,1:this.nodes_added)));
            else
                dist_vec = dist_vec + (this.tree(i,1:this.nodes_added)-New_Node(i)).^2; %Common Euq distance % maybe some thing else    
            end
            
        end        
        Neighbors_Nodes=find(dist_vec<this.delta_near*rand);
        Neighbors_Nodes = Neighbors_Nodes(Neighbors_Nodes~=Nearest_Ind);
    end
    
    function DrainNodes=Drain(this,New_Ind,Nearest_Ind)        
        dist_vec = sqrt((this.tree(1,1:this.nodes_added)-this.tree(1,New_Ind)).^2 + (this.tree(2,1:this.nodes_added)-this.tree(2,New_Ind)).^2 ...
            + (this.tree(3,1:this.nodes_added)-this.tree(3,New_Ind)).^2 + (this.tree(4,1:this.nodes_added)-this.tree(4,New_Ind)).^2  + ...
            (this.tree(5,1:this.nodes_added)-this.tree(5,New_Ind)).^2 + (this.tree(6,1:this.nodes_added)-this.tree(6,New_Ind)).^2);
        DrainNodes = find(dist_vec < this.delta_drain);
        DrainNodes = DrainNodes(DrainNodes~=Nearest_Ind);
%         DrainNodes = DrainNodes(DrainNodes~=New_Ind);        
        L=length(DrainNodes);
        for i=1:L            
            if this.children(DrainNodes(i))==0 && this.cumcost(DrainNodes(i)) > this.cumcost(New_Ind)
                this.removed = this.removed+1;
                this.remove_Node(DrainNodes(i));
            elseif this.cumcost(DrainNodes(i)) < this.cumcost(New_Ind)
                this.removed = this.removed+1;
                this.remove_Node(New_Ind);
            end
        end
%         RemovingHighCostNodes
        if ~isempty(this.goal_reached) && this.type=='p'
            HCNodes = find(this.cumcost > min(this.cumcost(this.goal_reached)));
            for i=length(HCNodes):-1:1
                if this.children(HCNodes(i))==0
                    this.remove_Node(HCNodes(i));
                end
            end
        end
    end
    
    function Parent_ind=find_NewParent(this,Neighbors,New_Node,Nearest_Ind)
        Parent_ind=Nearest_Ind;
        min_cost=this.cumcost(Nearest_Ind)+this.eval_cost_est(Nearest_Ind,New_Node);
        for i=1:length(Neighbors)
            TC=this.cumcost(Neighbors(i))+this.eval_cost_est(Neighbors(i),New_Node);
            if TC < min_cost && this.success(Neighbors(i)) > -8 && this.success(Neighbors(i)) <20
                min_cost=TC;
                Parent_ind = Neighbors(i);
            end
        end
    end
 
    function cost=eval_cost(this,Nearest_Ind,New_Node)
%         cost=norm(this.tree(1:2,Nearest_Ind)-New_Node(1:2));%+this.time(Nearest_Ind);
	cost = (norm(this.tree(1:2,Nearest_Ind)-New_Node(1:2),2))/mean([this.tree(4,Nearest_Ind) New_Node(4)]);

        
    end
    
    function cost=eval_cost_est(this,Nearest_Ind,New_Node)
%         cost=norm(this.tree(1:2,Nearest_Ind)-New_Node(1:2));%+this.time(Nearest_Ind);
%         cost = (norm(this.tree(1:2,Nearest_Ind)-New_Node(1:2),2))/norm(this.tree(4,Nearest_Ind));
          if abs(atan2(New_Node(2)-this.tree(2,Nearest_Ind),New_Node(1)-this.tree(1,Nearest_Ind) ) )<.6
              cost = (norm(this.tree(1:2,Nearest_Ind)-New_Node(1:2),2))/mean([this.tree(4,Nearest_Ind) New_Node(4)]);             
          else
              cost = 5*norm([abs(atan2(New_Node(2)-this.tree(2,Nearest_Ind),New_Node(1)-this.tree(1,Nearest_Ind) ) )]) /norm(this.tree(4,Nearest_Ind));
          end
%    cost = (norm(this.tree(1:2,Nearest_Ind)-New_Node(1:2),2))+ 1/mean([this.tree(4,Nearest_Ind) New_Node(4)]);
    end
    
    function New_Ind = add_Node(this,Parent_ind,New_Node)
       newcost = this.cumcost(Parent_ind) + this.eval_cost(Parent_ind,New_Node);       
       if newcost > min(this.cumcost(this.goal_reached))
           New_Ind=0;
           return;
       end
       this.nodes_added = this.nodes_added + 1;
       this.tree(:,this.nodes_added) = New_Node;
       this.tree_input(:,this.nodes_added) = this.model.Last_Input;
       this.parent(this.nodes_added) = Parent_ind;
       if Parent_ind<this.max_nodes
           this.children(Parent_ind) = this.children(Parent_ind) + 1;
       end
%      this.children(Parent_ind) = this.children(Parent_ind) + 1;
%      this.model_history{this.nodes_added}= this.model;
       this.cumcost(this.nodes_added) = newcost;%this.cumcost(Parent_ind) + this.eval_cost(Parent_ind,New_Node);   % cummulative cost
       this.time(this.nodes_added) = this.time(Parent_ind) + this.model.Last_Cost;
       this.distance(this.nodes_added) = norm([this.goal(1:2)-New_Node(1:2);.25*(this.goal(4)-New_Node(4));...
           .2*this.solvePI(this.goal(3)-New_Node(3)); ...
           1*abs(atan2(this.goal(2)-New_Node(2),this.goal(1)-New_Node(1)))  ],2);
       if this.distance(this.nodes_added) < this.delta_goal
           this.goal_reached = [this.goal_reached this.nodes_added];
       end
       if this.distance(this.nodes_added) < this.distance(this.best_node)
           this.best_node = this.nodes_added;
       end                  
       New_Ind = this.nodes_added;
    end
    
    function remove_Node(this,remove_ind)
        if this.nodes_added-remove_ind < 3  || this.nodes_added<2 ||  (remove_ind==this.best_node) || remove_ind==1 ...
                || ~isempty(find(this.goal_reached==remove_ind, 1)) %|| this.nodes_added == this.max
            return
        else            
            kids = find(this.parent==remove_ind);                        
            if ~isempty(kids)
                for i=1:numel(kids)
                    if this.children(kids(i))==0
                        this.remove_Node(kids(i));
                    end
                end
            end
            this.tree(:,1:this.nodes_added) = [this.tree(:,1:remove_ind-1) this.tree(:,remove_ind+1:this.nodes_added) 0*this.tree(:,2)];
            this.tree_input(:,1:this.nodes_added) = [this.tree_input(:,1:remove_ind-1) this.tree_input(:,remove_ind+1:this.nodes_added) 0*this.tree_input(:,2)];
            this.children(this.parent(remove_ind))=this.children(this.parent(remove_ind))-1;
            Indparentstochange = find(this.parent(1:this.nodes_added)>remove_ind);
            this.parent(Indparentstochange)=this.parent(Indparentstochange)-1;
            this.parent(1:this.nodes_added) = [this.parent(1:remove_ind-1) this.parent(remove_ind+1:this.nodes_added) 0];
            this.children(1:this.nodes_added) = [this.children(1:remove_ind-1) this.children(remove_ind+1:this.nodes_added) 0];
            this.cumcost(1:this.nodes_added) = [this.cumcost(1:remove_ind-1) this.cumcost(remove_ind+1:this.nodes_added) 0];
            this.distance(1:this.nodes_added) = [this.distance(1:remove_ind-1) this.distance((remove_ind+1):this.nodes_added) 0];
            this.success(1:this.nodes_added) = [this.success(1:remove_ind-1) this.success((remove_ind+1):this.nodes_added) 0];
%             this.distance(1:this.nodes_added) = [this.distance(1:remove_ind-1) this.distance((remove_ind+1):this.nodes_added) 0];            
            this.time(1:this.nodes_added) = [this.time(1:remove_ind-1) this.time((remove_ind+1):this.nodes_added) 0];
            this.nodes_added = this.nodes_added-1;
            indGR = find(this.goal_reached>remove_ind);
            this.goal_reached(indGR)= this.goal_reached(indGR)-1;
            if this.best_node>remove_ind
                this.best_node = this.best_node-1;
            end
        end
        
    end
       
	function rewire(this, new_ind, Neighbors, min_ind)
        for i=length(Neighbors):-1:1            
            if min_ind==Neighbors(i)
                continue;
            end
              tCost = this.cumcost(new_ind) + this.eval_cost_est(new_ind,this.tree(:,Neighbors(i)));
                if (tCost<this.cumcost(Neighbors(i))) % && check the feasibility, Dynamic and
                      this.model.IsFeasible=1;
                      this.model=this.model.run_Dyn_CL(this.tree(:,Neighbors(i)),RRT1.max_step);
                      tCost = this.cumcost(new_ind) + this.eval_cost(new_ind,this.model.Last_State);
                    if (this.model.IsFeasible && tCost<this.cumcost(Neighbors(i)))
                        difCost = this.cumcost(Neighbors(i)) - tCost;
                        this.cumcost(Neighbors(i)) = tCost;
                        kids = find(this.parent==Neighbors(i));
                        allkids = kids;
% % % % % % % % %
                        while true && ~isempty(kids)
                            nkids = [];
                        for j=1:numel(kids)
                            nkids = [nkids find(this.parent==kids(j))];
                        end
                        if isempty(nkids)
                                break
                        else
                            allkids = [allkids nkids] ;
                            kids = nkids;
                        end
                        end                 
% % % % % % % % %                         
                        this.cumcost(allkids)= this.cumcost(allkids)-difCost;
                        this.children(this.parent(Neighbors(i))) = this.children(this.parent(Neighbors(i))) - 1;
                        this.parent(Neighbors(i)) = new_ind;
%                         this.tree_input(:,Neighbors(i))=[-10;-10];
                        if new_ind<this.max_nodes
                            this.children(new_ind) = this.children(new_ind) + 1;
                        end
                        this.num_rewire= this.num_rewire+ 1;
                    end
                end
                %%model
        end
    end
    
	function model=FollowTrajectory(this,Inputs,Path,time)
        this.model=this.model.reset_states(Path(:,1),Inputs(:,1));
            for i=1:(numel(time)-1)
%                 if Inputs(1,i+1)==-10 && Inputs(2,i+1)==-10
%                 this.model=this.model.run_Dyn_CL(Path(:,i+1),time(i+1)-time(i));
%                 else
                this.model=this.model.run_Dyn(Inputs(:,i+1),time(i+1)-time(i));
%                 end
            end
%           this.model.All_States=this.model.All_States(:,1:end-1);
          this.model=this.model.run_Dyn_CL(Path(:,end),2*(time(end)-time(end-1)));
        model=this.model;
	end
    
function Implot2d(this)        
%     this.TrackSize();
	drawn_nodes = zeros(1, this.nodes_added);
	hold on;
% 	for i=length(this.obstacle(:,1)):-1:1
%         this.plot_circle_s(this.obstacle(i,1),this.obstacle(i,2),this.obstacle(i,3),'k');
%     end
    for i = this.nodes_added:-1:1;
	current_index = i;
         while(current_index ~= 1 && current_index ~= -1 && current_index~=0)            
         if(drawn_nodes(current_index) == false || drawn_nodes(this.parent(current_index)) == false)
%              plot(this.model_history{i}.All_States(1,:),this.model_history{i}.All_States(2,:),'r')
             plot(687+360*[this.tree(2,current_index);this.tree(2, this.parent(current_index))], ...
                 1125+390*[this.tree(1, current_index);this.tree(1, this.parent(current_index))],'-','LineWidth', 0.2,'Color',[0 .4 0]);
             plot(687+360*[this.tree(2,current_index);this.tree(2, this.parent(current_index))], ...
                 1125+390*[this.tree(1, current_index);this.tree(1, this.parent(current_index))],'.','Color',1*[0 .4 .4]);
             drawn_nodes(current_index) = true;
         end
         current_index = this.parent(current_index); 
         end
    end
%      this.plot_circle(687+360*this.goal(2),1125+390*this.goal(1),this.delta_goal*100);
%     this.plot_circle(this.tree(1,1),this.tree(2,1),.2);
    
%     for i =1:length(this.goal_reached)
     if ~isempty(this.goal_reached)
        [~,i]=min(this.cumcost(this.goal_reached));
        Cind = this.goal_reached(i);        
        while 1==1
        Pind = this.parent(Cind);
        if this.cumcost(this.goal_reached(i))==min(this.cumcost(this.goal_reached))
            if isempty(this.best_path)
                this.best_path = [Cind;Pind];
            else
                if isempty(this.best_path(this.best_path==Pind))
                this.best_path = [this.best_path;Pind];
                end
            end
            plot(687+360*[this.tree(2,Cind) this.tree(2,Pind)],1125+390*[this.tree(1,Cind) this.tree(1,Pind)],'b','LineWidth',1)
%             this.plot_circle_s(this.tree(1,Cind),this.tree(2,Cind),this.tree(4,Cind)/30,'b')
%              Plotcar(this.tree(:,Cind),this.tree_input(1,Cind),82,46,'-',[0 0 0]);
        else
            plot(687+360*[this.tree(2,Cind) this.tree(2,Pind)],1125+390*[this.tree(1,Cind) this.tree(1,Pind)],'r','LineWidth',1)
        end
        if Pind == 1
            break
        end
        Cind = Pind;
        end
    end   
    this.best_path = flip(this.best_path);
    axis equal
    end
    
    
    end %method

    methods(Static)

        function TrackSize()
        length=2.1336;%m;
        width=1.524;%m;
        radiu_in=0.4572;%m
        radiu_out=1.9812;%m                 
        x1 = [0.381 0.381];
        x2=[-1.2954 -1.2954];% -2.8194 -2.8194];
        y1 = [-1.5616428.*10^0 .75*length+1.5616428.*10^0];%  length 0];
        y2=[0 length]; 
        x3=[-2.8194 -2.8194];
        y3=[0 .95*length];
        c1 = linspace(0,-pi,20);
        c2 = linspace(0,pi,20);
        c3 = linspace(asin(4/6.5)-pi/2,-pi,20);
        c4 = linspace(pi/2-asin(4/6.5),.95*pi,20);
        plot(x1,y1,-x1,y2,...
        x2,y2,...
        x3,y3,...       
        radiu_out*cos(c4)-0.8382*10^(0),radiu_out*sin(c4)+.75*length...
        ,radiu_out*cos(c3)-0.8382*10^(0),radiu_out*sin(c3),...
        radiu_in*cos(c2)-0.8382*10^(0),radiu_in*sin(c2)+length...
        ,radiu_in*cos(c1)-0.8382*10^(0),radiu_in*sin(c1),...
        'LineWidth',3,'Color',[.7 .7 .7])
        axis equal
        end
        
        function plot_circle(x, y, r)
            t = 0:0.005:2*pi;
            cir_x = r*cos(t) + x;
            cir_y = r*sin(t) + y;
            plot(cir_x, cir_y, 'b-', 'LineWidth', 1.5);
        end
        
        function y=sat(x,bound)
           y=x;
            for i=1:length(x)
                if x(i)>bound(i,2)
                    y(i)=bound(i,2);
                elseif x(i)<bound(i,1)
                    y(i)=bound(i,1);
                end
            end
        end
        
        function y=solvePI(x)
            if x>2*pi
                y=x-2*pi;
            elseif x<-2*pi
                y=x+2*pi;
            else
                y=x;
            end
        end
        
        function plot_circle_s(x, y, r,c)
            THETA=linspace(0,2*pi,1000);
            RHO=ones(1,1000)*r;
            [X,Y] = pol2cart(THETA,RHO);
            X=X+x;
            Y=Y+y;
            h=fill(X,Y,c);
            axis square;
%             t = 0:0.005:2*pi;
%             cir_x = r*cos(t) + x;
%             cir_y = r*sin(t) + y;
%             plot(cir_x, cir_y, c, 'LineWidth',-1);
        end
        
    end
    
end