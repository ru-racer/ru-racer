classdef Bike%
    % Bike % verified
    properties
    Input_Size
    State_Size
    Initial_Input
    Initial_State
    Last_State
    Last_Input
    Last_Cost
    All_States
    All_Time
    All_Inputs
    IsFeasible
    bounds
    All_Dynamics
    Obstacles
    ObstaclesSize
    goal
    maptype
    end
    
     properties (Constant)
         del_T=.02;
         rewiring_value = .05;
         m=6.8; Izz=.15; lf=.15; lr=.25; g=9.8; h=.02;
         tirer=struct('B',10.275,'C',1.56,'D',-.79); tiref=struct('B',10.275,'C',1.56,'D',-.85)
     end
    
    methods
        
        function this = Bike(Initial_State,Initial_Input,map)
           this.Initial_State=Initial_State;
           this.goal=Initial_State;
           this.Initial_Input=Initial_Input;
           this.Last_State=Initial_State;
           this.Input_Size = length(Initial_Input);
           this.State_Size = length(Initial_State);
           this.All_States=[];           
           this.All_Inputs=[];
           this.All_States(:,1)=Initial_State;
           this.All_Time=0;
           this.All_Dynamics(:,1)=0*Initial_State;
           this.All_Inputs(:,1)=Initial_Input;
           this.bounds.x = [-25*ones(this.State_Size,1) 25*ones(this.State_Size,1)];
           this.bounds.dx= [-20*ones(this.State_Size,1) 20*ones(this.State_Size,1)];
           this.IsFeasible = 1;
           if ischar(map)
                this.Obstacles = imread(map);
                this.ObstaclesSize = size(this.Obstacles);
                this.maptype=1;
           else
                this.Obstacles = map;
                this.maptype=0;
           end
        end
        
        function dX = Dynamic(this,U)
            X = this.Last_State; %[x y psi vx vy dotpsi]
%           x=X(1);  y=X(2);
            psi=X(3);
            vx=X(4); vy=X(5); dotpsi=X(6);
            delta=U(1);
            sfx=U(2);srx=sfx;
            cd=cos(delta);sd=sin(delta);            
            sfy= ((vy+this.lf*dotpsi)*cd-vx*sd)/(vx*cd+(vy+this.lf*dotpsi)*sd);
            sry=(vy-this.lr*dotpsi)/vx;
            sf=sqrt(sfy^2+sfx^2); sr=sqrt(sry^2+srx^2);
            mur=this.tirer.D*sin(this.tirer.C*atan(this.tirer.B*sr));
            muf=this.tiref.D*sin(this.tiref.C*atan(this.tiref.B*sf));
            if abs(sf)>.01
            mufx=-muf*(sfx/sf); mufy=-muf*(sfy/sf);
            else
            mufx=0; mufy=0;
            end
            if abs(sr)>.001
            murx=-mur*(srx/sr); mury=-mur*(sry/sr);
            else
            murx=0; mury=0;
            end
            Ffz=(this.m*this.g*(this.lr-murx*this.h))/(this.lf+this.lr+this.h*(mufx*cd-mufy*sd-murx));
            Frz=this.m*this.g-Ffz;
            Ffx=mufx*Ffz;
            Frx=murx*Frz;
            Ffy=-mufy*Ffz;
            Fry=-mury*Frz;
            dotx=vx*cos(psi)-vy*sin(psi);
            doty=vx*sin(psi)+vy*cos(psi);
            dotvx=(Ffx*cd-Ffy*sd+Frx)/this.m+1*vy*dotpsi;
            dotvy=(Ffx*sd+Ffy*cd+Fry)/this.m-1*vx*dotpsi;
            dotdotpsi=(this.lf*(Ffx*sd+Ffy*cd)-this.lr*Fry)/this.Izz;
            dX=[dotx doty dotpsi dotvx dotvy dotdotpsi]'; %[dotx doty dotpsi dotvx dotvy dotdotpsi]
        end
        
        function this = run_Dyn(this,U,Horizon)
            this.IsFeasible  = 1;
            this.Last_Input=U;            
            for t=0:this.del_T:Horizon
            dX=Dynamic(this,U);
            this.Last_State=this.Last_State+dX*this.del_T;
            this.All_States=[this.All_States this.Last_State];
            this.All_Inputs=[this.All_Inputs U];
            this.All_Time=[this.All_Time this.All_Time(end)+this.del_T];
            this.All_Dynamics=[this.All_Dynamics dX];
            if (this.maptype==1)
                this.IsFeasible = check_feasibility_map(this,dX);
            else
                 this.IsFeasible = check_feasibility(this,dX);
            end
            if norm(this.Last_State(1:2)-this.goal(1:2))<2*this.rewiring_value
                this.Last_Cost = t;
                return;
            end
            if this.IsFeasible  == 0
                this.Last_Cost = 1000;
                return;
            end
            
            end
            this.Last_Cost = Horizon;
        end

        function this = run_Dyn1(this,U,Horizon)
            this.IsFeasible  = 1;
            this.Last_Input=U;            
            for t=0:this.del_T:Horizon            
            dX=Dynamic(this,U);
            this.Last_State=this.Last_State+dX*this.del_T;
            this.All_States=[this.All_States this.Last_State];
            this.All_Time=[this.All_Time this.All_Time(end)+this.del_T];
            this.All_Inputs=[this.All_Inputs U];
            this.All_Dynamics=[this.All_Dynamics dX];
            end
            this.Last_Cost = Horizon;
        end
        
        function this = run_Dyn_inv(this,U,Horizon)
            this.IsFeasible  = 1;
            this.Last_Input=U;
            for t=0:this.del_T:Horizon
                dX=Dynamic(this,U);
                this.Last_State=this.Last_State-dX*this.del_T;
                this.All_States=[this.All_States this.Last_State];
                this.All_Time=[this.All_Time this.All_Time(end)+this.del_T];
                this.All_Inputs=[this.All_Inputs U];
                this.All_Dynamics=[this.All_Dynamics dX];
%             if (this.maptype==1)
%                 this.IsFeasible = check_feasibility_map(this,dX);
%             else
%                 this.IsFeasible = check_feasibility(this,dX);
%             end
%             if this.IsFeasible  == 0
% %                 this.Last_State
%                 this.Last_Cost = 1000;
%                 break;
%             end
            end
            this.Last_Cost = Horizon;
        end
                
        function this = run_Dyn_CL(this,Xd,Horizon)
            this.IsFeasible  = 1;            
            for t=0:this.del_T:Horizon
            this.Last_Input=this.Controller(Xd);%U;
            dX=Dynamic(this,this.Last_Input);
            this.Last_State=this.Last_State+dX*this.del_T;
            this.All_States=[this.All_States this.Last_State];
            this.All_Time=[this.All_Time this.All_Time(end)+this.del_T];
            this.All_Inputs=[this.All_Inputs this.Last_Input];
            this.All_Dynamics=[this.All_Dynamics dX];
            this.Last_Cost = this.Last_Cost + this.del_T;
            if norm(Xd(1:3)-this.Last_State(1:3),2) < this.rewiring_value
                return
            end
            if this.maptype==1
                this.IsFeasible = check_feasibility_map(this,dX);
            else
                 this.IsFeasible = check_feasibility(this,dX);
            end
            if this.IsFeasible  == 0
                this.Last_Cost = 1000;
                break
            end
            end
            this.Last_Cost = Horizon;
        end
        
        function feasibility = check_feasibility(this,dX)
            for i=length(this.Obstacles(:,1)):-1:1
                     if ((this.All_States(1,end)-this.Obstacles(i,1))^2)+((this.All_States(2,end)-this.Obstacles(i,2))^2)< ...
                         this.Obstacles(i,3)*1
                        feasibility = 0;
                        return
                     end
            end
                 c1=(this.All_States(:,end) > this.bounds.x(:,1)) & (this.All_States(:,end) < this.bounds.x(:,2));
%                c2=(dX > this.bounds.dx(:,1)) & (dX < this.bounds.dx(:,2));
                 feasibility = min(c1);
        end
        
        function feasibility = check_feasibility_map(this,dX)
            c1=(this.All_States(:,end) > this.bounds.x(:,1)) & (this.All_States(:,end) < this.bounds.x(:,2));
            feasibility = min(c1);
            if (~feasibility)
                return
            end
            X=max(1,ceil(this.All_States(1,end)*100+2.819*100));
            Y=max(1,ceil(3.581*100-this.All_States(2,end)*100));
            X=min(this.ObstaclesSize(2),X);
            Y=min(this.ObstaclesSize(1),Y);
            if (this.Obstacles(Y,X)==0)
                feasibility = 0;
                return
            end
        end
        
        function this = reset_states(this,X,U)
           this.Initial_State=X;
           this.Initial_Input=U;
           this.Last_State=X;
           this.All_States=[];
           this.All_Time=0;
           this.All_Inputs=[];
           this.All_States(:,1)=X;
           this.All_Dynamics=[];
           this.All_Dynamics(:,1)=X*0;
           this.All_Inputs(:,1)=U;           
           this.IsFeasible = 1;
        end
        
        function U = Controller(this,Xd)
            X=this.Last_State;
            ke=.3; kp1=2; kp2=.01;
            derr=atan2(Xd(2)-X(2),Xd(1)-X(1));
            kp=kp1*exp(-kp2*norm(X(4)));
            delta=this.satu((kp*((-X(3)+derr)))+ke*(sin(Xd(3)-X(3))),pi/9);
            U(1,1)=delta;
            U(2,1)=this.satu(.1*exp(norm(derr))*(.1*(Xd(4)-X(4)) + sign(derr+55)*.07*norm(Xd(1:2)-X(1:2))),.1);
        end
                       
    end %methos
	
    methods(Static)
        function y=satu(x,sat)
            y=sign(x)*min(sat,abs(x));
        end
    end
    
end %calss

