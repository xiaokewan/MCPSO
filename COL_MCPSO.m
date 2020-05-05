clc;
clear;
close all;
tic
    CostFunction =  @(x) Sphere(x); 



    nVar = 30;
    VarSize = [1 nVar]; 
    VarMin = -10;	% Lower Bound of Decision Variables
    VarMax = 10;    % Upper Bound of Decision Variables
    nPops = 20;		% Population size of slave swarms
	nPopm = 20;		% Population size of master swarms
	ssn = 3;		% Number of slave swarms
	MaxIt = 1000;
    ShowIterInfo = 1;
    SelectLinearW = false;	% true: Linear Inertia weight // False: Constriction factor Inertia weight
%% Parameters of PSO
    % inertia weight and c1 c2
    %% w with constrict factor chi
    % Constriction Coefficients
if SelectLinearW
        wmax = 0.9;
        wmin = 0.4;
        w = wmax;
        c1 = 2.05;       % Personal Acceleration Coefficient
        c2 = 2.05;       % Social Acceleration Coefficient
        c3 = 2.00;       % Acceleration constant in COL-MSPSO
else
    kappa = 1;
    phi1 = 2.05;
    phi2 = 2.05;
    phi = phi1 + phi2;
    chi = 2*kappa/abs(2-phi-sqrt(phi^2-4*phi));
 
    w = chi;             % Intertia Coefficient
    
    c1 = chi*phi1;       % Personal Acceleration Coefficient
    c2 = chi*phi2;       % Social Acceleration Coefficient
    c3 = 2.00;
end
        
    wdamp = 1;           % Damping Ratio of Inertia Coefficient
    MaxVelocity = 0.2*(VarMax-VarMin);
    MinVelocity = -MaxVelocity;
	%% Initialization
	 
	% The Particle Template
    empty_particle.Position = [];
    empty_particle.Velocity = [];
    empty_particle.Cost = [];
    empty_particle.Best.Position = [];
    empty_particle.Best.Cost = [];

    % Create Population Array
    slave_particle = repmat(empty_particle, nPops, ssn);				%The range of one slave swarm
	master_particle = repmat(empty_particle, nPopm, 1);                 %The range of master swarm

	
    % Initialize Global Best
    GlobalBest1.Cost = inf;                                              % Initialize GlobalBest of whole slave swarms
    GlobalBest2.Cost = inf;
    emptySlaveGlobalBest.Cost=inf;
    emptySlaveGlobalBest.Position=[];
    SlaveGlobalBest = repmat(emptySlaveGlobalBest,ssn,1);               % Initialize GlobalBest of each slave swarm
    BestCosts = zeros(MaxIt,1);
    
    BestSlaveCosts = zeros(MaxIt,1);
    InteriaWeight = zeros(MaxIt,1);
    
    
%     for j=1:ssn
%         SlaveGlobalBest(j).Cost = inf; 
%         SlaveGlobalBest(j).Position = 0;
%     end
    
    for j=1:ssn
	   
        for i=1:nPops
			% Generate Random Solution
			slave_particle(i,j).Position = unifrnd(VarMin, VarMax, VarSize);

			% Initialize Velocity
			slave_particle(i,j).Velocity = zeros(VarSize);

			% Evaluation
			slave_particle(i,j).Cost = CostFunction(slave_particle(i,j).Position);

			% Update the Personal Best
			slave_particle(i,j).Best.Position = slave_particle(i,j).Position;
			slave_particle(i,j).Best.Cost = slave_particle(i,j).Cost;

			% Update Global Best
            if slave_particle(i,j).Best.Cost < SlaveGlobalBest(j).Cost
				SlaveGlobalBest(j) = slave_particle(i,j).Best;
            end
        end
        
        if SlaveGlobalBest(j).Cost < GlobalBest1.Cost
            GlobalBest1 = SlaveGlobalBest(j);
        end
    end
    
    for i=1:nPopm

        % Generate Random Solution
        master_particle(i).Position = unifrnd(VarMin, VarMax, VarSize);

        % Initialize Velocity
        master_particle(i).Velocity = zeros(VarSize);

        % Evaluation
        master_particle(i).Cost = CostFunction(master_particle(i).Position);

        % Update the Personal Best
        master_particle(i).Best.Position = master_particle(i).Position;
        master_particle(i).Best.Cost = master_particle(i).Cost;

        % Update Global Best
        if master_particle(i).Best.Cost < GlobalBest2.Cost
            GlobalBest2 = master_particle(i).Best;
        end

    end
    
 %% Main Loop of PSO
% slave swarms evaluation
    for it=1:MaxIt
        for j = 1:ssn
            for i=1:nPops

                % Update Velocity
                slave_particle(i,j).Velocity = w*slave_particle(i,j).Velocity ...
                    + c1*rand(VarSize).*(slave_particle(i,j).Best.Position - slave_particle(i,j).Position) ...
                    + c2*rand(VarSize).*(SlaveGlobalBest(j).Position - slave_particle(i,j).Position);

                % Apply Velocity Limits
                slave_particle(i,j).Velocity = max(slave_particle(i,j).Velocity, MinVelocity);
                slave_particle(i,j).Velocity = min(slave_particle(i,j).Velocity, MaxVelocity);

                % Update Position
                slave_particle(i,j).Position = slave_particle(i,j).Position + slave_particle(i,j).Velocity;

                % Apply Lower and Upper Bound Limits
                slave_particle(i,j).Position = max(slave_particle(i,j).Position, VarMin);
                slave_particle(i,j).Position = min(slave_particle(i,j).Position, VarMax);

                % Evaluation
                slave_particle(i,j).Cost = CostFunction(slave_particle(i,j).Position);

                % Update Personal Best
                if slave_particle(i,j).Cost < slave_particle(i,j).Best.Cost

                    slave_particle(i,j).Best.Position = slave_particle(i,j).Position;
                    slave_particle(i,j).Best.Cost = slave_particle(i,j).Cost;

                    % Update Global Best
                    if slave_particle(i,j).Best.Cost < SlaveGlobalBest(j).Cost
                        SlaveGlobalBest(j) = slave_particle(i,j).Best;
                    end            

                end
            end

            if SlaveGlobalBest(j).Cost < GlobalBest1.Cost
                GlobalBest1 = SlaveGlobalBest(j);
            end

        end

%% master swarm evaluation
        for i = 1:nPopm
%(COM-MCPSO)            
%             % Calculate Phi for COM-MSPSO
%             if GlobalBest1.Cost < GlobalBest1.Cost
%                 Phi=0;
%             elseif GlobalBest1.Cost == GlobalBest1.Cost
%                 Phi = 0.5;
%             else 
%                 Phi = 1;
%             end
         
			% Update Velocity
            master_particle(i).Velocity = w*master_particle(i).Velocity ...
                + c1*rand(VarSize).*(master_particle(i).Best.Position - master_particle(i).Position) ...
                + c2*rand(VarSize).*(GlobalBest2.Position - master_particle(i).Position)...
				+ c3*rand(VarSize).*(GlobalBest1.Position - master_particle(i).Position);

            % Apply Velocity Limits
            master_particle(i).Velocity = max(master_particle(i).Velocity, MinVelocity);
            master_particle(i).Velocity = min(master_particle(i).Velocity, MaxVelocity);
            
            % Update Position
            master_particle(i).Position = master_particle(i).Position + master_particle(i).Velocity;
            
            % Apply Lower and Upper Bound Limits
            master_particle(i).Position = max(master_particle(i).Position, VarMin);
            master_particle(i).Position = min(master_particle(i).Position, VarMax);

            % Evaluation
            master_particle(i).Cost = CostFunction(master_particle(i).Position);
                
            if master_particle(i).Cost < master_particle(i).Best.Cost

                    master_particle(i).Best.Position = master_particle(i).Position;
                    master_particle(i).Best.Cost = master_particle(i).Cost;       

                    % Update Global Best
                    if master_particle(i).Best.Cost < GlobalBest2.Cost
                        GlobalBest2 = master_particle(i).Best;
                    end            

            end
        end
 %% transfer results        
        % Decaying Inertia Coefficient
        if SelectLinearW
                 w = wmax -((wmax-wmin)/MaxIt)*it;
        end  
        % Store the Best Cost Value

        BestCosts(it) = GlobalBest2.Cost;
        
        BestSlaveCosts(it) = GlobalBest1.Cost;
        InteriaWeight(it) = w;
        
        % Display Iteration Information
        if ShowIterInfo
            disp(['Iteration ' num2str(it) ': Best Cost = ' num2str(BestCosts(it)) ': Best Slave Cost = ' num2str(BestSlaveCosts(it)) ': Inertia Weight = ' num2str(InteriaWeight(it))]);
        end


    end
    
    out.pop = slave_particle;
    out.BestSol = GlobalBest2;
    out.BestCosts = BestCosts;
toc    
    

%% Results

    subplot(2,1,1);
    % show BestCosts from master sawrm and all slave sawrms
    semilogy(BestCosts,':r','LineWidth',1.5); 
	title('COL-MCPSO');
    text(MaxIt,BestCosts(MaxIt),['(',num2str(MaxIt),',',num2str(BestCosts(MaxIt)),')'],'color','b');
    %text(MaxIt,BestCosts(p),'o','color','g')
    hold on
    semilogy(BestSlaveCosts,'-b','LineWidth',1.5); 
    text(MaxIt,BestSlaveCosts(MaxIt),['(',num2str(MaxIt),',',num2str(BestSlaveCosts(MaxIt)),')'],'color','b');
    legend('BestMasterCosts','BestSlaveCosts');
    xlabel('Iteration');
    ylabel('Best Cost(log)');
    grid on ;
    hold off
	
	% show Inertia Weight
    subplot(2,1,2);
    plot(InteriaWeight,'-k','LineWidth',1.5);
    title('Inertia-Weight');
    xlabel('Iteration');
    ylabel('Inertia Weight'); 
%     figure(2)
%     plot(master_particlevelocity,'LineWidth',2);
%     xlabel('Iteration');
%     ylabel('Master Global best Velocity');
% %end %end function        