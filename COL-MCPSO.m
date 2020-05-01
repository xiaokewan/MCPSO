clc;
clear;
close all;

    CostFunction = @(x) (x^2);



    nVar = 1;
    VarSize = [1 nVar]; 
    VarMin = -100;	% Lower Bound of Decision Variables
    VarMax = 100;    % Upper Bound of Decision Variables
    nPops = 20;	% Population size of slave swarms
	nPopm = 20;	% Population size of master swarms
	ssn = 4;		% Number of slave swarms
	MaxIt = 1000;
    ShowIterInfo = 1;
   
    %% inertia weight and c1 c2
    kappa = 1;
%     phi1 = 2.05;        
%     phi2 = 2.05;
%     phi = phi1 + phi2;
%     chi = 2*kappa/abs(2-phi-sqrt(phi^2-4*phi));     % constriction coefficients
    wmax = 0.9;
    wmin = 0.4;
    w = wmax;
    c1 = 2.05;       % Personal Acceleration Coefficient
    c2 = 2.05;       % Social Acceleration Coefficient
    c3 = 2.00;       % Acceleration constant in COL-MSPSO
    MaxVelocity = 0.2*(VarMax-VarMin);
    MinVelocity = -MaxVelocity;
    wdamp = 1;
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
        w = wmax -((wmax-wmin)/MaxIt)*it;
          
        % Store the Best Cost Value

        BestCosts(it) = GlobalBest2.Cost;

        % Display Iteration Information
        if ShowIterInfo
            disp(['Iteration ' num2str(it) ': Best Cost = ' num2str(BestCosts(it))]);
        end


    end
    
    out.pop = slave_particle;
    out.BestSol = GlobalBest2;
    out.BestCosts = BestCosts;
    

%% Results

    figure;
    %plot((BestCosts,'LineWidth',2); 
    semilogy(BestCosts,'LineWidth',2);      
    xlabel('Iteration');
    ylabel('Best Cost');
%end %end function        