%---------------------------------------------------------------------------------------------------
% For Paper
% "Nonlinear Distributed Model Predictive Flocking with Obstacle Avoidance"
% by P. Hastedt and H. Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Philipp Hastedt
%---------------------------------------------------------------------------------------------------

classdef PredictiveSqpAgent < DynamicUnicycle
    
    % Define constants of the flocking protocol
    properties
        d;              % desired distance to neighbours
        do;             % desired obstacle separation
        r;              % agent interaction range
        ro;             % obstacle interaction range
        lambda_u;       % control weight
        lambda_attr;    % attractive inter-agent weight
        lambda_rep;     % repulsive inter-agent weight
        lambda_obs;     % obstacle separation weight
        lambda_vel;     % velocity alignment weight
        Hp;             % prediction horizon
        n;              % state dimension
        m;              % input dimension
        u_max;          % maximum input
        epsilon;        % epsilon map parameter
        d_l;            % gamma look ahead distance
        q_pos;          % reference position weight
        q_psi;          % reference attitude weight
        q_v;            % reference linear velocity weight
        q_dpsi;         % reference angular velocity weight
        maxIterations;  % maximum number of iterations
    end
    
    % data to be transmitted in addition to position, velocity, and u
    properties
        num_N;          % number of neighbors
        num_O;          % number of obstacles
        neighbors;      % agent ids of neighbors
        U_opt;          % optimal input
        numberAgents;   % total number of agents
        reference;      % reference information
        obstacles;      % obstacle information
        tOptim = 0;     % summed time for optimization
    end
    
    % member variables
    properties(GetAccess = private, SetAccess = private)
        tau;    % sampling time
        handle = 0; % handle length
    end
    
    methods
        X = statePrediction(obj, x0, U);
        U = shiftUopt(obj, U_opt, u_end);
    end
    
    methods(Static)
        function e_nm = e_nm(n,m)
            e_nm = zeros(1,n);
            e_nm(m) = 1;
        end
        
        function y = epsilonNorm(z, epsilon)
            y = (sqrt(1 + epsilon^2 * norm(z)^2) - 1) / epsilon;
        end
    end
    
    methods
        function obj = PredictiveSqpAgent(id, param, initPos, initVel, initAtt, initAngVel, cfg)
            % call parent constructor
            obj@DynamicUnicycle(id, param.dT, cfg.mass, cfg.inertia, initPos, initAtt, initVel, initAngVel);
            
            % load data from config file
            obj.d = cfg.d;
            obj.do = cfg.do;
            obj.r = param.range;
            obj.ro = param.ro;
            
            obj.lambda_u = cfg.lambda_u;
            obj.lambda_attr = cfg.lambda_attr;
            obj.lambda_rep = cfg.lambda_rep;
            obj.lambda_obs = cfg.lambda_obs;
            obj.lambda_vel = cfg.lambda_vel;
            obj.d_l = cfg.d_l;
            obj.q_pos = cfg.q_pos;
            obj.q_psi = cfg.q_psi;
            obj.q_v = cfg.q_v;
            obj.q_dpsi = cfg.q_dpsi;
            
            obj.Hp = cfg.Hp;
            obj.n = 5;
            obj.m = 2;
            obj.u_max = cfg.u_max;
            
            obj.epsilon = cfg.epsilon;
            obj.maxIterations = cfg.maxIterations;
            
            % initialize member variables
            obj.tau = param.dT;
            obj.num_N = 0;
            obj.numberAgents = param.agentCount;
            obj.U_opt = zeros(obj.Hp * obj.m,1);
            obj.obstacles = param.obstacles;
            obj.tOptim = 0;
            
            % set reference
            if numel(param.reference) == 0 % no reference
                obj.reference = [];
            elseif size(param.reference,2) == 1 % one reference for whole swarm
                obj.reference = param.reference;
            else % one reference for each agent
                obj.reference = param.reference(:,id);
            end
        end
        
        function step(obj)
            obj.neighbors = zeros(obj.numberAgents,1);
            % Receive messages from the network
            messages = obj.receive();
            
            % Implement the flocking protocol
            obj.num_N = length(messages);
            x_i = obj.state;
            Xj_hat = [];
            if obj.num_N == 0
                obj.U_opt = obj.shiftUopt(obj.U_opt,0);
                u = 0*obj.U_opt(1:obj.m);
            else
                for message = messages
                    x_j = [message.data.position; message.data.attitude; message.data.velocity; message.data.angularVelocity];
                    U_j = obj.shiftUopt(message.data.U_opt,0);
                    obj.neighbors(message.data.id) = 1;
                    % calulate neighbor state estimation
                    Xj_hat = [Xj_hat; obj.statePrediction(x_j,U_j)];
                end
            end
            
            % get obstacles in range
            obj.num_O = 0;
            if ~isempty(obj.obstacles)
                [obj.num_O, betaAgents] = getObstaclesInRange(x_i(1:4), obj.obstacles, obj.ro);
            end
            
            % variable bounds
            ub_U = kron(ones(obj.Hp, 1), obj.u_max);
            lb_U = -1*ub_U;
            ub_X = ones(obj.n*obj.Hp,1)*inf;
            lb_X = -1*ub_X;
            ub_sigma_alpha = inf*ones(2*obj.num_N*obj.Hp,1);
            lb_sigma_alpha = zeros(2*obj.num_N*obj.Hp,1);
            ub_sigma_beta = inf*ones(obj.num_O*obj.Hp,1);
            lb_sigma_beta = zeros(obj.num_O*obj.Hp,1);
            ub = [ub_U; ub_X; ub_sigma_alpha; ub_sigma_beta];
            lb = [lb_U; lb_X; lb_sigma_alpha; lb_sigma_beta];
            
            % reference tracking
            if ~isempty(obj.reference)
                % position reference
                error_r = obj.reference(1:2)-x_i(1:2);
                dist_r = norm(error_r);
                if dist_r < obj.d_l
                    normed = error_r;
                else
                    normed = obj.d_l*error_r/dist_r;
                end
                ref_xy = x_i(1:2) + normed;
                
                % angle reference
                ref_psi = atan2(error_r(2), error_r(1));
            end
            
            % optimization problem
            U0 = obj.shiftUopt(obj.U_opt,0);
            X0 = obj.statePrediction(x_i,U0);
            Z0 = [U0; X0; zeros(2*obj.num_N*obj.Hp,1); zeros(obj.num_O*obj.Hp,1)];
            options = optimoptions('fmincon','Display', 'off');
            options = optimoptions(options,'Algorithm','sqp');
            options = optimoptions(options,'MaxIterations',obj.maxIterations);
            tOptimStart = tic;
            Z = fmincon(@objectiveFunction,Z0,[],[],[],[],lb,ub,@nlcon,options);
            obj.tOptim = obj.tOptim + toc(tOptimStart);
            u = Z(1:2);
            obj.U_opt = Z(1:obj.Hp*obj.m);
            
            
            % Evaluate dynamics
            obj.move(u);
            
            % Send message to network, include position and velocity
            data = struct;
            data.position = obj.state(1:2);
            data.velocity = obj.state(4);
            data.attitude = obj.state(3);
            data.angularVelocity = obj.state(5);
            data.u = u;
            data.num_N = obj.num_N;
            data.id = obj.id;
            data.neighbors = obj.neighbors;
            data.U_opt = obj.U_opt;
            data.tOptim = obj.tOptim;
            obj.send(data)
            
            % cost function
            function [objective, grad_objective] = objectiveFunction(Z)
                % separate optimization variables
                ind_Uend = obj.Hp*obj.m;
                ind_Xend = ind_Uend + obj.Hp*obj.n;
                ind_Salphaend = ind_Xend + 2*obj.Hp*obj.num_N; 
                U = Z(1 : ind_Uend);
                X = Z(ind_Uend+1 : ind_Xend);
                Salpha = Z(ind_Xend+1 : ind_Salphaend);
                Sbeta = Z(ind_Salphaend+1:end);
                Sp = Salpha(1:length(Salpha)/2);
                Sm = Salpha(length(Salpha)/2+1:end);
                
                % input
                R_U = kron(eye(obj.Hp),obj.lambda_u);
                objective = U'*R_U*U;
                grad_U = 2*R_U*U;
                
                % inter-agent
                objective = objective + obj.lambda_attr*(Sp'*Sp) + obj.lambda_rep*(Sm'*Sm);
                grad_Sp = 2*obj.lambda_attr*Sp;
                grad_Sm = 2*obj.lambda_rep*Sm;
                
                % reference tracking
                grad_X = zeros(size(X));
                if ~isempty(obj.reference)
                    Q_r = kron(eye(obj.Hp), blkdiag(obj.q_pos*eye(2), obj.q_psi, obj.q_v, obj.q_dpsi));
                    X_r = kron(ones(obj.Hp,1),[ref_xy; ref_psi; 0; 0]);
                    objective = objective + (X-X_r)'*Q_r*(X-X_r);
                    grad_X = 2*Q_r*X - 2*(X_r'*Q_r)';
                end
                
                % obstacle avoidance
                grad_Sbeta = [];
                if (obj.num_O > 0)
                    objective = objective +  obj.lambda_obs*(Sbeta'*Sbeta);
                    grad_Sbeta = 2*obj.lambda_obs*Sbeta;
                end
                
                % velocity alignment
                s_vel = [obj.e_nm(obj.n,4); obj.e_nm(obj.n,5)];
                for nu = 1:obj.num_N
                    s_n = kron(obj.e_nm(obj.num_N,nu),eye(obj.n*obj.Hp));
                    for hp = 1:obj.Hp
                        xj = s_n*Xj_hat;
                        s_hp = kron(obj.e_nm(obj.Hp,hp),eye(obj.n));
                        dvel = s_vel*s_hp*(X-xj);
                        objective = objective + dvel'*obj.lambda_vel*dvel;
                    end
                end
                
                % gradient construction
                grad_objective = [grad_U; grad_X; grad_Sp; grad_Sm; grad_Sbeta];
            end
            
            % nonlinear constraint function
            function [c, ceq] = nlcon(Z)
                c = [];
                
                % separate optimization variables
                ind_Uend = obj.Hp*obj.m;
                ind_Xend = ind_Uend + obj.Hp*obj.n;
                ind_Salphaend = ind_Xend + 2*obj.Hp*obj.num_N; 
                U = Z(1 : ind_Uend);
                X = Z(ind_Uend+1 : ind_Xend);
                Salpha = Z(ind_Xend+1 : ind_Salphaend);
                Sbeta = Z(ind_Salphaend+1:end);
                Sp = Salpha(1:length(Salpha)/2);
                Sm = Salpha(length(Salpha)/2+1:end);
                
                s_xy = [obj.e_nm(obj.n,1); obj.e_nm(obj.n,2)];
                
                % equality constraint
                ceq = X - obj.statePrediction(x_i,U);
                
                % alpha flocking
                Delta = [];
                epsilon_d = obj.epsilonNorm(obj.d, obj.epsilon);
                for nu = 1:obj.num_N
                    s_n = kron(obj.e_nm(obj.num_N,nu),eye(obj.n*obj.Hp));
                    for hp = 1:obj.Hp
                        s_hp = kron(obj.e_nm(obj.Hp,hp),eye(obj.n));
                        qi = s_xy*s_hp*X;
                        qj = s_xy*s_hp*s_n*Xj_hat;
                        delta_qij = obj.epsilonNorm(qj-qi, obj.epsilon)-epsilon_d;
                        Delta = [Delta;delta_qij];
                    end
                end
                c = [c; -Delta-Sm; Delta-Sp];
                
                % obstacle avoidance
                if (obj.num_O > 0)
                    Delta_obs = [];
                    epsilon_do = obj.epsilonNorm(obj.do, obj.epsilon);
                    for o = 1:obj.num_O
                        s_o = kron(obj.e_nm(obj.num_O,o),[eye(2) 0*eye(2)]);
                        qo = s_o*betaAgents;
                        for hp = 1:obj.Hp
                            s_hp = kron(obj.e_nm(obj.Hp,hp),eye(obj.n));
                            qi = s_xy*s_hp*X;
                            delta_qio = obj.epsilonNorm(qo-qi, obj.epsilon)-epsilon_do;
                            Delta_obs = [Delta_obs;delta_qio];
                        end
                    end
                    c = [c; -Delta_obs-Sbeta];
                end     
            end
        end
    end
end