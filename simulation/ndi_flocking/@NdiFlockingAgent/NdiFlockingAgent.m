%---------------------------------------------------------------------------------------------------
% For Paper
% "Nonlinear Distributed Model Predictive Flocking with Obstacle Avoidance"
% by P. Hastedt and H. Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Philipp Hastedt
%---------------------------------------------------------------------------------------------------

classdef NdiFlockingAgent < DynamicUnicycle
    
    % Define constants of the flocking protocol
    properties
        da;                 % desired inter agent distance
        db;                 % desired obstacle separation
        ra;                 % agent interaction range
        rb;                 % obstacle interaction range
        ha;                 % alpha bump function parameter 
        hb;                 % beta bump function parameter
        epsilon_sigma;      % sigma-norm parameter
        da_sigma;           % sigma-norm of da
        db_sigma;           % sigma-norm of db
        ra_sigma;           % sigma-norm of ra
        rb_sigma;           % sigma-norm of rb
        c1a;                % alpha position gain
        c1b;                % beta position gain
        c1g;                % gamma position gain 
        c2a;                % alpha velocity gain
        c2b;                % beta velocity gain
        c2g;                % gamma velocity gain
        pot_a;              % potential field parameter
        pot_b;              % potential field parameter
        
        u_max;              % maximum admissible input
        isSaturated;        % saturation flag
    end
    
    % data to be transmitted in addition to position, velocity, and u
    properties
        num_N;          % number of neighbors
        num_O;          % number of obstacles
        numberAgents;   % total number of agents
        neighbors;      % agent ids of neighbors
        reference;      % reference information
        obstacles;      % obstacle information
        positionHandle; % position with handle
        velocityHandle; % velocity of handle point
    end
    
    % member variables
    properties(GetAccess = private, SetAccess = private)
        T;              % sampling time
        n;              % state dimension
        m;              % input dimension
        handleLength;   % handle length      
    end
    
    methods(Static)
        function e_nm = e_nm(n,m)
            e_nm = zeros(1,n);
            e_nm(m) = 1;
        end
    end
    
    methods
        function obj = NdiFlockingAgent(id, param, initPos, initVel, initAtt, initAngVel, cfg)
            % call parent constructor
            obj@DynamicUnicycle(id, param.dT, cfg.mass, cfg.inertia, initPos, initAtt, initVel, initAngVel);
            
            % load parameters
            obj.da = cfg.d;
            obj.db = cfg.do;
            obj.ra = param.range;
            obj.rb = param.ro;
            obj.ha = cfg.ha;
            obj.hb = cfg.hb;
            obj.epsilon_sigma = cfg.epsilon_sigma;
            
            obj.c1a = cfg.c1a;
            obj.c2a = cfg.c2a;
            obj.c1b = cfg.c1b;
            obj.c2b = cfg.c2b;
            obj.c1g = cfg.c1g;
            obj.c2g = cfg.c2g;
            obj.pot_a = cfg.pot_a;
            obj.pot_b = cfg.pot_b;
            
            obj.u_max = cfg.u_max;
            obj.isSaturated = cfg.isSaturated;
            
            % calculate sigma values
            obj.da_sigma = sigma_norm(obj.da, obj.epsilon_sigma);
            obj.db_sigma = sigma_norm(obj.db, obj.epsilon_sigma);
            obj.ra_sigma = sigma_norm(obj.ra, obj.epsilon_sigma);
            obj.rb_sigma = sigma_norm(obj.rb, obj.epsilon_sigma);
            
            % initialize member variables
            obj.T = param.dT;
            obj.m = 2;
            obj.n = 5;
            obj.num_N = 0;
            obj.numberAgents = param.agentCount;
            obj.obstacles = param.obstacles;
            obj.handleLength = cfg.handleLength;
            
            % set reference
            if numel(param.reference) == 0
                obj.reference = [];
            elseif size(param.reference,2) == 1
                obj.reference = param.reference;
            else
                obj.reference = param.reference(:,id);
            end
            
            % initialize handle state
            [obj.positionHandle, obj.velocityHandle] = obj.getStateWithHandle(obj.handleLength);
        end
        
        function step(obj)
            u = zeros(obj.m, 1);
            obj.neighbors = zeros(obj.numberAgents,1);
            % Receive messages from the network
            messages = obj.receive();
            obj.num_N = length(messages);

            xH_i = [obj.positionHandle; obj.velocityHandle];
            
            xH_j = [];
            for message = messages
                obj.neighbors(message.data.id) = 1;
                xH_j = [xH_j; message.data.positionHandle; message.data.velocityHandle];
            end
            s_pos = kron([1 0],eye(2));
            s_vel = kron([0 1],eye(2));
            
            % alpha flocking component
            if obj.num_N ~= 0
                for nu = 1:obj.num_N
                    s_n = kron(obj.e_nm(obj.num_N,nu),eye(2*obj.m));
                    % compute sigma distance and gradient
                    xij = s_n*xH_j - xH_i;
                    qij = s_pos*xij;
                    pij = s_vel*xij;
                    [qij_sigma, grad_qij_sigma] = sigma_norm(qij, obj.epsilon_sigma);
                    
                    % calculate u_alpha_q
                    u_alpha_q = obj.c1a * phi_alpha(qij_sigma, obj.ra_sigma, obj.da_sigma, obj.ha, obj.pot_a, obj.pot_b) * grad_qij_sigma;
                    
                    % calculate u_alpha_p
                    aij = rho_h(qij_sigma / obj.ra_sigma, obj.ha);
                    u_alpha_p = obj.c2a * aij * pij;
                    
                    % add to overall input
                    u = u + u_alpha_q + u_alpha_p;
                end
            end
            
            % beta flocking component
            if ~isempty(obj.obstacles)
                [obj.num_O, betaAgents] = getObstaclesInRange(xH_i, obj.obstacles, obj.rb);
                if obj.num_O ~= 0
                    betaAgents = reshape(betaAgents,2*obj.m,[]);
                    betaAgents(obj.m+1:end,:) = [];
                    for i = 1:obj.num_O
                        % compute sigma distance and gradient
                        qio = betaAgents(:,i) - s_pos*xH_i;
                        [qio_sigma, grad_qio_sigma] = sigma_norm(qio, obj.epsilon_sigma);
                        
                        % calculate u_beta_q
                        u_beta_q = obj.c1b * phi_beta(qio_sigma, obj.db_sigma, obj.hb) * grad_qio_sigma;
                        
                        % add to overall input
                        u = u + u_beta_q;
                    end
                end
            end
            
            % gamma flocking component
            if ~isempty(obj.reference)
                qir = obj.reference(1:obj.m) - s_pos*xH_i;
                pir = obj.reference(obj.m+1:end) - s_vel*xH_i;
                [~, grad_qir_sigma] = sigma_norm(qir, 1);
                
                % calculate u_gamma_q
                u_gamma_q = obj.c1g * grad_qir_sigma;
                
                % calculate u_gamma_p
                u_gamma_p = obj.c2g * pir;
                
                % add to overall input
                u = u + u_gamma_q + u_gamma_p;
            end
            
            % NDI lineraization
            [Lf2_h,LgLf_h] = feedbackLinearization(obj.state, obj.M, obj.Iz, obj.handleLength);
            u = LgLf_h\(u-Lf2_h);
            
            % saturation
            if obj.isSaturated
                for i = 1:obj.m
                    u(i) = max(-obj.u_max(i), min(obj.u_max(i), u(i)));
                end
            end
            
            % Evaluate double integrator dynamics
            obj.move(u);
            
            % calculate handle state
            [obj.positionHandle, obj.velocityHandle] = obj.getStateWithHandle(obj.handleLength);
            
            % Send message to network, include position and velocity
            data = struct;
            data.position = obj.state(1:2);
            data.velocity = obj.state(4);
            data.attitude = obj.state(3);
            data.angularVelocity = obj.state(5);
            data.positionHandle = obj.positionHandle;
            data.velocityHandle = obj.velocityHandle;
            data.u = u;
            data.num_N = obj.num_N;
            data.id = obj.id;
            data.neighbors = obj.neighbors;
            obj.send(data)
        end
    end
end

