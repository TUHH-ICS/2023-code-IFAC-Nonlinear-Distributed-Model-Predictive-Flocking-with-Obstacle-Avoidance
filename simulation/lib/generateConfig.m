%---------------------------------------------------------------------------------------------------
% For Paper
% "Nonlinear Distributed Model Predictive Flocking with Obstacle Avoidance"
% by P. Hastedt and H. Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Philipp Hastedt
%---------------------------------------------------------------------------------------------------

function cfg = generateConfig(algorithms, index)

% Algorithms
% 1: Proposed Predictive SQP Flocking Algorithm
% 2: Non-predictive Olfati-Saber Flocking with Feedback Linearization

% common parameters
cfg.d       = 7;        % desired inter-agent distance
cfg.do      = 6;        % desired obstacle separation
cfg.m       = 2;        % space dimension
cfg.u_max   = [1;1];    % input constraint
cfg.mass = 1;           % mass                             
cfg.inertia = 0.25;     % inertia

switch(index)
    % Proposed Predictive SQP Flocking Algorithm
    case (1)
        cfg.Hp = 5;                                 % prediction horizon          
        cfg.epsilon = 10;                           % epsilon-map parameter
        cfg.lambda_u = diag([0.2 0.1]);             % input weight
        cfg.lambda_attr = 0.1;                      % attractive inter-agent weight
        cfg.lambda_rep = 1;                         % repulsive inter-agent weight
        cfg.lambda_obs = 1.2;                       % obstacle repulsion weight
        cfg.lambda_vel = diag([0.002 0.002]);       % velocity alignment weight
        cfg.d_l = 2;                                % look ahead distance for reference tracking
        cfg.q_pos = 0.1;                            % reference position weight
        cfg.q_psi = 0;                              % reference velocity weight
        cfg.q_v = 0.002;                            % reference velocity weight
        cfg.q_dpsi = 0.001;                         % reference velocity weight
        cfg.maxIterations = 10;                      % max SQP iterations
        
    % Non-predictive Olfati-Saber Flocking with Feedback Linearization    
    case (2)
        cfg.handleLength = 0.5;     % handle length
        cfg.isSaturated = true;     % saturation flag
        cfg.ha = 0.2;               % alpha bump function parameter 
        cfg.hb = 0.9;               % beta bump function parameter
        cfg.epsilon_sigma = 0.1;    % sigma-norm parameter
        cfg.c1a = 0.6;              % alpha position gain
        cfg.c2a= 2*sqrt(cfg.c1a);   % alpha velocity gain
        cfg.c1b = 0.55;             % beta position gain
        cfg.c2b = 0;                % beta velocity gain
        cfg.c1g = 0.32;             % gamma position gain
        cfg.c2g = 0.17;             % gamma velocity gain
        cfg.pot_a = 2;              % potential field parameter
        cfg.pot_b = 3;              % potential field parameter
end
save(strcat(algorithms(index),'/cfg/config.mat'),'cfg');
end

