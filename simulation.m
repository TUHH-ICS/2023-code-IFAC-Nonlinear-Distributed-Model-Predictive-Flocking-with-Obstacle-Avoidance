%---------------------------------------------------------------------------------------------------
% For Paper
% "Nonlinear Distributed Model Predictive Flocking with Obstacle Avoidance"
% by P. Hastedt and H. Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Philipp Hastedt
%---------------------------------------------------------------------------------------------------

clear;
close all;

%% Select Algorithm and Scenario
% Algorithms
% 1: Proposed Predictive SQP Flocking Algorithm
% 2: Non-predictive Olfati-Saber Flocking with Feedback Linearization

% Scenarios
% 1: Free flocking
% 2: Field of obstacles

% Configurations (for Algorithm 1, free flocking)
% 1: Custom configuration (see generateConfig() in /simulation/lib/ )
% 2: NSQP = 1
% 3: NSQP = 3
% 4: NSQP = 10
% 5: NSQP = 100

algorithmIndex  = 2;
scenarioIndex   = 1;
configIndex     = 1;

%% Setup
addpath(genpath('simulation/mas-simulation/lib'))
addpath(genpath('simulation'))
simPath = "simulation/";

algorithms = simPath + [...
    "predictive_sqp_flocking"
    "ndi_flocking"
    ];

preallocate = {...
    @preallocateSqpFlocking
    @preallocateNdiFlocking
    };

generateSetup = {...
    @generateSetupSqpFlocking
    @generateSetupNdiFlocking
    };
%% Set Agent Parameters
cfg = generateConfig(algorithms, algorithmIndex);
if (algorithmIndex == 1)
    switch (configIndex)
        case (2)
            cfg = load(strcat(algorithms(algorithmIndex),"/cfg/")+"config_NSQP_1").cfg;
        case (3)
            cfg = load(strcat(algorithms(algorithmIndex),"/cfg/")+"config_NSQP_3").cfg;
        case (4)
            cfg = load(strcat(algorithms(algorithmIndex),"/cfg/")+"config_NSQP_10").cfg;
        case (5)
            cfg = load(strcat(algorithms(algorithmIndex),"/cfg/")+"config_NSQP_100").cfg;
    end
end

%% Simulation Setup
% define output and initialization files and paths; set simulation
% parameters
outPath = strcat(simPath,"out/",erase(algorithms(algorithmIndex),simPath),"/");
outFile = outPath+"results_3_v2.mat";

% set simulation parameters
param.dimension  = 2;    % Dimension of the space the agents move in
param.range      = 8.4;  % Agent interaction range
param.ro         = 8.4;  % Obstacle interaction range
switch (scenarioIndex)
    case (1)
        initializationFile = simPath+"initialStates_10.mat";
        Tf               = 50;  % Simulation duration [s]
        param.agentCount = 10;  % Number of agents in the network
        param.reference = [];   % reference
        param.obstacles = [];   % obstacles
    case (2)
        initializationFile = simPath+"initialStates_20.mat";
        Tf               = 300;  % Simulation duration [s]
        param.agentCount = 20;   % Number of agents in the network
        param.reference = [85;85;0;0];
        param.obstacles = [ 35  60  30  55  75  15
                            35  35  60  55  45  45
                            6   3   2   1   2   1];
end

% set sampling time
switch (algorithmIndex)
    case (1)
        param.dT         = 0.3;  % Size of the simulation time steps [s]
    case(2)
        param.dT         = 0.02;  % Size of the simulation time steps [s]
end

init = load(initializationFile);
setup = generateSetup{algorithmIndex}(cfg, param, init);

%% Run Simulation
sim = SimulationManager(setup.Network, setup.Agents);
leech = preallocate{algorithmIndex}(setup, sim, Tf);
data = performSimulation(sim, leech, Tf);
out.t = data.t;
out.data = data.data;
save(outFile,'out','setup','param', 'cfg');