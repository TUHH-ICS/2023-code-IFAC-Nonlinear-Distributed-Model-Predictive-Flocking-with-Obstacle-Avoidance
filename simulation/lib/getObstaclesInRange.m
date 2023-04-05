%---------------------------------------------------------------------------------------------------
% For Paper
% "Nonlinear Distributed Model Predictive Flocking with Obstacle Avoidance"
% by P. Hastedt and H. Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Philipp Hastedt
%---------------------------------------------------------------------------------------------------

function [num_O, betaAgents] = getObstaclesInRange(x, obstacles, ro)
if ~isempty(obstacles)
    % Circular Obstacles: row 1: x, row 2: y, row 3: radius
    pos = x(1:length(x)/2);
    
    % find obstacles in range
    obstaclesInRange = [];
    num_O = 0;
    for i = 1:size(obstacles,2)
        distance = norm(pos-obstacles(1:end-1,i)) - obstacles(end,i);
        if (distance <= ro)
            num_O = num_O + 1;
            obstaclesInRange(:,num_O) = obstacles(:,i);
        end
    end
    betaAgents = [];
    if num_O~=0
        % calculate positions of beta agents
        betaAgents = zeros(length(x),num_O);
        for i = 1:num_O
            center = obstaclesInRange(1:end-1,i);
            radius = obstaclesInRange(end,i);
            q_hat =  center + radius*(pos - center)/norm((pos - center));
            betaAgents(:,i) = [q_hat; 0*q_hat];
        end
        betaAgents = reshape(betaAgents,[numel(betaAgents), 1]);
    end
else
    num_O = 0;
    betaAgents = [];
end
end