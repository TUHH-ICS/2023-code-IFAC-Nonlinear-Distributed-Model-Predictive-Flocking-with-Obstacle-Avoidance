%---------------------------------------------------------------------------------------------------
% For Paper
% "Nonlinear Distributed Model Predictive Flocking with Obstacle Avoidance"
% by P. Hastedt and H. Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Philipp Hastedt
%---------------------------------------------------------------------------------------------------

function [minDistNeighbors, minDistObstacles] = calculateMinimumDistances(out, param, plotResults, d, do, hasHandle, varargin)
data = out.data;
time = out.t;
minDistNeighbors = zeros(1,length(time));
minDistObstacles = zeros(1,length(time));

% Calculate minimum distance to neighbors
for t = 1:length(time)
    position = data.position;
    velocity = data.velocity;
    if nargin > 5
       if hasHandle
           position = data.positionHandle;
           velocity = data.velocityHandle;
       end
    end
    for i  = 1:param.agentCount
        % calculate minimum inter agent distance
        for j = 1:param.agentCount
            qij = norm(position(t,:,i)-position(t,:,j));
            if (qij<=param.range) && (i~=j)
                if minDistNeighbors(t) == 0
                    minDistNeighbors(t) = qij;
                end
                minDistNeighbors(t) = min(qij,minDistNeighbors(t));
            end
        end
        
        % minimum obstacle separation
        state = [position(t,:,i), velocity(t,:,i)]';
        [num_O, beta] = getObstaclesInRange(state, param.obstacles, param.ro);
        for o = 1:num_O
            stateDif = state - beta((num_O-1)*2*param.dimension + 1: num_O*2*param.dimension);
            qio = norm(stateDif(1:param.dimension));
            if (qio<=param.ro)
                if minDistObstacles(t) == 0
                    minDistObstacles(t) = qio;
                end
                minDistObstacles(t) = min(qio,minDistObstacles(t));
            end
        end
        
    end
    
end

if plotResults
    figure()
    subplot(1,2,1)
    plot(time, minDistNeighbors, 'LineWidth',1); hold on;
    plot(time,ones(size(time))*d,'r--');
    xlabel('time');
    ylabel('qij_{min}');
    title('minimum inter-agent distance');
    grid on;
    subplot(1,2,2)
    plot(time, minDistObstacles,'LineWidth',1); hold on;
    plot(time,ones(size(time))*do,'r--');
    xlabel('time');
    ylabel('qio_{min}');
    title('minimum obstacle separation');
    grid on;
    set(gca, 'XLim', [0 time(end-1)]);
end

end
