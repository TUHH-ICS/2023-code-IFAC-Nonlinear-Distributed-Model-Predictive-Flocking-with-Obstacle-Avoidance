%---------------------------------------------------------------------------------------------------
% For Paper
% "Nonlinear Distributed Model Predictive Flocking with Obstacle Avoidance"
% by P. Hastedt and H. Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Philipp Hastedt
%---------------------------------------------------------------------------------------------------

function [Jq,Jp] = calculatePerformanceIndices(data, rc, d, plotResults, hasHandle, varargin)
agentCount = length(data.data.position(1,1,:));
Jq = zeros(1,length(data.t));
Jp = zeros(1,length(data.t));
for t = 1:length(data.t)
    position = data.data.position;
    velocity = data.data.velocity;
    if nargin > 4
       if hasHandle
           position = data.data.positionHandle;
           velocity = data.data.velocityHandle;
       end
    end

    % calculate position performance index
    countQ = 0;
    for i  = 1:agentCount
        for j = 1:agentCount
            qij = norm(position(t,:,i)-position(t,:,j));
            if (qij<=rc) && (i~=j)
                Jq(t) = Jq(t)+(qij-d)^2;
                countQ = countQ+1;
            end
        end
    end
    if countQ ~=0
        Jq(t) = Jq(t)/countQ;
    end
    % calculate velocity performance index
    for i = 1:agentCount
        p_mean = sum(squeeze(velocity(t,:,:)),2)/agentCount;
        Jp(t) = Jp(t) + norm(velocity(t,:,i)-p_mean')^2/agentCount;
    end
end
if plotResults
    figure()
    subplot(1,2,1)
    plot(data.t, Jq);
    xlabel('time');
    ylabel('J_q');
    title('Position Irregularity');
    grid on;
    subplot(1,2,2)
    plot(data.t, Jp);
    xlabel('time');
    ylabel('J_p');
    title('Velocity Mismatch');
    grid on;
    set(gca, 'XLim', [0 data.t(end-1)]);
end

end