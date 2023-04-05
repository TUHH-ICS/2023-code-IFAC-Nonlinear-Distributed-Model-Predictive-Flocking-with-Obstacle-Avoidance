%---------------------------------------------------------------------------------------------------
% For Paper
% "Nonlinear Distributed Model Predictive Flocking with Obstacle Avoidance"
% by P. Hastedt and H. Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Philipp Hastedt
%---------------------------------------------------------------------------------------------------

function rms = calculateTotalInputUsed(out)
data = out.data;
agentCount = size(data.u,3);
rms = 0;
for i = 1:agentCount
    rms = rms + sqrt(sum((data.u(:,:,i)).^2,'all')/(size(data.u,1)*size(data.u,2)));
end

end