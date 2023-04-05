%---------------------------------------------------------------------------------------------------
% For Paper
% "Nonlinear Distributed Model Predictive Flocking with Obstacle Avoidance"
% by P. Hastedt and H. Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Philipp Hastedt
%---------------------------------------------------------------------------------------------------

function U = shiftUopt(obj, U_opt, u_end)
%SHIFTU_OPT shift U_opt one step in the future
% Inputs
%   U_opt   :   last step optimal input
%   u_end   :   assumption for last u value
% Outputs
%   U       :   shiftet (warm start) U_opt
U = zeros(size(U_opt));
U(1:end-obj.m) = U_opt(obj.m+1:end);
U(end-(obj.m-1):end) = ones(obj.m,1)*u_end;
end
