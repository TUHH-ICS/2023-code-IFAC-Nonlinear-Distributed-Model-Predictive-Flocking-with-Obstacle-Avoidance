%---------------------------------------------------------------------------------------------------
% For Paper
% "Nonlinear Distributed Model Predictive Flocking with Obstacle Avoidance"
% by P. Hastedt and H. Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Philipp Hastedt
%---------------------------------------------------------------------------------------------------

function X = statePrediction(obj, x0, U)
X = x0;
U = reshape(U,2,[])';
F = U(:,1);
Tau = U(:,2);
for i = 1:obj.Hp
    x_prev = X(end-4:end);
    
    x =     x_prev(1) + obj.tau*( cos(x_prev(3))*x_prev(4) - x_prev(5) * obj.handle * sin(x_prev(3)) );
    y =     x_prev(2) + obj.tau*( sin(x_prev(3))*x_prev(4) + x_prev(5) * obj.handle * cos(x_prev(3)) );
    psi =   x_prev(3) + obj.tau*x_prev(5);
    u =     x_prev(4) + obj.tau*F(i)/obj.M;    
    dpsi =  x_prev(5) + obj.tau*Tau(i)/obj.Iz;

    X = [X;x;y;psi;u;dpsi];
end
X = X(obj.n+1:end);
end

