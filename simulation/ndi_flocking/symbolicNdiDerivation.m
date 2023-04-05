%---------------------------------------------------------------------------------------------------
% For Paper
% "Nonlinear Distributed Model Predictive Flocking with Obstacle Avoidance"
% by P. Hastedt and H. Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Philipp Hastedt
%---------------------------------------------------------------------------------------------------

clear all;
syms x y psi v dpsi m I_zz d

%% define state and dynamcis
X = [x, y, psi, v, dpsi];

f = [cos(psi)*v - d*sin(psi)*dpsi;
    sin(psi)*v + d*cos(psi)*dpsi;
    dpsi;
    0;
    0];

g = [0 0;
    0 0;
    0 0;
    1/m 0;
    0 1/I_zz];

%% define position output vector
h = [x;
    y];

%% calculate lie derivatives
Lf_h = simplify(jacobian(h,X)*f);
Lg_h = simplify(jacobian(h,X)*g);
LgLf_h = simplify(jacobian(Lf_h,X)*g);
Lf2_h = simplify(jacobian(Lf_h,X)*f);

%% generate matlab function
matlabFunction(Lf2_h, LgLf_h,'File','feedbackLinearization','Vars', {[x; y; psi; v; dpsi], m, I_zz, d});




