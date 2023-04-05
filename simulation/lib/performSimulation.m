%---------------------------------------------------------------------------------------------------
% For Paper
% "Nonlinear Distributed Model Predictive Flocking with Obstacle Avoidance"
% by P. Hastedt and H. Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Philipp Hastedt
%---------------------------------------------------------------------------------------------------

function data = performSimulation(sim, leech, Tf)
profile clear;
fprintf('Simulation started.\n')
% Initialize remaining values for the simulation
t = 0;

% Save start time of the simulation. We want to periodically print the
% progress of the simulation.
lastprint = posixtime(datetime('now'));
starttime = lastprint;
tic
% profile on
while t < Tf
    t = sim.step();
    
    % Save current position of all agents
    leech.save(t)
    
    % Print progress every 2 seconds
    if posixtime(datetime('now')) - lastprint >= 5
        lastprint = posixtime(datetime('now'));
        percentFinished = t/Tf;
        timeElapsed = lastprint-starttime;
        timeRemaining = (1-percentFinished)/percentFinished * timeElapsed;
        fprintf('Simulation %d%% finished. Time remaining: %d Seconds\n', floor(100*percentFinished), floor(timeRemaining))
    end
end
% profile viewer
fprintf("Simulation completed in %.3g seconds!\n", toc);

data = leech;
end