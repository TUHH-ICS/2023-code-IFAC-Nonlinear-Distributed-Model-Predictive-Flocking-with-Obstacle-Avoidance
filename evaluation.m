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

%% Set evaluation index to reproduce paper results

% videos are available in the "root/video" directory

% Evaluation Scenario
% 1: Number of SQP interation
% 2: Performance comparison

evaluationIndex = 2;

%% Select Data to Evaluate
addpath(genpath('evaluation'),genpath('simulation'))
dataPath = "evaluation/data/";

% Available Data
simData = [
    "NSQP_1"        % 1
    "NSQP_3"        % 2
    "NSQP_10"       % 3
    "NSQP_100"      % 4
    "SQP_obstacle"  % 5
    "NDI_obstacle"  % 6
    ];

%% Comparison/Evaluation
switch (evaluationIndex)
    % 1: Number of SQP interation
    case (1)
        dataSelection = [1, 2, 3, 4];
        t = {};
        distN = {};
        distO = {};
        Jq = {};
        for i=1:length(dataSelection)
            load(dataPath+simData(dataSelection(i)));
            t{i} = out.t;
            [Jq{i},~] = calculatePerformanceIndices(out,8.4,7,0);
        end
        
        % plot Jq
        figure()
        for i = 1:length(dataSelection)
            plot(t{i},Jq{i}, 'DisplayName',replace(erase(simData(dataSelection(i)),".mat"),"_","\_"));
            hold on;
        end
        legend show;
        title('Comparison of convergence for different number of SQP iterations')
        xlabel('time (s)')
        ylabel('J_q')
        grid on;
        xlim([0 15]);
        
        % average optimization time comparison
        names = {};
        for i = 1:length(dataSelection-1)
            load(dataPath+simData(dataSelection(i)));
            optimizationTime(i) = sum(out.data.tOptim(end,1,:)) / (length(t{i})*size(out.data.tOptim,3));
            names{i} = replace(erase(simData(dataSelection(i)),".mat"),"_","\_");
        end
        figure()
        bar(1:i,optimizationTime);
        set(gca, 'XTick', 1:length(names),'XTickLabel',names);
        title('Average Optimization Time')
        
        % 2: Performance comparison
    case (2)
        dataSelection = [5,6];
        t = {};
        distN = {};
        distO = {};
        for i=1:length(dataSelection)
            load(dataPath+simData(dataSelection(i)));
            t{i} = out.t;
            [distN{i},distO{i}] = calculateMinimumDistances(out,param,0,7,6,0);
            distO{i}=changem(distO{i},nan);
        end
        
        % plot agent paths
        for j=1:length(dataSelection)
            figure()
            load(dataPath+simData(dataSelection(j)));
            viscircles(param.obstacles(1:2,:)',param.obstacles(3,:),'Color','black', 'LineWidth', 1); hold on;
            for i = 1:size(out.data.position,3)
                plot(out.data.position(:,1,i),out.data.position(:,2,i),'b'); hold on;
            end
            plot(squeeze(out.data.position(1,1,:)),squeeze(out.data.position(1,2,:)),'kx','MarkerSize',10,'LineWidth',1); hold on;
            plot(squeeze(out.data.position(end,1,:)),squeeze(out.data.position(end,2,:)),'kx','MarkerSize',10,'LineWidth',1); hold on;
            title("Agent Trajectories "+ replace(erase(simData(dataSelection(j)),".mat"),"_","\_"));
            xlabel('x');
            ylabel('y');
            xlim([-25 120]);
            ylim([-25 120]);
        end
        
        % minimum distances
        figure()
        xlabel('time');
        ylabel('distance');
        for i = 1:size(distN,2)
            plot(t{i},distN{i}, 'DisplayName',replace(erase(simData(dataSelection(i)),".mat"),"_","\_"));
            hold on;
            plot(t{i},distO{i}, '--','DisplayName',replace(erase(simData(dataSelection(i)),".mat"),"_","\_"));
            hold on;
        end
        legend show;
        grid on;
        xlim([0 200]);
        
        % obstacle clearance time
        names = {};
        for i = 1:length(dataSelection)
            load(dataPath+simData(dataSelection(i)));
            indices = find((~isnan(distO{i}))>0.5);
            clearanceTime(i) = t{i}(indices(end))-t{i}(indices(1));
            names{i} = replace(erase(simData(dataSelection(i)),".mat"),"_","\_");
        end
        figure()
        bar(1:i,clearanceTime);
        set(gca, 'XTick', 1:length(names),'XTickLabel',names);
        title('Obstacle Clearance Time')
        
        % input comparison
        names = {};
        for i = 1:length(dataSelection)
            load(dataPath+simData(dataSelection(i)));
            u_rms(i) = calculateTotalInputUsed(out);
            names{i} = replace(erase(simData(dataSelection(i)),".mat"),"_","\_");
        end
        figure()
        bar(1:i,u_rms);
        set(gca, 'XTick', 1:length(names),'XTickLabel',names);
        title('RMS Input Values')
end