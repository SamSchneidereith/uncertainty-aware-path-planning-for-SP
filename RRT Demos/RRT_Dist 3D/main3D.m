%% Steward Platform Path Planning W Schneidereith
%% Problem Setup

clc; clear; close all;
cfg = config();
numTrials = cfg.general.numTrials; maxTime = cfg.general.maxTime;
Data = struct(); Data.Trials = cell(numTrials, 1);

timestamp = char(datetime('now','Format','dMMMy_HH''h''mm'));
fileloc = [pwd, '\results\', timestamp, '\'];
% if ~isfolder(fileloc); mkdir(fileloc); end

fprintf('\n=== STEWARD PLATFORM PATH PLANNING CONFIG ===\n');
fprintf('Num Trials:          %d\n', numTrials);
fprintf('Start Time:          %s\n', string(datetime('now','Format','HH:mm')));
fprintf('Time/Trial:          %d s\n', maxTime);
fprintf('Estimated Runtime:   ~%.1f minutes\n', numTrials * maxTime/60);
fprintf('=============================================\n\n');

%% Trial Loop
for trial = 1:numTrials
    fprintf('=== Trial %d/%d ===\n', trial, numTrials)
    
    planner = Planner(cfg);
    Data.Trials{trial} = planner.run();
    
    % Plot Trial
    if cfg.plot.trial
        G = planner.G;
        % fig = G.plotTree();
        fig = G.plotPathTree();
        % fig = G.plotPathNoTree();
        % saveas(fig, fullfile(fileloc, 'FullTree.png'));
        pause(5);
    end
end
disp('Experiment Complete');

%% Analysis and Data Exportation
% analysis = ExperimentAnalyzer(Data, cfg.startPosition, cfg.goalPosition, cfg.general.maxTime, cfg.env.dimensionMins, cfg.env.dimensionMaxs);
% analysis.extractStats();
% 
% if cfg.plot.exp && numTrials > 1
%     fig = analysis.plotCostVsTime();
%     fig = analysis.plotNodesVsTime();
%     fig = analysis.plotFinalCostHistogram();
%     % saveas(fig, fullfile(fileloc, 'FullTree.png'));
%     disp('Plotting Complete');
% end

% save([fileloc, 'config.mat'], 'cfg');
% save([fileloc, 'Data.mat'], 'Data');