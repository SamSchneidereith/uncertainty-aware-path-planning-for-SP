%% Steward Platform Path Planning - W Schneidereith 2026
%% Problem Setup

addpath('planning')
addpath('dataStructures')
addpath('hardware')
addpath('analysis')
addpath('tests')

clc; clear; close all;
SP = StewartPlatform(); cfg = config(SP); 
numTrials = cfg.general.numTrials; maxTime = cfg.general.maxTime;
Data = struct(); Data.Trials = cell(numTrials, 1);

timestamp = char(datetime('now','Format','dMMMy_HH''h''mm'));
fileloc = [pwd, '\results\', timestamp, '\'];
if ~isfolder(fileloc); mkdir(fileloc); end

fprintf('\n=== STEWARD PLATFORM PATH PLANNING CONFIG ===\n');
fprintf('Num Trials:          %d\n', numTrials);
fprintf('Start Time:          %s\n', string(datetime('now','Format','HH:mm')));
fprintf('Time/Trial:          %d s\n', maxTime);
fprintf('Estimated Runtime:   ~%.1f minutes\n', numTrials * maxTime/60);
fprintf('=============================================\n\n');

%% Trial Loop
for trial = 1:numTrials
    fprintf('=== Trial %d/%d ===\n', trial, numTrials)
    
    planner = Planner(cfg, SP);
    Data.Trials{trial} = planner.run();
    
    % Plot Trial
    if cfg.plot.trial
        G = planner.G;
        trialFolder = [pwd, '\results\', timestamp, '\', sprintf('Trial %02d', trial)]; mkdir(trialFolder);
        % fig = G.plotTree(); saveas(fig, fullfile(fileloc, sprintf('Trial %02d', trial),'Tree.png'));
        % fig = G.plotCostTree(); saveas(fig, fullfile(fileloc, sprintf('Trial %02d', trial),'CostTree.png'));
        % fig = G.plotPathTree(); saveas(fig, fullfile(fileloc, sprintf('Trial %02d', trial),'PathTree.png'));
        fig = G.plotPathNoTree(); saveas(fig, fullfile(trialFolder ,'PathNoTree.png'));
        pause(10);
    end
end
disp('Experiment Complete');

%% Analysis and Data Exportation
analysis = ExperimentAnalyzer(Data);

if cfg.plot.exp && numTrials > 1
    fig = analysis.plotCostVsTime(); saveas(fig, fullfile(fileloc, 'CostVsTime.png'));
    fig = analysis.plotCostVsTimeBoxWhisker(12); saveas(fig, fullfile(fileloc, 'CostVsTimeBoxWhisker.png'))
    fig = analysis.plotDistVsTime(); saveas(fig, fullfile(fileloc, 'DistVsTime.png'));
    % fig = analysis.plotUncertaintyVsTime(); saveas(fig, fullfile(fileloc, 'UncertaintyVsTime.png')); % Uncertainty at goal node (not optimized)
    % fig = analysis.plotNodesVsTime(); saveas(fig, fullfile(fileloc, 'NodesVsTime.png'));
    % fig = analysis.plotBallRadVsTime(); saveas(fig, fullfile(fileloc, 'BallRadVsTime.png'));
    % fig = analysis.plotFinalCostHistogram(); saveas(fig, fullfile(fileloc, 'CostHistogram.png'));
end

save([fileloc, 'config.mat'], 'cfg');
save([fileloc, 'Data.mat'], 'Data');