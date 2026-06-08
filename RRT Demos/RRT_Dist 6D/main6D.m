%% Steward Platform Path Planning - W Schneidereith 2026
%% Problem Setup

clc; clear; close all;
cfg = config();
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
    
    % if(trial > ceilt(2*numTrials/3)); cfg.alg.type = 2; elseif(trial > ceil(numTrials/3)); cfg.alg.type = 1; end
    planner = Planner(cfg);
    Data.Trials{trial} = planner.run();
    
    % Plot Trial
    if cfg.plot.trial
        trialFolder = [pwd, '\results\', timestamp, '\', sprintf('Trial %02d', trial)]; mkdir(trialFolder);
        G = planner.G;
        % fig = G.plotTree(); saveas(fig, fullfile(fileloc, sprintf('Trial %02d', trial),'Tree.png'));
        fig = G.plotPathTree(); saveas(fig, fullfile(fileloc, sprintf('Trial %02d', trial),'PathTree.png'));
        fig = G.plotPathNoTree(); saveas(fig, fullfile(trialFolder ,'PathNoTree.png'));
        pause(10);
    end
end
disp('Experiment Complete');

%% Analysis and Data Exportation
% analysis = ExperimentAnalyzer(Data);
% 
% if cfg.plot.exp && numTrials > 1
%     fig = analysis.plotCostVsTime(); saveas(fig, fullfile(fileloc, 'CostVsTime.png'));
%     fig = analysis.plotNodesVsTime(); saveas(fig, fullfile(fileloc, 'NodesVsTime.png'));
%     fig = analysis.plotBallRadVsTime(); saveas(fig, fullfile(fileloc, 'BallRadVsTime.png'));
%     fig = analysis.plotFinalCostHistogram(); saveas(fig, fullfile(fileloc, 'CostHistogram.png'));
% end
% 
% save([fileloc, 'config.mat'], 'cfg');
% save([fileloc, 'Data.mat'], 'Data');

%% Alg Comparision

numAlgs = 3;

algCosts = cell(numAlgs,1);
timeVec = Data.Trials{1}.Time;

for a = 1:numAlgs
    algCosts{a} = [];
end

for t = 1:length(Data.Trials)
    trial = Data.Trials{t};
    alg = trial.alg + 1; % convert 0,1,2 -> 1,2,3
    algCosts{alg} = [algCosts{alg}; trial.Cost'];
end

%% Normalized Cost
c_opt = Planner.distFun(cfg.startNode.position, cfg.goalNode.position, cfg.env.W);
normCost = cell(numAlgs,1);
for a = 1:numAlgs
    normCost{a} = algCosts{a} / c_opt;
end

meanCost = cell(numAlgs,1);
for a = 1:numAlgs
    meanCost{a} = mean(normCost{a},1,'omitnan');
end

fig = figure;
hold on
plot(timeVec, meanCost{1}, 'LineWidth',2)
plot(timeVec, meanCost{2}, 'LineWidth',2)
plot(timeVec, meanCost{3}, 'LineWidth',2)
yline(1,'--k')
legend('RRT','RRT*','RRT#')
xlabel('Time (s)')
ylabel('Normalized Cost')
title('Normalized Path Cost vs Time - 50 Trials')
grid on
saveas(fig, fullfile(fileloc, 'NormalizedCost.png'));

%% Perc. Success
tol = 0.1;
percentSuccess = cell(numAlgs,1);
for a = 1:numAlgs
    trials = normCost{a};
    success = trials <= (1 + tol);
    percentSuccess{a} = mean(success,1)*100;
end

fig = figure;
hold on
plot(timeVec, percentSuccess{1}, 'LineWidth',2)
plot(timeVec, percentSuccess{2}, 'LineWidth',2)
plot(timeVec, percentSuccess{3}, 'LineWidth',2)
legend('RRT','RRT*','RRT#')
xlabel('Time (s)')
ylabel('Success Rate (%)')
% set(gca,'Xscale','log')
title('Probability of Finding Optimal Path - 50 Trials')
grid on
ylim([0 100])
saveas(fig, fullfile(fileloc, 'PercentSuccess.png'));