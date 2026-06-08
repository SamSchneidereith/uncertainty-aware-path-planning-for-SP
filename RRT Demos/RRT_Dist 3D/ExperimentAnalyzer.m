classdef ExperimentAnalyzer

properties
    Data
    numTrials
    startPosition
    goalPosition
    % maxTime
    dimensionMins
    dimensionMaxs
    minCost
    stats, paths
end
methods

    function obj = ExperimentAnalyzer(Data, startPosition, goalPosition, maxTime, dimMins, dimMaxs)
        obj.Data = Data;
        obj.numTrials = length(Data.Trials);
        obj.startPosition = startPosition;
        obj.goalPosition  = goalPosition;
        % obj.maxTime       = maxTime;
        obj.dimensionMins = dimMins;
        obj.dimensionMaxs = dimMaxs;
        [obj.stats, obj.paths] = obj.extractStats();
    end

    % function runAll(obj)
    %     S = obj.extractTrialStats();
    % 
    %     obj.plotHistograms(S);
    %     obj.plotErrorCurves(S);
    %     obj.plotTreeGrowth(S);
    %     obj.plotBallRadius(S);
    %     obj.plotFinalPaths(S);
    % end

    function [S, P] = extractStats(obj)

        N = obj.numTrials;

        S.finalcost  = NaN(N,1);
        S.firstTime = NaN(N,1);
        S.timeCurves = cell(N,1);
        S.nodeCurves = cell(N,1);
        S.costCurves  = cell(N,1);
        P.finalPath = cell(N,1);

        for i = 1:N
            t   = obj.Data.Trials{i}.Time;
            nn  = obj.Data.Trials{i}.NumNodes;
            cost= obj.Data.Trials{i}.Cost;
            % br  = obj.Data.Trials{i}.BallRad;

            S.timeCurves{i} = t;
            S.nodeCurves{i} = nn;
            % S.ballRadCurves{i} = br;

            idxFirst = find(~isnan(cost) & ~isinf(cost),1,'first');

            if isempty(idxFirst)
                continue;
            end

            idxLast = find(~isnan(cost) & ~isinf(cost),1,'last');

            S.finalCost(i)  = cost(idxLast);
            S.firstTime(i) = t(idxFirst);
            S.costCurves{i} = cost(idxFirst:idxLast);

            if ~isempty(obj.Data.Trials{i}.Path) && ...
               length(obj.Data.Trials{i}.Path) >= idxLast && ...
               ~isempty(obj.Data.Trials{i}.Path{idxLast})
                path = obj.Data.Trials{i}.Path{idxLast};
                P.finalPath{i} = obj.Data.Trials{i}.Tree.nodes(path);
            end
        end

        fprintf('Successful trials: %d / %d\n', ...
            sum(~isnan(S.finalCost)), N);
    end

   function fig = plotCostVsTime(obj)
        fig = figure('Name','CostVsTime','Units','inches','Position',[0 0 6 4]);
        movegui(fig,'center');
        hold on; grid on;
    
        N = obj.numTrials;
        allCurves = obj.stats.costCurves;
        maxLen = max(cellfun(@length, allCurves));
    
        M = NaN(N, maxLen);
    
        for i = 1:N
            c = allCurves{i};
            if ~isempty(c)
                M(i,1:length(c)) = c(:);
                plot(1:length(c), c);
            end
        end
    
        mu = mean(M,1,'omitnan');
        % plot(1:maxLen, mu, 'k', LineWidth = 2);
    
        xlabel('Time Step');
        ylabel('Cost');
        title('Cost vs Time');
    end

    function fig = plotNodesVsTime(obj)
        fig = figure('Name','NodesVsTime','Units','inches','Position',[0 0 6 4]);
        movegui(fig,'center');
        hold on; grid on;
    
        N = obj.numTrials;
        maxLen = 0;
    
        for i = 1:N
            nn = obj.Data.Trials{i}.NumNodes;
            if ~isempty(nn)
                plot(1:length(nn), nn);
                maxLen = max(maxLen, length(nn));
            end
        end
    
        % M = NaN(N, maxLen);
        % for i = 1:N
        %     nn = obj.Data.Trials{i}.NumNodes;
        %     if ~isempty(nn)
        %         M(i,1:length(nn)) = nn(:);
        %     end
        % end
    
        % mu = mean(M,1,'omitnan');
        % plot(1:maxLen, mu, 'k', LineWidth = 2);
    
        xlabel('Time Step');
        ylabel('Number of Nodes');
        title('Tree Growth vs Time');
    end

    function fig = plotFinalCostHistogram(obj)
        vals = obj.stats.finalCost;
        vals = vals(isfinite(vals));
    
        fig = figure('Name','FinalCostHistogram','Units','inches','Position',[0 0 6 4]);
        movegui(fig,'center');
        hold on; grid on;
    
        subplot(1,2,1);
        vals = obj.stats.finalCost(~isnan(obj.stats.finalCost));
        if isempty(vals)
            title('Final Path Error (No Successful Trials)');
            grid on;
        else
            histogram(vals, 'Normalization','pdf'); hold on;
            mu = mean(vals); sigma = std(vals);
            xfit = linspace(min(vals), max(vals), 200);
            yfit = normpdf(xfit, mu, sigma);
            plot(xfit, yfit, 'r', 'LineWidth', 2);
            title('Final Path Error');
            xlabel('Error'); ylabel('PDF');
            grid on;
        end

        % --- Time to First Path ---
        subplot(1,2,2);
        vals = obj.stats.firstTime(~isnan(obj.stats.firstTime));
        if isempty(vals)
            title('Time to First Path (No Successful Trials)');
            grid on;
        else
            histogram(vals, 'Normalization','pdf'); hold on;
            mu = mean(vals); sigma = std(vals);
            xfit = linspace(min(vals), max(vals), 200);
            yfit = normpdf(xfit, mu, sigma);
            plot(xfit, yfit, 'r', 'LineWidth', 2);
            title('Time to First Path');
            xlabel('Time (s)'); ylabel('PDF');
            grid on;
        end
    end
end
end


% function expAnalysis(Data, startPosition, goalPosition, ax_bds, fileloc)
% 
%     numTrials = numel(Data);
%     finalErr  = NaN(numTrials,1);
%     firstTime = NaN(numTrials,1);
%     timeCurves = cell(numTrials,1);
%     nodeCurves = cell(numTrials,1);
%     errCurves  = cell(numTrials,1);
%     finalPath = cell(numTrials,1);
%     nodePositions = cell(numTrials,1);
% 
%     % Extract Data Safely
%     for i = 1:numTrials
%         err = Data(i).PathErr;
%         t   = Data(i).Time;
%         nn  = Data(i).NumNodes;
%         br  = Data(i).BallRad;
% 
%         timeCurves{i} = t;
%         nodeCurves{i} = nn;
%         ballRadCurves{i} = br;
% 
%         idxFirst = find(~isnan(err),1,'first');
% 
%         if isempty(idxFirst)
%             finalPath{i} = [];
%             nodePositions{i} = [];
%             continue;
%         end
% 
%         idxLast = find(~isnan(err),1,'last');
% 
%         finalErr(i)  = err(idxLast);
%         firstTime(i) = t(idxFirst);
%         errCurves{i} = err(idxFirst:idxLast);
% 
%         if ~isempty(Data(i).Path) && ...
%            length(Data(i).Path) >= idxLast && ...
%            ~isempty(Data(i).Path{idxLast})
% 
%             finalPath{i} = Data(i).Path{idxLast};
%             nodePositions{i} = Data(i).Tree.NodePositions;
%         else
%             finalPath{i} = [];
%             nodePositions{i} = [];
%         end
%     end
% 
%     numSuccess = sum(~isnan(finalErr));
%     disp(['Successful trials: ', num2str(numSuccess), ...
%           ' / ', num2str(numTrials)]);
% 
%     %% ===========================
%     % Figure 1: Histograms
%     % ===========================
%     figure('Name','Figure1_Histograms');
% 
%     % --- Final Path Error ---
%     subplot(1,2,1);
%     vals = finalErr(~isnan(finalErr));
%     if isempty(vals)
%         title('Final Path Error (No Successful Trials)');
%         grid on;
%     else
%         histogram(vals, 'Normalization','pdf'); hold on;
%         mu = mean(vals); sigma = std(vals);
%         xfit = linspace(min(vals), max(vals), 200);
%         yfit = normpdf(xfit, mu, sigma);
%         plot(xfit, yfit, 'r', 'LineWidth', 2);
%         title('Final Path Error');
%         xlabel('Error'); ylabel('PDF');
%         grid on;
%     end
% 
%     % --- Time to First Path ---
%     subplot(1,2,2);
%     vals = firstTime(~isnan(firstTime));
%     if isempty(vals)
%         title('Time to First Path (No Successful Trials)');
%         grid on;
%     else
%         histogram(vals, 'Normalization','pdf'); hold on;
%         mu = mean(vals); sigma = std(vals);
%         xfit = linspace(min(vals), max(vals), 200);
%         yfit = normpdf(xfit, mu, sigma);
%         plot(xfit, yfit, 'r', 'LineWidth', 2);
%         title('Time to First Path');
%         xlabel('Time (s)'); ylabel('PDF');
%         grid on;
%     end
% 
%     saveas(gcf, [fileloc 'Figure1_Histograms.png']);
% 
%     %% ===========================
%     % Figure 2: Error Curves
%     % ===========================
%     figure('Name','Figure2_ErrorCurves');
% 
%     % --- Error vs Time ---
%     subplot(1,2,1); hold on; grid on;
%     for i = 1:numTrials
%         if ~isempty(errCurves{i})
%             idxFirst = find(~isnan(Data(i).PathErr),1,'first');
%             if ~isempty(idxFirst)
%                 idxLast = idxFirst + length(errCurves{i}) - 1;
%                 plot(Data(i).Time(idxFirst:idxLast), errCurves{i});
%             end
%         end
%     end
%     xlabel('Time (s)');
%     ylabel('Path Error');
%     title('Error vs Time');
% 
%     % --- Error vs Nodes ---
%     subplot(1,2,2); hold on; grid on;
%     for i = 1:numTrials
%         if ~isempty(errCurves{i})
%             idxFirst = find(~isnan(Data(i).PathErr),1,'first');
%             if ~isempty(idxFirst)
%                 idxLast = idxFirst + length(errCurves{i}) - 1;
%                 plot(Data(i).NumNodes(idxFirst:idxLast), errCurves{i});
%             end
%         end
%     end
%     xlabel('Number of Nodes');
%     ylabel('Path Error');
%     title('Error vs Nodes');
% 
%     saveas(gcf, [fileloc 'Figure2_ErrorCurves.png']);
% 
%     %% ===========================
%     % Figure 3: Tree Growth over time
%     % ===========================
%     figure('Name','Figure3_TreeGrowthvsTime'); 
%     hold on; grid on;
% 
%     for i = 1:numTrials
%         if ~isempty(timeCurves{i}) && ~isempty(nodeCurves{i})
%             plot(timeCurves{i}, nodeCurves{i});
%         end
%     end
% 
%     xlabel('Time (s)');
%     ylabel('Number of Nodes');
%     title('Tree Growth vs Time');
% 
%     saveas(gcf, [fileloc 'Figure3_TreeGrowth.png']);
% 
%     %% ===========================
%     % Figure 4: Shrinking Ball Rad
%     %============================
%     figure('Name','Figure3_TreeGrowthvsTime'); 
%     hold on; grid on;
% 
%     for i = 1:numTrials
%         if ~isempty(timeCurves{i}) && ~isempty(ballRadCurves{i})
%             plot(timeCurves{i}, ballRadCurves{i});
%         end
%     end
% 
%     xlabel('Time (s)');
%     ylabel('Ball Radius');
%     title('Search Ball Radius vs Time');
% 
%     saveas(gcf, [fileloc 'Figure4_BallRad.png']);
% 
%     %% ===========================
%     % Figure 5: Final Paths
%     % ===========================
%     figure('Name','Figure4_FinalPaths','Units','inches','Position',[0 0 10 4]);
% 
%     % --- Position ---
%     subplot(1,2,1);
%     hold on; grid on; axis equal;
%     view(45,25);
%     title('Final Trajectories — Position');
%     xlabel('X'); ylabel('Y'); zlabel('Z');
%     axis(ax_bds(1:6));
% 
%     for i = 1:numTrials
%         if ~isempty(finalPath{i}) && ~isempty(nodePositions{i})
%             pos = nodePositions{i}(:,1:3);
%             plot3(pos(finalPath{i},1), ...
%                   pos(finalPath{i},2), ...
%                   pos(finalPath{i},3), ...
%                   'b','LineWidth',3);
%         end
%     end
% 
%     plot3(startPosition(1), startPosition(2), startPosition(3), ...
%           'og','MarkerSize',10,'LineWidth',2);
%     plot3(goalPosition(1), goalPosition(2), goalPosition(3), ...
%           'xr','MarkerSize',10,'LineWidth',3);
%     hold off;
% 
%     % --- Orientation ---
%     subplot(1,2,2);
%     hold on; grid on; axis equal;
%     view(45,25);
%     title('Final Trajectories — Orientation');
%     xlabel('Roll'); ylabel('Pitch'); zlabel('Yaw');
%     axis(ax_bds(7:12));
% 
%     for i = 1:numTrials
%         if ~isempty(finalPath{i}) && ~isempty(nodePositions{i})
%             rot = rad2deg(nodePositions{i}(:,4:6));
%             plot3(rot(finalPath{i},1), ...
%                   rot(finalPath{i},2), ...
%                   rot(finalPath{i},3), ...
%                   'b','LineWidth',3);
%         end
%     end
% 
%     plot3(rad2deg(startPosition(4)), ...
%           rad2deg(startPosition(5)), ...
%           rad2deg(startPosition(6)), ...
%           'og','MarkerSize',10,'LineWidth',2);
% 
%     plot3(rad2deg(goalPosition(4)), ...
%           rad2deg(goalPosition(5)), ...
%           rad2deg(goalPosition(6)), ...
%           'xr','MarkerSize',10,'LineWidth',3);
% 
%     hold off;
% 
%     saveas(gcf, [fileloc 'Figure5_FinalPaths.png']);
% 
% end
