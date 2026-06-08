classdef ExperimentAnalyzer < handle

properties
    Data
    numTrials
    stats
end

methods

    function obj = ExperimentAnalyzer(Data)
        obj.Data = Data;
        obj.numTrials = numel(Data.Trials);
        obj.stats = obj.extractStats();
    end

    function S = extractStats(obj)

        N = obj.numTrials;

        S.finalCost   = NaN(N,1);
        S.firstTime   = NaN(N,1);
        S.timeCurves  = cell(N,1);
        S.nodeCurves  = cell(N,1);
        S.ballRadCurves = cell(N,1);
        S.costCurves  = cell(N,1);
        S.distCurves  = cell(N,1);
        S.uncertaintyCurves = cell(N,1);
        S.finalPaths  = cell(N,1);

        for i = 1:N

            trial = obj.Data.Trials{i};

            t   = trial.Time;
            nn  = trial.NumNodes;
            cost= trial.Cost;
            dist= trial.PathDist;
            uncertainty = trial.GoalUncertainty;
            br  = trial.BallRad;

            S.timeCurves{i}   = t;
            S.nodeCurves{i}   = nn;
            S.ballRadCurves{i}= br;

            validIdx = find(isfinite(cost));

            if isempty(validIdx)
                continue;
            end

            idxFirst = validIdx(1);
            idxLast  = validIdx(end);

            S.firstTime(i)   = t(idxFirst);
            S.finalCost(i)   = cost(idxLast);
            cost_full = cost;
            dist_full = dist;
            unc_full  = uncertainty;
            
            % Before first valid → Inf
            cost_full(1:idxFirst-1) = Inf;
            dist_full(1:idxFirst-1) = Inf;
            unc_full(1:idxFirst-1)  = Inf;
            
            % After last valid → hold final value (or keep as-is)
            cost_full(idxLast+1:end) = cost(idxLast);
            dist_full(idxLast+1:end) = dist(idxLast);
            unc_full(idxLast+1:end)  = uncertainty(idxLast);
            
            S.costCurves{i}  = cost_full;
            S.distCurves{i}  = dist_full;
            S.uncertaintyCurves{i} = unc_full;

            if idxLast <= numel(trial.Path) && ~isempty(trial.Path{idxLast})
                pathIdx = trial.Path{idxLast};
                S.finalPaths{i} = trial.Tree.nodes(pathIdx);
            end
        end

        fprintf('Successful trials: %d / %d\n', ...
            sum(isfinite(S.finalCost)), N);
    end

    function fig = plotCostVsTime(obj)
        fig = figure('Name','CostVsTime');
        hold on; grid on;

        curves = obj.stats.costCurves;
        time = obj.stats.timeCurves;

        for i = 1:obj.numTrials
            if ~isempty(curves{i}) && ~isempty(time{i})
                plot(time{i}, curves{i});
            end
        end

        [~, mu, maxLen] = obj.buildMeanCurve(curves);
        plot(time{1}, mu, 'k--', 'LineWidth',2);

        xlabel('Time (s)');
        ylabel('Cost');
        % set(gca,'XScale','log');
        title('Cost vs Time');
    end

    function fig = plotCostVsTimeBoxWhisker(obj, numBins)
        if nargin < 2; numBins = 10; end
    
        fig = figure('Name', 'CostVsTime_BoxWhisker');
        hold on; grid on;
    
        t      = obj.stats.timeCurves{1};
        curves = obj.stats.costCurves;
        N      = obj.numTrials;
    
        % Pick evenly spaced time snapshot indices
        snapIdx = round(linspace(1, numel(t), numBins));
        snapTimes = t(snapIdx);
    
        % Build matrix: rows = trials, cols = snapshots
        M = NaN(N, numBins);
        for i = 1:N
            c = curves{i};
            if ~isempty(c)
                M(i, :) = c(snapIdx);
            end
        end
    
        boxplot(M, snapTimes, 'Whisker', 1.5, 'Symbol', 'r+');
    
        xlabel('Time [s]');
        ylabel('Cost');
        title('Cost vs Time');
        % xticklabels(arrayfun(@(x) sprintf('%.0f', x), snapTimes, 'UniformOutput', false));
        % ylim([250 450])
        xticklabels(linspace(0,500, 11));
    end

    function fig = plotDistVsTime(obj)
        fig = figure('Name','DistVsTime');
        hold on; grid on;

        curves = obj.stats.distCurves;
        time = obj.stats.timeCurves;

        for i = 1:obj.numTrials
            if ~isempty(curves{i})
                plot(time{i}, curves{i});
            end
        end

        % [~, mu, maxLen] = obj.buildMeanCurve(curves);
        % plot(1:maxLen, mu, 'k--', 'LineWidth',2);

        xlabel('Time Step');
        ylabel('Distance');
        % set(gca,'XScale','log');
        title('Distance vs Time');
    end

    function fig = plotUncertaintyVsTime(obj)
        fig = figure('Name','UncertaintyVsTime');
        hold on; grid on;

        curves = obj.stats.uncertaintyCurves;
        time = obj.stats.timeCurves;

        for i = 1:obj.numTrials
            if ~isempty(curves{i})
                plot(time{i}, curves{i});
            end
        end

        [~, mu, maxLen] = obj.buildMeanCurve(curves);
        plot(time{1}, mu, 'k--', 'LineWidth',2);

        xlabel('Time Step');
        ylabel('Uncertainty');
        % set(gca,'XScale','log');
        title('Uncertainty vs Time');
    end

    function fig = plotNodesVsTime(obj)
        fig = figure('Name','NodesVsTime');
        hold on; grid on;

        curves = obj.stats.nodeCurves;

        for i = 1:obj.numTrials
            if ~isempty(curves{i})
                plot(curves{i});
            end
        end

        xlabel('Time Step');
        ylabel('Number of Nodes');
        title('Tree Growth vs Time');
    end

    function fig = plotBallRadVsTime(obj)
        fig = figure('Name','BallRadiusVsTime');
        hold on; grid on;

        curves = obj.stats.ballRadCurves;
        time = obj.stats.timeCurves;

        for i = 1:obj.numTrials
            if ~isempty(curves{i})
                plot(time{i}, curves{i});
            end
        end

        xlabel('Time Step');
        ylabel('Ball Radius');
        title('Rewiring Radius vs Time');
    end

    function fig = plotFinalCostHistogram(obj)

        fig = figure('Name','FinalStatistics');

        subplot(1,2,1);
        vals = obj.stats.finalCost(isfinite(obj.stats.finalCost));

        if ~isempty(vals)
            histogram(vals,'Normalization','pdf'); hold on;
            mu = mean(vals);
            sigma = std(vals);
            xfit = linspace(min(vals),max(vals),200);
            plot(xfit, normpdf(xfit,mu,sigma),'r','LineWidth',2);
            title('Final Cost');
        else
            title('No Successful Trials');
        end
        grid on;

        subplot(1,2,2);
        vals = obj.stats.firstTime(isfinite(obj.stats.firstTime));

        if ~isempty(vals)
            histogram(vals,'Normalization','pdf'); hold on;
            mu = mean(vals);
            sigma = std(vals);
            xfit = linspace(min(vals),max(vals),200);
            plot(xfit, normpdf(xfit,mu,sigma),'r','LineWidth',2);
            title('Time to First Path');
        else
            title('No Successful Trials');
        end
        grid on;
    end

    function [M, mu, maxLen] = buildMeanCurve(~, curves)

        N = numel(curves);
        maxLen = max(cellfun(@length, curves));

        M = NaN(N, maxLen);

        for i = 1:N
            c = curves{i};
            if ~isempty(c)
                M(i,1:length(c)) = c(:);
            end
        end

        mu = mean(M,1,'omitnan');
    end
end
end