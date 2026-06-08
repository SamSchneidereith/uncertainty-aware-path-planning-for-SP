
nD = G.numDims;
nD1 = nD+1;
nD2 = nD+2;
minVal = min(colorTrajForPlotting(:,nD1));
maxVal = max(colorTrajForPlotting(:,nD1));

% Set the number of colors wanted:
n = 256;
map = jet(n);
% Add white for NaN spaces:
map(end+1,:) = [1 1 1];
Buckets = linspace(minVal,maxVal,n);
% Give each error a value:
colorTrajForPlotting(:,nD2) = NaN;
color = NaN(size(colorTrajForPlotting,1),1);
for i = 1:size(colorTrajForPlotting,1)
    % Check if NaN
    if isnan(colorTrajForPlotting(i,nD1))
        color(i) = n+1;
        continue
    else
        for j = 1:n
            if colorTrajForPlotting(i,nD1) <= Buckets(j)
                color(i) = j;
                break
            end
        end
    end

end
colorTrajForPlotting(:,nD2) =  color(:);



%% Possible speed up:
tstart = tic;
% put all the trajectories in there own arrays, then plot each one
j = 1;
k = 1;
holder = ceil(size(colorTrajForPlotting,1)/G.n);
if nD == 2
for i = 1:size(colorTrajForPlotting,1)
    
    if ~isnan(colorTrajForPlotting(i,1))
        Path(k,:) = colorTrajForPlotting(i,1:2);
        Color = colorTrajForPlotting(i,nD2);
        k = k+1;
        
    else
        k = 1;
        Traj(j).path = Path;
        Traj(j).color = Color;
        j = j+1;
        Path = NaN(holder,2);
    end
   
end
end
%%

fig = figure(2);
%fig.Units = 'normalized';
%fig.Position = [0 0 1 1];
for i = 1:size(Traj,2)
    ColorPlot = plot(Traj(i).path(:,1),Traj(i).path(:,2));
    ColorPlot.Color = map(Traj(i).color,:);
    hold on
    c = colorbar;
    colormap('jet');
    c.Label.String = 'Error';
end
tend1 = toc(tstart);

%%
tstart = tic;
fig = figure(2);
fig.Units = 'normalized';
fig.Position = [0 0 1 1];
for i = 1: size(colorTrajForPlotting,1)-1
   ColorPlot = plot([colorTrajForPlotting(i,1), colorTrajForPlotting(i+1,1)],[colorTrajForPlotting(i,2), colorTrajForPlotting(i+1,2)]);
   hold on
   ColorPlot.Color = map(colorTrajForPlotting(i+1,4),:);
end
G.plotStartNode('ob', 'MarkerSize', 10, 'LineWidth', 3)
G.plotGoalNode( 'xb', 'MarkerSize', 10, 'LineWidth', 3)
hold off
tend2 = toc(tstart);