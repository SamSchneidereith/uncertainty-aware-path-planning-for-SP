function ColorCode(G,colorTrajForPlotting,bestPathTrajForPlotting)
%COLORCODE Sorts and graphs paths based on the error at the end of the path

% Find the number of dimensions
nD = G.numDims;
nD1 = nD+1;
% Set the number of colors wanted:
n = 64;
map = jet(n);
% Find the min and max error:
minVal = min(colorTrajForPlotting(:,nD1));
maxVal = max(colorTrajForPlotting(:,nD1));
% Set up the buckets to sort the paths by:
Buckets = linspace(minVal,maxVal,n);
% Set up the structure to store the paths:
SortedTraj(n).path = {};

% Loop over all the nodes in the graph:
for i = 2:G.n
    j = 1;
    while j < n
        % Find which bucket the error fits into:
        if G.nodes{i,1}.err > Buckets(j)
            j = j + 1;
        else
            break
        end
    end
    % If the error is in the first bucket, put it here (shouldn't be
    % though):
    if j == 1
        % Increase the size of the bucket and store the path in a cell:
        ind = size(SortedTraj(1).path,1) + 1;
        len = size(G.nodes{i,1}.parentTraj,1);
        cel = mat2cell(G.nodes{i,1}.parentTraj,len,nD);
        SortedTraj(1).path(ind,1) = cel;
    else
        % Increase the size of the bucket and store the path in a cell:
        ind = size(SortedTraj(j).path,1) + 1;
        len = size(G.nodes{i,1}.parentTraj,1);
        cel = mat2cell(G.nodes{i,1}.parentTraj,len,nD);
        SortedTraj(j).path(ind,1) = cel;
    end
end

% Plot the trajectories with their corresponding colors:
if nD ~= 2
    fig = figure;
    fig.Units = 'inches';
    fig.Position = [0 0 6 4];
    subplot(1,2,1)
    hold on
    for i = 1:n
        % Plot each trajectory in the position space:
        if size(SortedTraj(i).path,1) > 0
            Paths = [];
            loops = size(SortedTraj(i).path,1);
            for j = 1:loops
                Paths = [Paths; SortedTraj(i).path{j,1}];
                Paths = [Paths; NaN(1,6)];
            end
            plot3(Paths(:,1),Paths(:,2),Paths(:,3),'color',map(i,:),'LineWidth',2);
        end
    end
    % Plot the start and goal node:
    G.plotStartNode('ok', 'MarkerSize', 10, 'LineWidth', 2)
    G.plotGoalNode( 'xk', 'MarkerSize', 10, 'LineWidth', 2)
    hold off
    title('Error Growth throughout Graph')
    %{
c = colorbar;
colormap('jet');
c.Label.String = 'Error from Start Node';
caxis([minVal maxVal]);
ticks = linspace(minVal,maxVal,8);
c.TicksMode = 'manual';
c.Ticks = ticks;
    %}
    xlabel('X Position');
    ylabel('Y Position');
    zlabel('Z Position');
    view(3)
    grid on
    
    subplot(1,2,2)
    hold on
    for i = 1:n
        % Plot each trajectory in the orientation space:
        if size(SortedTraj(i).path,1) > 0
            Paths = [];
            loops = size(SortedTraj(i).path,1);
            for j = 1:loops
                Paths = [Paths; SortedTraj(i).path{j,1}];
                Paths = [Paths; NaN(1,6)];
            end
            plot3(rad2deg(Paths(:,4)),rad2deg(Paths(:,5)),rad2deg(Paths(:,6)),'color',map(i,:),'LineWidth',2);
        end
    end
    G.plotStartNodeRot('ok', 'MarkerSize', 10, 'LineWidth', 2)
    G.plotGoalNodeRot( 'xk', 'MarkerSize', 10, 'LineWidth', 2)
    hold off
    % Add the color bar that shows the errors, make sure to have the limits of
    % the bar adjusted for the min and max error values:
    c = colorbar;
    colormap('jet');
    c.Label.String = 'Total Error from Start Node';
    caxis([minVal maxVal]);
    ticks = linspace(minVal,maxVal,8);
    c.TicksMode = 'manual';
    c.Ticks = ticks;
    title('Error Growth throughout Graph')
    xlabel('X Rotation');
    ylabel('Y Rotation');
    zlabel('Z Rotation');
    view(3)
    grid on
else
    % Do the same thing as above but for the 2D case:
    figure
    hold on
    for i = 1:n
        if size(SortedTraj(i).path,1) > 0
            Paths = [];
            loops = size(SortedTraj(i).path,1);
            for j = 1:loops
                Paths = [Paths; SortedTraj(i).path{j,1}];
                Paths = [Paths; NaN(1,nD)];
            end
            plot(Paths(:,1),Paths(:,2),'color',map(i,:),'LineWidth',2);
        end
    end
    G.plotStartNode('ok', 'MarkerSize', 10, 'LineWidth', 2)
    G.plotGoalNode( 'xk', 'MarkerSize', 10, 'LineWidth', 2)
    plot(bestPathTrajForPlotting(:,1), bestPathTrajForPlotting(:,2), '--k', 'LineWidth', 0.5)
    hold off
    c = colorbar;
    colormap('jet');
    c.Label.String = 'Error from Start Node';
    caxis([minVal maxVal]);
    ticks = linspace(minVal,maxVal,8);
    c.TicksMode = 'manual';
    c.Ticks = ticks;
    %}
    xlabel('X Position');
    ylabel('Y Position');
    title('Error Growth throughout Graph')
    grid on
end
end

