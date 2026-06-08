classdef motionGraph < handle

% The MIT License (MIT)
%
% Copyright June, 2019 Michael Otte, Universtiy of Maryland
%
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, including without limitation the rights
% to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
% copies of the Software, and to permit persons to whom the Software is
% furnished to do so, subject to the following conditions:
%
% The above copyright notice and this permission notice shall be included in
% all copies or substantial portions of the Software.
% 
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
% THE SOFTWARE.

    % Modified by Sam Schneidereith 2026
    properties
        numDims              % number of dimensions in the C-Space
        n                    % total number of nodes in the graph
        nodes                % an n length cell array of node structures
  
        startNode            % node handel to start node
        goalNode             % node handel to goal node
    
        motionPath           % an 1 X l array of indicies (might change to nodes)
                                   
        dimensionMins        % min coordinate along each dimension
        dimensionMaxs        % max coordinate along each dimension  
    end
    methods
        function obj = motionGraph(numDims, dimMins, dimMaxs, maxNodes)
            % constructor, returns an empty motion graph
            obj.numDims = numDims;
            obj.n = 0;
            obj.nodes = cell(maxNodes,1);   % Initializes nodes array, preallocates space for speed
           
            obj.startNode = [];
            obj.goalNode = [];
            
            obj.motionPath = [];

            obj.dimensionMins = dimMins;
            obj.dimensionMaxs = dimMaxs;
        end
        
        function obj = populateGraphNodes(obj, start, goal) % maxNodes,  delta, ballConst,          
            obj.startNode = start; 
            obj.insertNode(start);
            obj.goalNode = goal; 
            obj.insertNode(goal)

            obj.startNode.g = 0;  obj.startNode.lmc = 0;
            obj.goalNode.g = inf; obj.goalNode.lmc = inf;
            obj.startNode.inHeap = false; obj.goalNode.inHeap = false;
            obj.startNode.heapIndex = -1;  obj.goalNode.heapIndex = -1;
            obj.startNode.successors = []; obj.goalNode.successors = [];
        end
        
        function ret = hyperBallRad(obj, delta, ballConstant)
            % Hyperball radius for RRT algorithms 
            ret =  min(delta, ballConstant*((log(1+obj.n)/(obj.n))^(1/obj.numDims)));
        end
        
        function insertNode(obj, node)
           % Inserts node into the graph
           
           obj.n = obj.n + 1;
           obj.nodes{obj.n} = node;
           node.id = obj.n;
        end
        
        function ret = randomPoint(obj, goalBias)
            % Returns a random point from withing the min and max

            if nargin > 1 && rand() < goalBias % Samples goal if goalBias passed in
                ret = obj.goalNode.position;
                return
            end
            ret = obj.dimensionMins + rand(1, obj.numDims) .* (obj.dimensionMaxs - obj.dimensionMins);
        end
        
        function ret = extractMotionPath(obj)
            % assuming that a search has been run that sets up parent
            % pointers, this extracts the path (indixies) from start to goal
            
            path = [];
            if (~isempty(obj.goalNode.parent))
                node = obj.goalNode;
                while ~isempty(node)
                    path = [node.id path];
                    node = node.parent;
                end
                obj.motionPath = path;
                ret = path;
            else
                ret = NaN;
            end
        end
        
        % Plotting Functions (Designed only for use with 6DoF)
        function fig = plotTree(obj)
            [edges, ~] = obj.computeEdges(obj.nodes, obj.numDims);
            
            % Unpack Edges
            X = squeeze(edges(1,:,:));    Y = squeeze(edges(2,:,:));     Z = squeeze(edges(3,:,:));
            Roll = squeeze(edges(4,:,:)); Pitch = squeeze(edges(5,:,:)); Yaw = squeeze(edges(6,:,:));
            
            start = obj.startNode.position;
            goal = obj.goalNode.position;
            axbds = reshape([obj.dimensionMins(:) obj.dimensionMaxs(:)].', 1, []);

            fig = figure('Name','FullTree','Units','inches','Position',[0 0 10 4]);
            movegui(fig,'center');
            subplot(1,2,1) % Position
            hold on; grid on; axis equal; view(3);
            title('Tree Overview — Position')
            xlabel('X'); ylabel('Y'); zlabel('Z');
            axis(axbds(1:6))  
            plot3(X, Y, Z, 'k-');   %Edges
            plot3(start(1), start(2), start(3), 'rx', MarkerSize = 10, LineWidth = 2);  %Start
            plot3(goal(1), goal(2), goal(3), 'go', MarkerSize = 10, LineWidth = 2);     %Goal

            subplot(1,2,2) % Orientation
            hold on; grid on; axis equal; view(3);
            title('Tree Overview — Orientation')
            xlabel('Roll'); ylabel('Pitch'); zlabel('Yaw');
            axis(axbds(7:12))  
            plot3(Roll, Pitch, Yaw, 'k-');   %Edges
            plot3(start(4), start(5), start(6), 'rx', MarkerSize = 10, LineWidth = 2);  %Start
            plot3(goal(4), goal(5), goal(6), 'go', MarkerSize = 10, LineWidth = 2);     %Goal
        end

        function fig = plotPathTree(obj)
            [edges, ~] = obj.computeEdges(obj.nodes, obj.numDims);
            pathEdges = obj.computeEdges(obj.nodes(obj.motionPath), obj.numDims);

            % Unpack Edges
            X = squeeze(edges(1,:,:));    Y = squeeze(edges(2,:,:));     Z = squeeze(edges(3,:,:));
            Roll = squeeze(edges(4,:,:)); Pitch = squeeze(edges(5,:,:)); Yaw = squeeze(edges(6,:,:));
            pathX = squeeze(pathEdges(1,:,:));    pathY = squeeze(pathEdges(2,:,:));     pathZ = squeeze(pathEdges(3,:,:));
            pathRoll = squeeze(pathEdges(4,:,:)); pathPitch = squeeze(pathEdges(5,:,:)); pathYaw = squeeze(pathEdges(6,:,:));

            start = obj.startNode.position;
            goal = obj.goalNode.position;
            axbds = reshape([obj.dimensionMins(:) obj.dimensionMaxs(:)].', 1, []);

            fig = figure('Name','PathFullTree','Units','inches','Position',[0 0 10 4]);
            movegui(fig,'center');
            subplot(1,2,1) % Position
            hold on; grid on; axis equal; view(3);
            title('Tree Overview — Position')
            xlabel('X'); ylabel('Y'); zlabel('Z');
            axis(axbds(1:6))  
            plot3(X, Y, Z, 'Color',[0.7 0.7 0.7]);   %Edges
            plot3(pathX, pathY, pathZ, 'b-', LineWidth = 2);    %Path
            plot3(start(1), start(2), start(3), 'rx', MarkerSize = 10, LineWidth = 2);  %Start
            plot3(goal(1), goal(2), goal(3), 'go', MarkerSize = 10, LineWidth = 2);     %Goal

            subplot(1,2,2) % Orientation
            hold on; grid on; axis equal; view(3);
            title('Tree Overview — Orientation')
            xlabel('Roll'); ylabel('Pitch'); zlabel('Yaw');
            axis(axbds(7:12))  
            plot3(Roll, Pitch, Yaw, 'Color',[0.7 0.7 0.7]);   %Edges
            plot3(pathRoll, pathPitch, pathYaw, 'b-', LineWidth = 2); %Path
            plot3(start(4), start(5), start(6), 'rx', MarkerSize = 10, LineWidth = 2);  %Start
            plot3(goal(4), goal(5), goal(6), 'go', MarkerSize = 10, LineWidth = 2);     %Goal
        end

        function fig = plotPathNoTree(obj)
            pathEdges = obj.computeEdges(obj.nodes(obj.motionPath), obj.numDims);

            % Unpack Edges
            pathX = squeeze(pathEdges(1,:,:));    pathY = squeeze(pathEdges(2,:,:));     pathZ = squeeze(pathEdges(3,:,:));
            pathRoll = squeeze(pathEdges(4,:,:)); pathPitch = squeeze(pathEdges(5,:,:)); pathYaw = squeeze(pathEdges(6,:,:));

            start = obj.startNode.position;
            goal = obj.goalNode.position;
            axbds = reshape([obj.dimensionMins(:) obj.dimensionMaxs(:)].', 1, []);

            fig = figure('Name','PathNoTree','Units','inches','Position',[0 0 10 4]);
            movegui(fig,'center');
            subplot(1,2,1) % Position
            hold on; grid on; axis equal; view(3);
            title('Tree Overview — Position')
            xlabel('X'); ylabel('Y'); zlabel('Z');
            axis(axbds(1:6))  
            plot3(pathX, pathY, pathZ, 'b-', LineWidth = 2);    %Path
            plot3(start(1), start(2), start(3), 'rx', MarkerSize = 10, LineWidth = 2);  %Start
            plot3(goal(1), goal(2), goal(3), 'go', MarkerSize = 10, LineWidth = 2);     %Goal

            subplot(1,2,2) % Orientation
            hold on; grid on; axis equal; view(3);
            title('Tree Overview — Orientation')
            xlabel('Roll'); ylabel('Pitch'); zlabel('Yaw');
            axis(axbds(7:12))  
            plot3(pathRoll, pathPitch, pathYaw, 'b-', LineWidth = 2); %Path
            plot3(start(4), start(5), start(6), 'rx', MarkerSize = 10, LineWidth = 2);  %Start
            plot3(goal(4), goal(5), goal(6), 'go', MarkerSize = 10, LineWidth = 2);     %Goal
        end

        function fig = plotCostTree(obj)
            [edges, edgeCosts] = obj.computeEdges(obj.nodes, obj.numDims);
        
            % Unpack Edges
            X = squeeze(edges(1,:,:));    Y = squeeze(edges(2,:,:));     Z = squeeze(edges(3,:,:));
            Roll = squeeze(edges(4,:,:)); Pitch = squeeze(edges(5,:,:)); Yaw = squeeze(edges(6,:,:));
        
            % Colormap scaling
            finiteMask = isfinite(edgeCosts);
            finiteCosts = edgeCosts(finiteMask);
            gmin = min(finiteCosts); gmax = max(finiteCosts);
            gnorm = zeros(size(edgeCosts));
            gnorm(finiteMask) = (edgeCosts(finiteMask) - gmin) / (gmax - gmin + eps);
            % gmin = min(edgeCosts); gmax = max(edgeCosts);
            % gnorm = (edgeCosts - gmin) / (gmax - gmin + eps);
            cmap = turbo(256);
        
            start = obj.startNode.position;
            goal  = obj.goalNode.position;
            axbds = reshape([obj.dimensionMins(:) obj.dimensionMaxs(:)].', 1, []);
        
            fig = figure('Name','CostTree','Units','inches','Position',[0 0 10 4]);
            movegui(fig,'center');

            subplot(1,2,1) % Position
            hold on; grid on; axis equal; view(3);
            title('Tree Edge Cost — Position')
            xlabel('X'); ylabel('Y'); zlabel('Z');
            axis(axbds(1:6))
        
            for i = 1:size(X,2)
                cind = max(1, round(gnorm(i)*255));
                plot3(X(:,i), Y(:,i), Z(:,i), ...
                      'Color', cmap(cind,:), ...
                      'LineWidth', 1.5);
            end
            plot3(start(1), start(2), start(3), 'rx', 'MarkerSize',10,'LineWidth',2);
            plot3(goal(1),  goal(2),  goal(3),  'go', 'MarkerSize',10,'LineWidth',2);
        
            colormap(gca, cmap);
            clim([gmin gmax]);
            colorbar;
            ylabel(colorbar, 'Child Node Cost (g)');
        
            subplot(1,2,2) % Orientation
            hold on; grid on; axis equal; view(3);
            title('Tree Edge Cost — Orientation')
            xlabel('Roll'); ylabel('Pitch'); zlabel('Yaw');
            axis(axbds(7:12))
        
            for i = 1:size(Roll,2)
                cind = max(1, round(gnorm(i)*255));
                plot3(Roll(:,i), Pitch(:,i), Yaw(:,i), ...
                      'Color', cmap(cind,:), ...
                      'LineWidth', 1.5);
            end
            plot3(start(4), start(5), start(6), 'rx', 'MarkerSize',10,'LineWidth',2);
            plot3(goal(4),  goal(5),  goal(6),  'go', 'MarkerSize',10,'LineWidth',2);
        
            colormap(gca, cmap);
            clim([gmin gmax]);
            colorbar;
            ylabel(colorbar, 'Child Node Cost (g)');
        end

        function [edges, edgeCosts] = computeEdges(obj, nodes, numDims)
        
            numNodes = length(nodes);
            edges = nan(numDims, 2, numNodes);
            edgeCosts = nan(1, numNodes);
 
            idx = 0;
            for i = 1:numNodes
                if ~isempty(nodes{i})
                    parent = nodes{i}.parent;
        
                    if ~isempty(parent)
                        idx = idx + 1;
        
                        n1 = nodes{i}.position(:);   % child
                        n2 = parent.position(:);     % parent
        
                        edges(:,1,idx) = n1;
                        edges(:,2,idx) = n2;
                        edgeCosts(idx) = nodes{i}.g;
                    end
                end
            end
        
            edges = edges(:,:,1:idx);
            edgeCosts = edgeCosts(1:idx);
        end
    end
end

