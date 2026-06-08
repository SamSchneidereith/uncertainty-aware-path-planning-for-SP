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


    % Copyright 2019 Michael Otte, University of Maryland
    
    % impliments a graph. 
    
    properties
        n                   % total number of nodes in the graph
       
        numDims             % number of dimensions in the C-Space
        
        nodes               % an n length cell array of node structures

        startInd            % index (ID) of start node in nodePositions
        goalInd             % index (ID) of goal node in nodePositions
  
        startNode           % node handel to start node
        goalNode            % node handel to goal node
    
        motionPath          % an 1 X l array of indicies
        
        usingPlotData       % set to true to store things that facilitate plotting
        
        
        inListForPlotting   % slightly messy, this is redundant, but checking if we're 
                            % plotting each time a node  has their list flag change will 
                            % take a lot of time, we we will need to track this in parallel
                            % when we do plot, the existance of this array makes things run 
                            % much more quickly
                            
        % stuff used to help Matlab plot more quickly
        % note these are only used if usingPlotData = true
        nodePositionsForPlotting  % n by 2 array (only used for plotting)
        
        edgesForPlotting           % used for plotting all edges as a single curve
    
        parentEdgesForPlotting     % used for plotting parent edges as a single curve
        
        % stuff sampling based motion planning
        delta                      % RRT delta parameter
        maxNodesAllowed            % algorithm ends after the graph contains this many nodes
                                   % even if the goal has not been found.
                                   
        dimensionMins              % min coordinate along each dimension
        dimensionMaxs              % max coordinate along each dimension
           
        ballConstant               % used for RRT* and other algorithms that use a shrinking ball radius etc.
    end
    methods
        function obj = motionGraph()
            % constructor, returns an empty motion graph
            obj.n = 0;
            
            obj.numDims = 0;
            
            obj.nodePositionsForPlotting = [];
            
            obj.nodes = cell(0,1);
           
            obj.startInd = -1;
            obj.goalInd = -2;
          
            obj.startNode = [];
            obj.goalNode = [];
            
            
            obj.motionPath = [];
            
            obj.usingPlotData = false;
            
            obj.inListForPlotting  = zeros(0,1); 
            
            obj.delta = Inf;
            obj.maxNodesAllowed = Inf;
            obj.dimensionMins = [];
            obj.dimensionMaxs = [];
            obj.ballConstant = Inf;
        end
        
        function obj = populateGraphNodes(obj,nodePositionsRaw, startIndex, goalIndex)
            % numElements is the maximum number of elements that
            % will ever be inserted
            obj.n = size(nodePositionsRaw,1);
            
            obj.numDims = size(nodePositionsRaw,2);
            
            obj.nodes = cell(obj.n,1);
            for i = 1:obj.n
                obj.nodes{i} = ssspGraphNode(nodePositionsRaw(i,:), i);        
            end
            
            obj.startInd = startIndex;
            obj.goalInd = goalIndex;
          
            obj.startNode = obj.nodes{startIndex};
            obj.goalNode = obj.nodes{goalIndex};

            % slightly messy, this is redundant, but checking if we're 
            % plotting each time a node  has their list flag change will 
            % take a lot of time, we we will need to track this in parallel
            obj.inListForPlotting  = zeros(obj.n,1);
        end
        
        function makeNeighborhoodGraph(obj, neighborhoodRadius)
            % adds edges to the graph such that each node is attached
            % to all nodes within neighborhoodRadius of itself
            
            % calculations edge weights
            for i = 1:obj.n
                if mod(i,100) == 0
                   disp(['building graph neighborhoods ' num2str(i) ' of ' num2str(obj.n) ' complete'])
                end
                
                for j = 1:obj.n
                    if i == j 
                        continue 
                    end 
                    dist = sqrt(sum((obj.nodes{i}.position - obj.nodes{j}.position).^2,2));
                    if dist < neighborhoodRadius
                        obj.nodes{i}.neighbors{length(obj.nodes{i}.neighbors) +1} = obj.nodes{j};
                        obj.nodes{i}.costsToNeighbors = [obj.nodes{i}.costsToNeighbors dist];  
                    end
                end
            end    
        end
        

        function populateForSBSearch(obj, delta, dimensionMins, dimensionMaxs, ballConstant, startPose, goalPose, maxNodes, numDims)
            % sets up the graph for a sampling based search (RRT, etc)
             
            obj.numDims = numDims;
            obj.dimensionMins = dimensionMins;
            obj.dimensionMaxs = dimensionMaxs;
            obj.nodes = cell(maxNodes,1);
           
            obj.startInd = 1;
            obj.startNode = ssspGraphNode(startPose, obj.startInd);
            obj.nodes{obj.startInd} = obj.startNode;
            
            obj.goalInd = 2;
            obj.goalNode = ssspGraphNode(goalPose, obj.goalInd);
            obj.nodes{obj.goalInd} = obj.goalNode;
            obj.delta = delta;     
            obj.ballConstant = ballConstant;             
            obj.n = 2;
            obj.maxNodesAllowed = maxNodes;
        end
        
        function ret = hyperBallRad(obj)
            % calculates the current hyperball radius for RRT like algorithms 
        
            ret =  min(obj.delta, obj.ballConstant*((log(1+obj.n)/(obj.n))^(1/obj.numDims)));
        end
        
        
        
        function ret = insertNewNodeForSBSearch(obj, position)
           % creates a new node for the position and then inserts in the
           % graph, returns a handle to the node
           
           obj.n = obj.n + 1;
           obj.nodes{obj.n} = ssspGraphNode(position, obj.n);
           ret = obj.nodes{obj.n};
        end
        
        function ret = randomPoint(obj)
            % returns a random point from withing the min and max bounds
            ret = obj.dimensionMins + rand(1, obj.numDims) .* (obj.dimensionMaxs - obj.dimensionMins);
        end
            
        function saveToFile(obj, filename)
            % saves the graph to a text file
            
            fileID = fopen(filename, 'w');
            fprintf(fileID,'%d\n',obj.n);
            fprintf(fileID,'%d\n',obj.numDims);
            for i = 1:obj.n
                obj.nodes{i}.saveToFileID(fileID)
            end
            fprintf(fileID,'%d\n',obj.startInd);
            fprintf(fileID,'%d\n',obj.goalInd);
            
            fclose(fileID);
        end
        
        function loadFromFile(obj, filename)
            % loads the graph from the text file
            
            fileID = fopen(filename, 'r');
            obj.n = fscanf(fileID,'%d\n',[1 1]);
            obj.numDims = fscanf(fileID,'%d\n',[1 1]);
            
            obj.nodes = cell(obj.n,1);
            neighborInds = cell(obj.n,1);    % each cell stores the inds of the neighbors
            for i = 1:obj.n
                
                obj.nodes{i} = ssspGraphNode([], i);
                neighborInds{i} = obj.nodes{i}.loadFromFileID(fileID);
            end
            
            % now that we have all the nodes, we can link up each node to its
            % neighbors
            for i = 1:obj.n
                for j = 1:length(neighborInds{i})
                    obj.nodes{i}.neighbors{j,1} = obj.nodes{neighborInds{i}(j)};
                end
            end
            
            obj.startInd = fscanf(fileID,'%d\n', [1 1]);
            obj.goalInd = fscanf(fileID,'%d\n', [1 1]);
            
            obj.startNode = obj.nodes{obj.startInd};
            obj.goalNode = obj.nodes{obj.goalInd};
            
            % slightly messy, this is redundant, but checking if we're 
            % plotting each time a node has their list flag change will 
            % take a lot of time, we we will need to track this in parallel
            obj.inListForPlotting  = zeros(obj.n,1);
            
            fclose(fileID);
        end
        
        function extractMotionPath(obj)
            % assuming that a search has been run that sets up parent
            % pointers, this extracts the path (indixies) from start to goal
        
            thisNode = obj.goalNode;
            pathTemp = [thisNode.id];
            while thisNode.parentID ~= 0
                thisNode = thisNode.parentNode;
                pathTemp = [pathTemp thisNode.id];
            end    
            obj.motionPath = pathTemp; 
        end
        
        function calculatePlottingData(obj)
            % used to set up data (nodes and edges) for plotting, 
            % assumes all nodes and edges are already defined
            
            obj.usingPlotData = true;
            
            obj.nodePositionsForPlotting = zeros(obj.n, obj.numDims);
            for i = 1:obj.n
                obj.nodePositionsForPlotting(i,:) = obj.nodes{i}.position(:);
            end  

            
            edgesForPlottingTemp = nan(obj.n*obj.n*3*2,obj.numDims);
            k = 1;
            for i = 1:obj.n
                thisNode = obj.nodes{i};
                for j = 1:length(thisNode.neighbors)
                    neighborNode = thisNode.neighbors{j};
                    if thisNode.id < neighborNode.id
                        edgesForPlottingTemp(k,:) = thisNode.position(:);
                        edgesForPlottingTemp(k+1,:) = neighborNode.position(:);
                        k = k+3;
                    end
                end
            end
            obj.edgesForPlotting = edgesForPlottingTemp(1:k,:);
                   
        end
        
        function calculateParentEdgesForPlotting(obj)
            % goes through the graph and sets up parentEdgesForPlotting
            % to reflect the current state of the graph
            
            parentEdgesForPlottingTemp = nan(2*obj.n*3,obj.numDims);  % this helps matlab to plot faster later
            k = 1;
            for i = 1:obj.n
                thisNode = obj.nodes{i};
            
                if thisNode.parentID ~= 0
                    parentEdgesForPlottingTemp(k,:) = thisNode.position(:);
                    parentEdgesForPlottingTemp(k+1,:) = thisNode.parentNode.position(:);
                
                     k = k + 3;
                end
            end
            obj.parentEdgesForPlotting = parentEdgesForPlottingTemp(1:k, :);
        end
        
        
        function ret = getParentTrajectoriesForPlotting(obj, numSubDim)
            % this returns a matrix containing all parent trajectories,
            % seperated by nans so that we can plot the trajectories of the
            % shortest path tree.
            % Note: we only retung the first numSubDim dimensions of the
            % trajectories (since they may contain more dimensions that the
            % C-space)
             
            % first pass, figure out how big of a storage matrix we need
            k = 1;
            for i = 1:obj.n
                k = k + size(obj.nodes{i}.parentTraj, 1) + 1;
            end
            
            % allocate storage space
            ret = nan(k, numSubDim);
            
            % second pass, populate the matrix
            k = 1;
            for i = 2:obj.n
                if isempty(obj.nodes{i}.parentTraj)
                    continue
                end
                thisLen = size(obj.nodes{i}.parentTraj, 1);
                ret(k:k+thisLen-1, :) = obj.nodes{i}.parentTraj(:,1:numSubDim);
                k = k + size(obj.nodes{i}.parentTraj, 1) + 1;
            end
            
        end
                function ret = getColorParentTrajectoriesForPlotting(obj, numSubDim)
            % this returns a matrix containing all parent trajectories,
            % seperated by nans so that we can plot the trajectories of the
            % shortest path tree.
            % Note: we only retung the first numSubDim dimensions of the
            % trajectories (since they may contain more dimensions that the
            % C-space)
             
            % first pass, figure out how big of a storage matrix we need
            k = 1;
            for i = 1:obj.n
                k = k + size(obj.nodes{i}.parentTraj, 1) + 1;
            end
            
            % allocate storage space
            ret = nan(k, numSubDim);
            
            % second pass, populate the matrix
            k = 1;
            for i = 2:obj.n
                if isempty(obj.nodes{i}.parentTraj)
                    continue
                end
                thisLen = size(obj.nodes{i}.parentTraj, 1);
                ret(k:k+thisLen-1, 1:numSubDim-1) = obj.nodes{i}.parentTraj(:,1:numSubDim-1);
                ret(k:k+thisLen-1, numSubDim) = obj.nodes{i}.parentNode.err;
                k = k + size(obj.nodes{i}.parentTraj, 1) + 1;
            end
            
        end
        
        
        function ret = getBestPathTrajectoriesForPlotting(obj, numSubDim)
            % this returns a matrix containing just the parent trajectories,
            % along the best path 
            % Note: we only retung the first numSubDim dimensions of the
            % trajectories (since they may contain more dimensions that the
            % C-space)
             
            % first pass, figure out how big of a storage matrix we need
            k = 1;
            thisNode = obj.goalNode;
            while thisNode.parentID ~= 0
                k = k + size(thisNode.parentTraj, 1);
                thisNode = thisNode.parentNode;
            end
            
            % allocate storage space
            ret = nan(k, numSubDim);
            
            % second pass, populate the matrix
            k = 1;
            thisNode = obj.goalNode;
            while thisNode.parentID ~= 0
                thisLen = size(thisNode.parentTraj, 1);
                ret(k:k+thisLen-1, :) = thisNode.parentTraj(end:-1:1,1:numSubDim);
                k = k + thisLen;
                thisNode = thisNode.parentNode;
            end
        end
        
        function plotEdges(obj, varargin)
            % plots the edges, call this the same way as plot (multiple
            % arguments accepted)
            if obj.numDims == 2
                plot(obj.edgesForPlotting(:,1), obj.edgesForPlotting(:,2),varargin{:}) 
            else
                plot3(obj.edgesForPlotting(:,1), obj.edgesForPlotting(:,2), obj.edgesForPlotting(:,3),varargin{:})  
            end
        end
        
        function plotRotEdges(obj, varargin)
            % plots the edges, call this the same way as plot (multiple
            % arguments accepted)

            plot3(rad2deg(obj.edgesForPlotting(:,4)), rad2deg(obj.edgesForPlotting(:,5)), rad2deg(obj.edgesForPlotting(:,6)),varargin{:})  
        end 
        
        function plotNodes(obj, varargin)
            % plots only the nodes
            % call this the same way as plot (multiple arguments accepted)
            if obj.numDims == 2
                plot(obj.nodePositionsForPlotting(:,1), obj.nodePositionsForPlotting(:,2), varargin{:})
            else
                plot3(obj.nodePositionsForPlotting(:,1), obj.nodePositionsForPlotting(:,2), obj.nodePositionsForPlotting(:,3), varargin{:})
            end
        end 
        
        function plotRotNodes(obj, varargin)
            % plots only the nodes
            % call this the same way as plot (multiple arguments accepted)
            
            plot3(rad2deg(obj.nodePositionsForPlotting(:,4)), rad2deg(obj.nodePositionsForPlotting(:,5)), rad2deg(obj.nodePositionsForPlotting(:,6)), varargin{:})
        end
        
        function plotParentEdges(obj, varargin)
            % plots the edges, call this the same way as plot (multiple
            % arguments accepted)
            if obj.numDims == 2
                plot(obj.parentEdgesForPlotting(:,1),obj.parentEdgesForPlotting(:,2), varargin{:})
            else
                plot3(obj.parentEdgesForPlotting(:,1),obj.parentEdgesForPlotting(:,2), obj.parentEdgesForPlotting(:,3), varargin{:})
            end
        end
        
        function plotParentRotEdges(obj, varargin)
            % plots the edges, call this the same way as plot (multiple
            % arguments accepted)
                plot3(rad2deg(obj.parentEdgesForPlotting(:,4)),rad2deg(obj.parentEdgesForPlotting(:,5)), rad2deg(obj.parentEdgesForPlotting(:,6)), varargin{:})
        end   
        
        function plotNodesWithInListVal(obj, val, varargin)
            % plots only the nodes that have inList (inListForPlotting)
            % flag set to val (useful for seeing what is going on).
            % call this the same way as plot (multiple arguments accepted)
            if obj.numDims == 2
                plot(obj.nodePositionsForPlotting(obj.inListForPlotting == val,1), obj.nodePositionsForPlotting(obj.inListForPlotting == val,2),varargin{:})
            else
                plot3(obj.nodePositionsForPlotting(obj.inListForPlotting == val,1), obj.nodePositionsForPlotting(obj.inListForPlotting == val,2), obj.nodePositionsForPlotting(obj.inListForPlotting == val,3),varargin{:})
            end
        end 
        
        function plotNodesWithInListValRot(obj, val, varargin)
            % plots only the nodes that have inList (inListForPlotting)
            % flag set to val (useful for seeing what is going on).
            % call this the same way as plot (multiple arguments accepted)
            
            plot3(rad2deg(obj.nodePositionsForPlotting(obj.inListForPlotting == val,4)), rad2deg(obj.nodePositionsForPlotting(obj.inListForPlotting == val,5)), rad2deg(obj.nodePositionsForPlotting(obj.inListForPlotting == val,6)),varargin{:})
        end 
          
        function plotStartNode(obj, varargin)
            % plot the start node
            % call this the same way as plot (multiple arguments accepted)
            if obj.numDims == 2
                plot(obj.nodePositionsForPlotting(obj.startInd,1), obj.nodePositionsForPlotting(obj.startInd,2),varargin{:})   
            else
                plot3(obj.nodePositionsForPlotting(obj.startInd,1), obj.nodePositionsForPlotting(obj.startInd,2), obj.nodePositionsForPlotting(obj.startInd,3),varargin{:})    
            end
        end
        
        function plotStartNodeRot(obj, varargin)
            % plot the start node
            % call this the same way as plot (multiple arguments accepted)
                      
            plot3(rad2deg(obj.nodePositionsForPlotting(obj.startInd,4)), rad2deg(obj.nodePositionsForPlotting(obj.startInd,5)), rad2deg(obj.nodePositionsForPlotting(obj.startInd,6)),varargin{:})    
        end    

        function plotStartNode3DPosition(obj, varargin)
            % plot the start node
            % call this the same way as plot (multiple arguments accepted)
                      
            plot3(obj.startNode.position(1), obj.startNode.position(2), obj.startNode.position(3),varargin{:})   
        end   
        
        function plotStartNode2DPosition(obj, varargin)
            % plot the start node
            % call this the same way as plot (multiple arguments accepted)
            
            plot(obj.startNode.position(1), obj.startNode.position(2),varargin{:})
        end
        
        function plotStartNode3DOrientation(obj, varargin)
            % plot the start node
            % call this the same way as plot (multiple arguments accepted)
            
            plot3(rad2deg(obj.startNode.position(4)), rad2deg(obj.startNode.position(5)), rad2deg(obj.startNode.position(6)),varargin{:})
        end
        
        function plotGoalNode(obj, varargin) 
            % plot the goal node
            % call this the same way as plot (multiple arguments accepted)
            if obj.numDims == 2
                plot(obj.nodePositionsForPlotting(obj.goalInd,1), obj.nodePositionsForPlotting(obj.goalInd,2), varargin{:})
            else
                plot3(obj.nodePositionsForPlotting(obj.goalInd,1), obj.nodePositionsForPlotting(obj.goalInd,2), obj.nodePositionsForPlotting(obj.goalInd,3), varargin{:})
            end
        end
        
        function plotGoalNodeRot(obj, varargin)
            % plot the goal node
            % call this the same way as plot (multiple arguments accepted)
            
            plot3(rad2deg(obj.nodePositionsForPlotting(obj.goalInd,4)), rad2deg(obj.nodePositionsForPlotting(obj.goalInd,5)), rad2deg(obj.nodePositionsForPlotting(obj.goalInd,6)), varargin{:})
        end
        
        function plotGoalNode3DPosition(obj, varargin) 
            % plot the goal node
            % call this the same way as plot (multiple arguments accepted)
            
            plot3(obj.goalNode.position(1), obj.goalNode.position(2), obj.goalNode.position(3),varargin{:})
        end
        
        function plotGoalNode2DPosition(obj, varargin)
            % plot the goal node
            % call this the same way as plot (multiple arguments accepted)
            
            plot(obj.goalNode.position(1), obj.goalNode.position(2),varargin{:})
        end
        function plotGoalNode3DOrientation(obj, varargin)
            % plot the goal node
            % call this the same way as plot (multiple arguments accepted)
            
            plot3(rad2deg(obj.goalNode.position(4)),rad2deg(obj.goalNode.position(5)), rad2deg(obj.goalNode.position(6)),varargin{:})
        end
        
        function plotMotionPath(obj, varargin) 
            % plot the path
            % call this the same way as plot (multiple arguments accepted)
            
            pathX = zeros(1,length(obj.motionPath));
            pathY = zeros(1,length(obj.motionPath));
            
            for p = 1:length(obj.motionPath)
                pathX(p) = obj.nodePositionsForPlotting(obj.motionPath(p),1);
                pathY(p) = obj.nodePositionsForPlotting(obj.motionPath(p),2);
            end
            if obj.numDims > 2
                pathZ = zeros(1,length(obj.motionPath));
                for p = 1:length(obj.motionPath)
                    pathZ(p) = obj.nodePositionsForPlotting(obj.motionPath(p),3);
                end
                plot3(pathX , pathY, pathZ, varargin{:})
            else
                plot(pathX, pathY, varargin{:})
            end
            
             
        end
        
        function plotRotMotionPath(obj, varargin)
            % plot the path
            % call this the same way as plot (multiple arguments accepted)
            
            pathX = zeros(1,length(obj.motionPath));
            pathY = zeros(1,length(obj.motionPath));
            pathZ = zeros(1,length(obj.motionPath));
            for p = 1:length(obj.motionPath)
                pathX(p) = rad2deg(obj.nodePositionsForPlotting(obj.motionPath(p),4));
                pathY(p) = rad2deg(obj.nodePositionsForPlotting(obj.motionPath(p),5));
                pathZ(p) = rad2deg(obj.nodePositionsForPlotting(obj.motionPath(p),6));
            end
            
            plot3(pathX , pathY, pathZ, varargin{:}) 
        end
    end
end

