classdef ssspGraphNode < handle

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
    
    % impliments a SSSP Graph Node
        
    properties
        id               % the unique id of this node
        position         % the node's position in the C-space (i.e., its state)
        
        neighbors        % an array of node handles (to sucessor neighbors, if that is considered)
        errToNeighbors   % an array of costs to the neighbors
        PMatToNeighbors  % an array of Error Cov matrices to the neighbors
        
        costToStart      % through the graph
        inList           % status: in the open list, etc.
        
        parentID         % id of this node's parent
        parentNode       % node handle of this node's parent
        
              
        % the following are used for sorting in the binary heap:
        
        heapIndex        % (necessary to enable internal removes)
        inHeap           % (true if in heap, otherwise false)
        
        % the following are used more for sampling based motion planning
        PMat                       % Error Covariance matrix 
        lmc                        % local minimum cost
        err                   % Trace of the error covariance matrix
        parentTraj                 % trajectory from parent
        predecessors               % predicessor neighbors 
        errFromPredecessors      % an array of costs of Pos to the neighbors
        PMatFromPredecessors
        trajectoriesFromPredecessors   % an array of trajectories (arrays)
        trajectoriesToNeighbors        % an array of trajectories (arrays)
        
    end
    methods
        function obj = ssspGraphNode(positionState, idNum)
            obj.id = idNum;
            obj.position = positionState;
            obj.neighbors = cell(0,1);          % gets populated later
            obj.errToNeighbors = [];        % gets populated later
            obj.PMatToNeighbors = cell(0,1);        
            
            obj.costToStart = inf;
            obj.inList = 0;
            obj.parentID = 0;
            % obj.parentNode;                 % gets populated later
            numDims = max(size(positionState)); % Number of dimensions of the position
          
            obj.inHeap = false;
            obj.heapIndex = -1;
          
            obj.lmc = inf;
            obj.PMat = 1*eye(numDims);     
            obj.err = inf;
            obj.parentTraj = [];                        % gets populated later
            obj.predecessors  = cell(0,1); 
            obj.errFromPredecessors = [];             % gets populated later
            obj.PMatFromPredecessors = cell(0,1);             % gets populated later
            obj.trajectoriesFromPredecessors = cell(0,1);
            obj.trajectoriesToNeighbors = cell(0,1);
        end
        
        
        function saveToFileID(obj, fileID)
            % saves a single node from the file, assuming that file is
            % already open for writing.
            % NOTE: this only save the structureal parts of the graph
            % and not any information for a particular search.
            
            fprintf(fileID,'%d\n', obj.id);
            fprintf(fileID,'%d\n', length(obj.position));
            
            for i = 1:length(obj.position)-1
                fprintf(fileID,'%f,', obj.position(i));
            end
            fprintf(fileID,'%f\n', obj.position(end));
                            
            fprintf(fileID,'%d\n',length(obj.neighbors));
            for i = 1:length(obj.neighbors)-1
                fprintf(fileID,'%d,', obj.neighbors{i}.id);
            end
            if length(obj.neighbors) > 0
                fprintf(fileID,'%d\n', obj.neighbors{length(obj.neighbors)}.id);
            end
            
            for i = 1:length(obj.neighbors)-1
                fprintf(fileID,'%f,', obj.costsToNeighbors(i));
            end
            if length(obj.neighbors) > 0
                fprintf(fileID,'%f\n', obj.costsToNeighbors(length(obj.neighbors)));
            end
        end
        
        
        function neighborInds = loadFromFileID(obj, fileID)
            % loads a single node from the file, assuming that file is
            % already open for reading.
            % it returns the indicies of this node's neighbors, and does
            % not set up the structure handles (this must be done once
            % all nodes have been read in the calling function)
            
            % NOTE: this only save the structureal parts of the graph
            % and not any information for a particular search.
            
            
            obj.id = fscanf(fileID,'%d\n', [1 1]);
            positionLength = fscanf(fileID,'%d\n', [1 1]);
            
            obj.position = zeros(1,positionLength);
            for i = 1:positionLength-1
                obj.position(i) = fscanf(fileID,'%f,', [1 1]);
            end              
            obj.position(positionLength) = fscanf(fileID,'%f\n', [1 1]);
            
            numNeighbors = fscanf(fileID,'%d\n', [1 1]);
            
            neighborInds = zeros(numNeighbors, 1);
            for i = 1:numNeighbors-1
                stuffRead = fscanf(fileID,'%d,', [1 1]);
                neighborInds(i) = stuffRead;
            end
            if numNeighbors > 0
                neighborInds(numNeighbors) = fscanf(fileID,'%d\n', [1 1]);
            end
            
            obj.costsToNeighbors = zeros(numNeighbors, 1);
            for i = 1:numNeighbors-1
                obj.costsToNeighbors(i) = fscanf(fileID,'%f,', [1 1]);
            end
            if numNeighbors > 0
                obj.costsToNeighbors(end) = fscanf(fileID,'%f\n', [1 1]); 
            end
        end
        
    end
end

