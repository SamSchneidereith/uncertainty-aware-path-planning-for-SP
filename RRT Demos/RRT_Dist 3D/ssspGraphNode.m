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



    % Modified by Sam Schneidereith 2026  
    properties
        id               % For use in motionGraph
        position         % the node's position in the C-space (i.e., its state) 
        % neighbors        % an array of node handles (to sucessor neighbors, if that is considered)
        
        
        parentNode       % node handle of this node's parent
        
              
        % the following are used for sorting in the binary heap:
        heapIndex        % (necessary to enable internal removes)
        inHeap           % (true if in heap, otherwise false)
        
        % the following are used more for sampling based motion planning
        lmc                        % local minimum cost
        g                          % Cost / g value - Sam
        successors = cell(0,0)     % successors


        % err                        % Trace of the error covariance matrix
        % PMat                       % Error Covariance matrix 
        % parentTraj                 % trajectory from parent
        % errFromPredecessors      % an array of costs of Pos to the neighbors
        % PMatFromPredecessors
        % trajectoriesFromPredecessors   % an array of trajectories (arrays)
        % trajectoriesToNeighbors        % an array of trajectories (arrays)
        % errToNeighbors   % an array of costs to the neighbors
        % PMatToNeighbors  % an array of Error Cov matrices to the neighbors
    end
    methods
        function obj = ssspGraphNode(positionState)
            obj.position = positionState;
            % obj.neighbors = cell(0,1);          % gets populated later
            obj.successors = cell(0,1);

            obj.inHeap = false;
            obj.heapIndex = -1;
          
            obj.lmc = inf;
            obj.g = inf;

            % obj.errToNeighbors = [];        % gets populated later
            % obj.PMatToNeighbors = cell(0,1);        
            % obj.parentID = 0;
            % obj.parentNode;                 % gets populated later
            % obj.PMat = 1*eye(numDims);     
            % obj.err = inf;
            % obj.parentTraj = [];                        % gets populated later
            % obj.predecessors  = cell(0,1); 
            % obj.errFromPredecessors = [];             % gets populated later
            % obj.PMatFromPredecessors = cell(0,1);             % gets populated later
            % obj.trajectoriesFromPredecessors = cell(0,1);
            % obj.trajectoriesToNeighbors = cell(0,1);

            % if ~isempty(parentNode)
            %     %work on me later
            % end
        end        
    end
end

