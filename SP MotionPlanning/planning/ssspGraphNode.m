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
        id                         % For use in motionGraph
        position                   % Mean position in belief space
        parent
        lmc                        % local minimum cost
        g                          % Cost / g value
        P                          % Error covariance
        dist                       % Dist to go
        successors = cell(0,0)     % successors
            
        % the following are used for sorting in the binary heap:
        heapIndex
        inHeap
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
    function obj = ssspGraphNode(position, parent)
        arguments
            position (:,1) double
            parent = []
        end

        obj.position = position;
        obj.successors = cell(0,1);

        obj.inHeap = false;
        obj.heapIndex = -1;
      
        obj.lmc = inf;
        obj.g = inf;
        obj.P =100*eye(6);

        obj.parent = parent;

        if ~isempty(parent); obj.dist = parent.dist + Planner.distFun(position, parent, 3.819718634205489e+02); else; obj.dist = inf;end

    end  

    function s = saveobj(obj)
        s.id = obj.id;
        s.position = obj.position;
        s.g = obj.g;
        s.lmc = obj.lmc;
        s.parent = obj.parent;
        % Do not save successors
    end

    function bool = isDescendant(obj, potentialAncestor)
        bool = false;
        currentNode = obj.parent;
        while ~isempty(currentNode)
            if currentNode == potentialAncestor
                bool = true;
                return;
            end
            currentNode = currentNode.parent;
        end
    end
    end
end

