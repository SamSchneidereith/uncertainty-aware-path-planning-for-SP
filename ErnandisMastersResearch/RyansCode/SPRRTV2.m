clear TrialNum Data
Trials = 100;
for k = Trials:-1:1
    Data(k).Path.IT = nan(1,1000);
    Data(k).Nodes.IT = cell(1,1000);
end
for TrialNum = 1:100
clearvars -except TrialNum Data
close all;
dbstop if error
warning('off', 'MATLAB:bvp4c:RelTolNotMet');   % turn off this particular warning about convergence tolerances not being met (we just throw out points of which this happens)
% these are the start/goal positions (note that they are reset
% to the closest nodes in the graph if the graph is loaded
% from a file), if instead we want to use a node ID, then
% that can be done with the following option

%------------------------- BEGIN USER INPUT ------------------------------%
Tol = 5e-1;             % Goal position tolerance          
algorithmToUse = 2;     % 0: RRT
                        % 1: RRT*
                        % 2: RRT#
SP.height = 326.2957;   % Base height from bottom plate to top plate [mm]

drawEachStep = false;   % Set to true to draw each step when plotting.
maxNodes = 10000;       % Max number of nodes in the graph structure.
goalBias = 0.05;        % This fraction of the time sample the goal
Loop = 0;
tend = 5*60;
%%% If using obstacles, add initial stuff here.

savePath = false;
pathFileName = 'SP_RRT.txt'; % Only used if savePath is true

numDims = 6;    % Number of dimensions in the C-space


fprintf('Trial Num %2i\n', TrialNum);
% Start and Goal Positions:
startPosition = [0, 0, SP.height, 0, 0, 0];
goalPosition = [50, 50, 50+SP.height, deg2rad(5), deg2rad(5), deg2rad(5)];

% Dimension Settings:
dimensionMins = [-100, -100, SP.height, deg2rad(-15), deg2rad(-15), deg2rad(-15)];
dimensionMaxs = [100, 100, 200+SP.height, deg2rad(15), deg2rad(15), deg2rad(15)];


wPos = norm(dimensionMaxs(1:3)-dimensionMins(1:3));
wRot = norm(dimensionMaxs(4:6)-dimensionMins(4:6));
W = 0.5*wPos/wRot; % Weight on Rotation terms in distance functions



delta = 10;         % Distance to expand tree toward random point
ballConstant = 100; % Affects the size of the relative neighborhood


%--------------------------- END USER INPUT ------------------------------%


steer = @SPSteer; % steering function to use
costAdd = @(PosCost,RotCost) (PosCost+W*RotCost);
%%% If using obstacles, call function to populate obstacles here.

% Set up stopping Criteria function:
stoppingCriteriaType = algorithmToUse;

if stoppingCriteriaType == 0
    % Assumes forward search, and that we set cost to start to be non-inf
    % when it is connected to the graph (used with RRT).
    stopingCriteriaMet = @(graph) (~isinf(graph.goalNode.costPosError) || graph.n >= graph.maxNodesAllowed);

    
elseif stoppingCriteriaType == 1
    % Assumes forward search, and that we just go until the max number of
    % nodes is reached.
    % stopingCriteriaMet = @(graph)  (graph.n >= graph.maxNodesAllowed);
    stopingCriteriaMet =@(tcur) (tcur >= tend);
    
elseif stoppingCriteriaType == 2
    % Assumes forward search, and that we just go until the max number of
    % nodes is reached.
    % stopingCriteriaMet = @(graph)  (graph.n >= graph.maxNodesAllowed);
    stopingCriteriaMet =@(tcur) tcur >= tend;
else
    error('unknown stopping criteria function')
end


% Set up graph structure:
G = motionGraph();
populateForSBSearch(G,  delta, dimensionMins, dimensionMaxs, ballConstant, startPosition, goalPosition, maxNodes, numDims);

% Set up parameters of topology for KD-tree and other functions:
kdTreeDistFunct = @(inputA, inputB) norm(inputA(1:3)-inputB(1:3))+W*norm(inputA(4:6)-inputB(4:6));
kdTree = KDTree(numDims, kdTreeDistFunct);

% No wrapping dimensions so none need to be set up (otherwise do it here).

% Set up trajectory distance function:
trajectoryLength = @TrajLength;


% Remember axis limits for plotting:
ax_bds = [dimensionMins(1:3) rad2deg(dimensionMins(4:6)); dimensionMaxs(1:3) rad2deg(dimensionMaxs(4:6))];
ax_bds = ax_bds(:)';


tstart = tic;           % start timing
tcur = 0;
Q = sbbstreeQ(maxNodes/10);    % Self balancing binary search tree (heap),
% this will grow if necessary
% NOTE: that because this implementation has
% only a singleton key and not a tupal key,
% the resulting algorithm may not work well
% in euclidian spaces on grids (or other
% places where cost value collisions are
% likely), it should be fine for the 2pbvp
% case here.

% ----------------- Start of algorithm -----------------


% Mark cost to start:
G.startNode.costPosError = 0;
G.startNode.costRotError = 0;
G.goalNode.costPosError = Inf;
G.goalNode.costRotError = Inf;

% Insert start and goal into the KD tree. Note that given reuse of sorting
% data structures with similarily named fields, we use different node types
% for the KD tree than for the graph. The KD tree nodes will hold a handle
% to the graph node in their (the KD tree node's) payload field. This is
% why we use the payload versions of the kdtree functions.

kdInsertAsPayload(kdTree, G.startNode);
goalInKDTree = false;
goalErr = inf;
% while ~stopingCriteriaMet(G)
while ~stopingCriteriaMet(tcur)
    
    % Get random point, occasionally sample the goal
    usingGoal = false;
    if rand() < goalBias
        % Use the goal node
        randPoint = G.goalNode.position;
        usingGoal = true;
    else
        GoodPoint = 1;
        while GoodPoint == 1
            % sample a random point
            randPoint = randomPoint(G);
            GoodPoint = PosCheck(randPoint');
        end
    end
    
    % Find the nearest point that is already in the graph
    [closestNode, distToclosestNode] = kdFindNearestPayload(kdTree, randPoint);
    
    % Now sample a new point found by steering from closestNode toward randPoint
    % note that the following function will return a local path from
    % closestNode's position to the new point
    [localTrajectory,trueTrajectory] = steer(closestNode.position', randPoint', G.delta,W,Tol);
    if isempty(localTrajectory)
        %   warning('could not connect')
        continue
    end
    
    newPoint = localTrajectory(end, 1:numDims);
    % New Version of Trajectory Length:
    costPosError = norm(localTrajectory(end,1:3)-trueTrajectory(end,1:3));
    costRotError = norm(localTrajectory(end,4:6)-trueTrajectory(end,4:6));
    % New Cost Function:
    costPosErrorOfNewPoint = closestNode.costPosError + costPosError;
    costRotErrorOfNewPoint = closestNode.costRotError + costRotError;
    parentNode = closestNode;                % we will change later if we find a better one (if using RRT*, etc)
    
    if algorithmToUse == 0
        % running RRT
        % we only need to consider one node for parent
        
    elseif algorithmToUse == 1
        % Running RRT*
        range = hyperBallRad(G);   % radius of shrinking D-Ball
        disp(['hyperball rad: ' num2str(range) ', nodes in graph: ' num2str(G.n)])
        
        kNN = kdFindWithinRange(kdTree, range, parentNode.position);
        numNeighbors = length(kNN);
        
        for n_i = 1:numNeighbors
            neighborNode = kNN{n_i}.payload;
            
            if closestNode.id == neighborNode.id
                % we've already calculated trajectory from this node to new
                % point (it was the first thing in localTrajectory)
                continue
            elseif kdTreeDistFunct(neighborNode.position, newPoint) + costAdd(neighborNode.costPosError,neighborNode.costRotError) < costAdd(costPosErrorOfNewPoint,costRotErrorOfNewPoint)
                % the above check is a heuristic check, if we are here than
                % the neighbor node MAY be better, but now we need to do the
                % harder actual 2pbvp check to make sure
                
                
                [neighborLocalTrajectory,neighborTrueTrajectory] = steer(neighborNode.position', newPoint', delta,W,Tol);
                if isempty(neighborLocalTrajectory)
                    % warning('could not connect from neighbor')
                    continue
                end

                
                
                
                
                thisCostPosError = norm(neighborLocalTrajectory(end,1:3)-neighborTrueTrajectory(end,1:3));
                thisCostRotError = norm(neighborLocalTrajectory(end,4:6)-neighborTrueTrajectory(end,4:6));
                if costAdd((thisCostPosError + neighborNode.costPosError),(thisCostRotError+neighborNode.costRotError)) < costAdd(costPosErrorOfNewPoint,costRotErrorOfNewPoint)
                    % we have found a better parent
                    costPosError = thisCostPosError;
                    costRotError = thisCostRotError;
                    localTrajectory = neighborLocalTrajectory;
                    costPosErrorOfNewPoint = costPosError + neighborNode.costPosError;
                    costRotErrorOfNewPoint = costRotError + neighborNode.costRotError; 
                    parentNode = neighborNode;
                end
                
            end
        end
        
        % at this point we have found the best neighbor in the set to use
        % as a parent
    elseif algorithmToUse == 2
        % Running RRT#
        % (we will save all neighbors as part of the graph)      
        % we take into account actual trajectory length for all nodes
        % within the shrinking D-ball
        
        range = hyperBallRad(G);   % radius of shrinking D-Ball
        %disp(['hyperball rad: ' num2str(range) ', nodes in graph: ' num2str(G.n)])
                
        kNN = kdFindWithinRange(kdTree, range, parentNode.position);
        numPredecessors = length(kNN);
        
        % unlike the slighly sloppy (but slightly faster) way we did it 
        % for RRT*, here for RRT# will will explicitly name the best 
        % trajectories as we loop over the neighbor set
        % at this point we search for predecessor neighbors
        
        bestCostPosError = Inf;
        bestCostRotError = Inf;
        bestPredecessorIndex = -1;
        
        predecessorNodes = cell(numPredecessors,1);
        trajectoriesFromPredecessors = cell(numPredecessors,1);
        trajectoryFromPredecessorCostPosError = inf(numPredecessors,1);
        trajectoryFromPredecessorCostRotError = inf(numPredecessors,1);
        numValidPredecessors = 0;  % incriments up as neighbors are added
        
        for n_i = 1:numPredecessors
            predecessorNode = kNN{n_i}.payload;
             
            if closestNode.id == predecessorNode.id
               % we've already calculated trajectroy from this node to new
               % point (it was the first thing in localTrajectory)
               % so just copy the following values over
               predecessorLocalTrajectory = localTrajectory;
               thisCostPosError = norm(localTrajectory(end,1:3)-trueTrajectory(end,1:3));
               thisCostRotError = norm(localTrajectory(end,4:6)-trueTrajectory(end,4:6));
               
               
            else
                %  now we need to do the 2pbvp check to make sure there is
                %  a valid trajectory from this node to its neighbor
              
              
                [predecessorLocalTrajectory,predecessorTrueTrajectory] = steer(predecessorNode.position', newPoint', delta,W,Tol);
                if isempty(predecessorLocalTrajectory)
                  % warning('could not connect from neighbor')
                  continue
                end
                
                thisCostPosError = norm(predecessorLocalTrajectory(end,1:3)-predecessorTrueTrajectory(end,1:3));
                thisCostRotError = norm(predecessorLocalTrajectory(end,4:6)-predecessorTrueTrajectory(end,4:6));
            end      
            
            % now we add this incomming edge (from the predessor neighbor)
            % to a temporary list that we will use later if this becomes an
            % actual node in the graph
            numValidPredecessors = numValidPredecessors + 1;
            predecessorNodes{numValidPredecessors,1} = predecessorNode;
            trajectoriesFromPredecessors{numValidPredecessors,1} = predecessorLocalTrajectory;
            trajectoryFromPredecessorCostPosError(numValidPredecessors,1) = thisCostPosError;
            trajectoryFromPredecessorCostRotError(numValidPredecessors,1) = thisCostRotError;
            
            if costAdd(thisCostPosError + min(predecessorNode.costPosError, predecessorNode.lmcPos),(thisCostRotError + min(predecessorNode.costRotError, predecessorNode.lmcRot))) < costAdd(bestCostPosError,bestCostRotError)
                % we have found a better parent
                
                bestCostPosError = min(predecessorNode.costPosError, predecessorNode.lmcPos) + costPosError;
                bestCostRotError = min(predecessorNode.costRotError, predecessorNode.lmcRot) + costRotError;
                bestPredecessorIndex = numValidPredecessors;
            end
        end   
        
        % now store the parent information 
        if numValidPredecessors <= 0
            error('had a connection but failed to find it in neighbor search')
        end 
        
        if bestPredecessorIndex < 0
          % no parent was found, so just continue  
           error('had a connection but failed to find a best predecessor')
        end
        

    end
    goalError = kdTreeDistFunct(newPoint,G.goalNode.position);
    %goalError = costAdd(norm(newPoint(1:3)-G.goalNode.position(1:3)),norm(newPoint(4:6)-G.goalNode.position(4:6)));
    if goalError < goalErr
        goalErr = goalError;
    end
    if goalErr < Tol && goalInKDTree == false
        
        % if we just got back the goal's position
        newNode = G.goalNode;
        if ~goalInKDTree
            kdInsertAsPayload(kdTree, newNode);
            goalInKDTree = true;
        end
    else
        
        % insert a new graph node at that point and get a handle to it
        newNode = insertNewNodeForSBSearch(G, newPoint);
        
        % also insert that node into the kd-tree
        kdInsertAsPayload(kdTree, newNode);
    end
    
    if algorithmToUse == 0 || algorithmToUse == 1
        % running RRT or RRT*
        % make the closest node the parent of the new node
        
        if parentNode.id == newNode.id
            % don't make a node a parent of itself
            continue
        end
        
        newNode.parentID = parentNode.id;
        newNode.parentNode = parentNode;
        newNode.parentTraj = localTrajectory;
        % newNode.costFromStart = localTrajectoryLength + parentNode.costFromStart;
        newNode.costPosError = costPosError + parentNode.costPosError;
        newNode.costRotError = costRotError + parentNode.costRotError;
        
    elseif algorithmToUse == 2
        % running RRT#
               
        % now add the "from" neighbors and neighbor trajectories (recall
        % we're doing forward search, so "from" neighbors are the predessors we
        % follow back to the start, and "to" neighbors are the successors that
        % might ba able to use this node as a parent)
        
        if newNode.id == G.goalNode.id
            % the goal is the tricky case, since it already has some of
            % these neighbors stored already
            
            % first figure out which of the newly found neighbors are
            % actually new neighbors
            predecessorInds = inf(length(newNode.predecessors) + numValidPredecessors,1);
            
            for n_i = 1:length(newNode.predecessors)
              predecessorInds(n_i) = newNode.predecessors{n_i}.id;
            end
            for n_i = 1:numValidPredecessors
                if predecessorNodes{n_i}.id == G.goalNode.id
                  % we also don't want to use the goal as its own predecessor
                  continue
                end
                predecessorInds(length(newNode.neighbors) + n_i) = predecessorNodes{n_i}.id;
            end
            [junk, uniqueIndsInds, alsoJunk] = unique(predecessorInds, 'stable');
            
            % the following will remove all indicies associated with old
            % predecessors in the goal (keeping only the indicies of the
            % new predecessors)
            uniqueIndsInds = uniqueIndsInds(uniqueIndsInds > length(newNode.predecessors)) - length(newNode.predecessors);
            
            % now finally copy over the new ones
            oldLength = length(newNode.predecessors);
            newNode.predecessors = [newNode.predecessors ; predecessorNodes(uniqueIndsInds,1)];
            newNode.costsPosFromPredecessors = [newNode.costsPosFromPredecessors ; trajectoryFromPredecessorCostPosError(uniqueIndsInds,1)];  
            newNode.costsRotFromPredecessors = [newNode.costsRotFromPredecessors ; trajectoryFromPredecessorCostRotError(uniqueIndsInds,1)];
            newNode.trajectoriesFromPredecessors = [newNode.trajectoriesFromPredecessors ; trajectoriesFromPredecessors(uniqueIndsInds,1)];     
            
            % update the indicies to reflet the new predecessors actually stored in the node's list 
            uniqueIndsInds = oldLength+1:length(newNode.predecessors);
            
            % now we need to see if any of the new nodes yield a better cost from start 
            bestPredecessorIndex = -1;
            bestCostPosError = inf;
            bestCostRotError = inf;

            for n_i = 1:length(newNode.predecessors)
                thisPredecessor = newNode.predecessors{n_i};
                thisCostPosError = thisPredecessor.costPosError + newNode.costsPosFromPredecessors(n_i,1);
                thisCostRotError = thisPredecessor.costRotError + newNode.costsRotFromPredecessors(n_i,1);
                
                if costAdd(thisCostPosError,thisCostRotError) < costAdd(bestCostPosError,bestCostRotError)
                    bestCostPosError = thisCostPosError;
                    bestCostRotError = thisCostRotError;
                    bestPredecessorIndex = n_i;
                end
            end
            
            
            if  bestPredecessorIndex > 0
                % only update the goal's parent if it actually helps
                parentNode = newNode.predecessors{bestPredecessorIndex,1};  
              
                newNode.lmcPos = bestCostPosError;
                newNode.lmcRot = bestCostRotError;
                newNode.parentID = parentNode.id;
                newNode.parentNode = parentNode;
                newNode.parentTraj = newNode.trajectoriesFromPredecessors{bestPredecessorIndex,1};
              
            end
            
        else
            
            parentNode = predecessorNodes{bestPredecessorIndex};
            
            
            % for normal nodes, we just init these things as required
            newNode.lmcPos = trajectoryFromPredecessorCostPosError(bestPredecessorIndex) + parentNode.costPosError;
            newNode.lmcRot = trajectoryFromPredecessorCostRotError(bestPredecessorIndex) + parentNode.costRotError;
            newNode.parentID = parentNode.id;
            newNode.parentNode = parentNode;
            newNode.predecessors = predecessorNodes(1:numValidPredecessors);
            newNode.costsPosFromPredecessors = trajectoryFromPredecessorCostPosError(1:numValidPredecessors,1);  
            newNode.costsRotFromPredecessors = trajectoryFromPredecessorCostRotError(1:numValidPredecessors,1);  
            newNode.trajectoriesFromPredecessors = trajectoriesFromPredecessors(1:numValidPredecessors,1);
            newNode.parentTraj = trajectoriesFromPredecessors{bestPredecessorIndex,1};
            
            uniqueIndsInds = 1:numValidPredecessors;

        end 
        

        % now we need to update the successor lists of all these predessors
        % to include the new node and trajectory
        
        for n_i_i = 1:length(uniqueIndsInds)
          n_i = uniqueIndsInds(n_i_i);
          predessorNode = newNode.predecessors{n_i};
           
          numValidSuccessors = length(predessorNode.neighbors) + 1;
          
          
          % this is a place for potential speed improvement, since right
          % now we're growing these on the fly without any explicit
          % prealocation of memory
          predessorNode.neighbors{numValidSuccessors,1} = newNode; 
          predessorNode.costsPosToNeighbors(numValidSuccessors,1) = newNode.costsPosFromPredecessors(n_i,1);
          predessorNode.costsRotToNeighbors(numValidSuccessors,1) = newNode.costsRotFromPredecessors(n_i,1);
          predessorNode.trajectoriesToNeighbors{numValidSuccessors,1} = newNode.trajectoriesFromPredecessors{n_i,1};
        end
  
    end
    
    if algorithmToUse == 1
        % we are running RRT*
        % we need to adds the reverse edges to the graph
        for n_i = 1:length(kNN)
            
           neighborNode = kNN{n_i}.payload;
             
           if neighborNode.id == newNode.parentNode.id || neighborNode.id == newNode.id
              % we do not want to rewire the new node or its parent
              continue 
           end
           
           % we do want to see if we would like to rewire these other
           % neighbors to use the new node as their parent

             
            if kdTreeDistFunct(newNode.position, neighborNode.position) + costAdd(newNode.costPosError,newNode.costRotError) < costAdd(neighborNode.costPosError,neighborNode.costRotError)
                % the above check is a heuristic check, if we are here than
                % the neighbor node MAY do better to use the new node as 
                % its parent, but now we need to do the harder actual 2pbvp 
                % check to make sure
              
              
                [neighborLocalTrajectory,neighborTrueTrajectory] = steer(newNode.position', neighborNode.position', delta, W, Tol);
                if isempty(neighborLocalTrajectory)
                   % warning('could not connect to neighbor')
                  continue
                end
 
                thisCostPosError = norm(neighborLocalTrajectory(end,1:3)-neighborTrueTrajectory(end,1:3));
                thisCostRotError = norm(neighborLocalTrajectory(end,4:6)-neighborTrueTrajectory(end,4:6));
                if costAdd(thisCostPosError + newNode.costPosError,thisCostRotError+newNode.costRotError) < costAdd(neighborNode.costPosError,neighborNode.costRotError)
                  % the new node is a better parent to the neighbor than it
                  % current parent
                  
                  neighborNode.parentID = newNode.id;
                  neighborNode.parentNode = newNode;
                  neighborNode.parentTraj = neighborLocalTrajectory;
                  neighborNode.costPosError = thisCostPosError + newNode.costPosError;
                  neighborNode.costRotError = thisCostRotError + newNode.costRotError;
                end
                
            end    
            
        end        
    elseif algorithmToUse == 2
        % running RRT#
        % need to add reverse edges to the graph
        
        if newNode.id ~= G.goalNode.id
            % we'll take the lazy approach and just say that no nodes will 
            % care to direcly use the goal node as their parent 
            % (checking this would be slighly more involved since it would
            % tend to accumulate neighbors)
             
            % store these temproarily here, for building, then copy to node later
            numSuccessors = length(kNN); 
            sucessorNodes = cell(numSuccessors,1);
            trajectoriesToSuccessors = cell(numSuccessors,1);
            trajectoryToSuccessorPosCost = inf(numSuccessors,1);
            trajectoryToSuccessorRotCost = inf(numSuccessors,1);
            numValidSuccessors = 0;  % incriments up as neighbors are added
            
            
            for n_i = 1:length(kNN)
            
                neighborNode = kNN{n_i}.payload;
             
                if neighborNode.id == newNode.id
                    % we do not want to rewire the new node to itself
                    % (though, under normal circumstance I do not think
                    % this case actually happens)
                    continue 
                end
          
                [neighborLocalTrajectory,neighborTrueTrajectory] = steer(newNode.position' , neighborNode.position', delta,W,Tol);
                if isempty(neighborLocalTrajectory)
                   % warning('could not connect to neighbor')

                   continue
                end

                thisCostPosError = norm(neighborLocalTrajectory(end,1:3)-neighborTrueTrajectory(end,1:3));
                thisCostRotError = norm(neighborLocalTrajectory(end,4:6)-neighborTrueTrajectory(end,4:6));
                
                numValidPredecessors = length(neighborNode.predecessors) + 1;
                neighborNode.predecessors{numValidPredecessors,1} = neighborNode; 
                neighborNode.costsPosFromPredecessors(numValidPredecessors,1) = thisCostPosError;
                neighborNode.costsRotFromPredecessors(numValidPredecessors,1) = thisCostRotError;
                neighborNode.trajectoriesFromPredecessors{numValidPredecessors,1} = neighborLocalTrajectory;
                
                
                numValidSuccessors = numValidSuccessors + 1;
                sucessorNodes{numValidSuccessors,1} = neighborNode;
                trajectoriesToSuccessors{numValidSuccessors,1} = neighborLocalTrajectory;
                trajectoryToSuccessorPosCost(numValidSuccessors,1) = thisCostPosError;
                trajectoryToSuccessorRotCost(numValidSuccessors,1) = thisCostRotError;
            end  
            
            
            % now copy over the successors into the neighbor spots
            newNode.neighbors = sucessorNodes(1:numValidSuccessors,1);
            newNode.costsPosToNeighbors = trajectoryToSuccessorPosCost(1:numValidSuccessors,1);
            newNode.costsRotToNeighbors = trajectoryToSuccessorRotCost(1:numValidSuccessors,1);
            newNode.trajectoriesToNeighbors = trajectoriesToSuccessors(1:numValidSuccessors,1);
            
        end   
        
        % now add the newly created to the queue (or update its position if
        % if it is the goal node and already in there... update() handles 
        % the case the node is already in the queue automatically)
        update(Q, newNode, min(costAdd(newNode.costPosError,newNode.costRotError), costAdd(newNode.lmcPos,newNode.lmcRot)));
        
        % now we do the replan (i.e. the make consistant) step
        while(~isempty(topKey(Q)) )   % && topKey(Q) <= min(G.goalNode.costFromStart, G.goalNode.lmc) )
          % while the heap is nonempty and the top nodes is at a lower level set than the goal
        
         
          thisNode = pop(Q);                        % get top node
          thisNode.costPosError = thisNode.lmcPos;    % set cost to lmc
          thisNode.costRotError = thisNode.lmcRot;    % set cost to lmc
          
          % now we need to see if any of the nodes that can be accessed
          % from this node would like to use this node as their parent
          
          for n_i = 1:length(thisNode.neighbors)
              successorNode = thisNode.neighbors{n_i};
              if successorNode.id == thisNode.parentNode.id  
                  continue  % explicitly avoid parent loops (in cases of numerical rounding error this could be an issue) 
              elseif successorNode.id == G.startNode.id
                  continue  % by definition the start node cannot be improved
              end
              
              costPosFromStartViaNewNode = thisNode.costPosError + thisNode.costsPosToNeighbors(n_i,1);
              costRotFromStartViaNewNode = thisNode.costRotError + thisNode.costsRotToNeighbors(n_i,1);
              if costAdd(successorNode.lmcPos,successorNode.lmcRot) > costAdd(costPosFromStartViaNewNode,costRotFromStartViaNewNode)
                  successorNode.lmcPos = costPosFromStartViaNewNode;
                  successorNode.lmcRot = costRotFromStartViaNewNode;
                  successorNode.parentID = thisNode.id;
                  successorNode.parentNode = thisNode;
                  successorNode.parentTraj = thisNode.trajectoriesToNeighbors{n_i,1};
              end
              
              if costAdd(successorNode.costPosError,successorNode.costRotError) == costAdd(successorNode.lmcPos,successorNode.lmcRot)
                remove(Q, successorNode);
              else
                update(Q, successorNode, min(costAdd(newNode.costPosError,newNode.costRotError),costAdd(newNode.lmcPos,newNode.lmcRot)));
              end            
          end
          
          
          
          
        end
        
    end
    
    
    if algorithmToUse == 0
       disp(num2str(G.n))
    end
        
   
   
       % ------------------- plot stuff -----------------
    if drawEachStep 
        fig = figure(1);
        fig.Units = 'normalized';
        fig.Position = [0 0 1 1];
        subplot(1,2,1)
        %G.plotEdges('Color',[0.7,0.7,0.7])  % draws edges gray
        hold on
        G.calculateParentEdgesForPlotting()     % note that this takes a bit of time, only use for demo
        G.plotParentEdges('r', 'LineWidth', 2)
        axis equal
        hold on
        
        %G.plotNodesWithInListVal(0, '.k', 'MarkerSize', 3)  % unvisited
        %G.plotNodesWithInListVal(1, 'or', 'MarkerSize', 3)  % open
        %G.plotNodesWithInListVal(2, 'ok', 'MarkerSize', 3)  % closed
        
        G.plotStartNode3DPosition('ob', 'MarkerSize', 10, 'LineWidth', 3)
        G.plotGoalNode3DPosition( 'xb', 'MarkerSize', 10, 'LineWidth', 3)
        
        hold off
        axis(ax_bds(1:6))
        grid on
        
        xlabel('X')
        ylabel('Y')
        zlabel('Z')
        title('Position')
        
        subplot(1,2,2)
        G.plotParentRotEdges('r', 'LineWidth', 2)
        hold on
        
        G.plotStartNode3DOrientation('ob', 'MarkerSize', 10, 'LineWidth', 3)
        G.plotGoalNode3DOrientation('xb', 'MarkerSize', 10, 'LineWidth', 3)
        hold off
        axis(ax_bds(7:12))
        grid on
        xlabel('Xrot')
        ylabel('Yrot')
        zlabel('Zrot')
        title('Orientation')
        pause(.000)        
    end
    % ------------------- end plot stuff -----------------
    pause(0.0)
   %} 
   tcur=toc(tstart);
   
   if goalInKDTree == true
       Loop = Loop +1;
       G.extractMotionPath();
       Data(TrialNum).Path(Loop).IT = G.motionPath;
       Data(TrialNum).Nodes(Loop).IT = G.nodes(G.motionPath);      
   end
   
end
end

%%
% Extract Path
G.extractMotionPath()

if savePath
    csvwrite(pathFileName,G.motionPath')
end

calculatePlottingData(G)
trajForPlotting = getParentTrajectoriesForPlotting(G,6);
bestPathTrajForPlotting = getBestPathTrajectoriesForPlotting(G,6);


% ------------------- plot stuff -----------------

fig = figure(2);
fig.Units = 'normalized';
fig.Position = [0 0 1 1];
subplot(1,2,1)
G.plotEdges('Color',[0.7,0.7,0.7])  % draws edges gray
hold on
G.calculateParentEdgesForPlotting()

G.plotParentEdges('r','LineWidth', 2)

G.plotNodesWithInListVal(0, '.k', 'MarkerSize', 3)  % unvisited
G.plotNodesWithInListVal(1, 'or', 'MarkerSize', 3)  % open
G.plotNodesWithInListVal(2, 'ok', 'MarkerSize', 3)  % closed

G.plotMotionPath('b',  'LineWidth', 4)

G.plotStartNode('ob', 'MarkerSize', 10, 'LineWidth', 3)
G.plotGoalNode( 'xb', 'MarkerSize', 10, 'LineWidth', 3)
grid on
hold off
axis(ax_bds(1:6))
axis equal
xlabel('X')
ylabel('Y')
zlabel('Z')
title('Posistion')

subplot(1,2,2)
G.plotRotEdges('Color',[0.7,0.7,0.7])  % draws edges gray
hold on
G.calculateParentEdgesForPlotting()

G.plotParentRotEdges('r','LineWidth', 2)

G.plotNodesWithInListValRot(0, '.k', 'MarkerSize', 3)  % unvisited
G.plotNodesWithInListValRot(1, 'or', 'MarkerSize', 3)  % open
G.plotNodesWithInListValRot(2, 'ok', 'MarkerSize', 3)  % closed

G.plotRotMotionPath('b',  'LineWidth', 4)

G.plotStartNodeRot('ob', 'MarkerSize', 10, 'LineWidth', 3)
G.plotGoalNodeRot( 'xb', 'MarkerSize', 10, 'LineWidth', 3)
grid on
hold off
axis(ax_bds(7:12))
axis equal
xlabel('Xrot')
ylabel('Yrot')
zlabel('Zrot')
title('Orientation')


% now plot just the search tree
fig = figure(3);
fig.Units = 'normalized';
fig.Position = [0 0 1 1];
subplot(1,2,1)
G.plotParentEdges('r', 'LineWidth', 2)
axis(ax_bds(1:6))
hold on

G.plotMotionPath('b',  'LineWidth', 4)

G.plotStartNode('ob', 'MarkerSize', 10, 'LineWidth', 3)
G.plotGoalNode( 'xb', 'MarkerSize', 10, 'LineWidth', 3)
grid on
hold off
axis equal
xlabel('X')
ylabel('Y')
zlabel('Z')
title('Posistion')

subplot(1,2,2)
G.plotParentRotEdges('r', 'LineWidth', 2)
axis(ax_bds(7:12))
axis equal
hold on

G.plotRotMotionPath('b',  'LineWidth', 4)

G.plotStartNodeRot('ob', 'MarkerSize', 10, 'LineWidth', 3)
G.plotGoalNodeRot( 'xb', 'MarkerSize', 10, 'LineWidth', 3)
grid on
hold off
axis equal
xlabel('Xrot')
ylabel('Yrot')
zlabel('Zrot')
title('Orientation')
% Save figure to file:
filename = pwd;
if algorithmToUse == 0
    filename = strcat(filename,'\RRT\','W2ChosenPath.png');
elseif algorithmToUse == 1
    filename = strcat(filename,'\RRTstar\','W2ChosenPath.png');
elseif algorithmToUse == 2
    filename = strcat(filename,'\RRTSharp\','W2ChosenPath.png');
end
saveas(fig,filename);

fig = figure(4);
fig.Units = 'normalized';
fig.Position = [0 0 1 1];
subplot(1,2,1)
plot3(trajForPlotting(:,1), trajForPlotting(:,2), trajForPlotting(:,3),'m', 'LineWidth', 2)
axis(ax_bds(1:6))
axis equal
hold on 
plot3(bestPathTrajForPlotting(:,1), bestPathTrajForPlotting(:,2), bestPathTrajForPlotting(:,3), 'b', 'LineWidth', 2)
G.plotStartNode('ok', 'MarkerSize', 10, 'LineWidth', 1)
G.plotGoalNode( 'xk', 'MarkerSize', 10, 'LineWidth', 1)
grid on
[x,y,z] = sphere;
radP = zeros(1,size(G.motionPath,2));
for i = 1:size(G.motionPath,2)
    radP(i) = G.nodes{G.motionPath(i)}.costPosError;
    xr = x*radP(i);
    yr = y*radP(i);
    zr = z*radP(i);
    xloc = xr+G.nodes{G.motionPath(i)}.position(1);
    yloc = yr+G.nodes{G.motionPath(i)}.position(2);
    zloc = zr+G.nodes{G.motionPath(i)}.position(3);
    ErrBall = surf(xloc,yloc,zloc);
    ErrBall.EdgeAlpha = 0;
    ErrBall.FaceColor = [0 1 1];
    ErrBall.FaceAlpha = 0.5;
end
hold off;
axis equal
xlabel('X')
ylabel('Y')
zlabel('Z')
title('Posistion')

subplot(1,2,2)
plot3(rad2deg(trajForPlotting(:,4)), rad2deg(trajForPlotting(:,5)), rad2deg(trajForPlotting(:,6)),'m', 'LineWidth', 2)
axis(ax_bds(7:12))
axis equal
hold on 
plot3(rad2deg(bestPathTrajForPlotting(:,4)), rad2deg(bestPathTrajForPlotting(:,5)), rad2deg(bestPathTrajForPlotting(:,6)), 'b', 'LineWidth', 2)
G.plotStartNodeRot('ok', 'MarkerSize', 10, 'LineWidth', 1)
G.plotGoalNodeRot( 'xk', 'MarkerSize', 10, 'LineWidth', 1)
grid on
[x,y,z] = sphere;
radO = zeros(1,size(G.motionPath,2));
for i = 1:size(G.motionPath,2)
    radO(i) = rad2deg(G.nodes{G.motionPath(i)}.costRotError);
    xr = x*radO(i);
    yr = y*radO(i);
    zr = z*radO(i);
    xloc = xr+rad2deg(G.nodes{G.motionPath(i)}.position(4));
    yloc = yr+rad2deg(G.nodes{G.motionPath(i)}.position(5));
    zloc = zr+rad2deg(G.nodes{G.motionPath(i)}.position(6));
    ErrBall = surf(xloc,yloc,zloc);
    ErrBall.EdgeAlpha = 0;
    ErrBall.FaceColor = [0 1 1];
    ErrBall.FaceAlpha = 0.1;
end
hold off;
xlabel('Xrot')
ylabel('Yrot')
zlabel('Zrot')
title('Orientation')
filename = pwd;
if algorithmToUse == 0
    filename = strcat(filename,'\RRT\','W2ChosenPathErrBall.png');
elseif algorithmToUse == 1
    filename = strcat(filename,'\RRTstar\','W2ChosenPathErrBall.png');
elseif algorithmToUse == 2
    filename = strcat(filename,'\RRTSharp\','W2ChosenPathErrBall.png');
end
saveas(fig,filename);

% Plot Cost from Start along the Path:
fig = figure(5);
subplot(1,2,1)
j = 0;
for i = size(G.motionPath,2):-1:1
    j=j+1;
    xAx(j) = j;
    yAxP(j) = G.nodes{G.motionPath(i)}.costPosError;
end
plot(xAx,yAxP);
xlim([0, max(xAx)]);
xlabel('Node on Path');
ylabel('Position Error');
title('Error Growth along Path');

subplot(1,2,2)
j = 0;
for i = size(G.motionPath,2):-1:1
    j=j+1;
    xAx(j) = j;
    yAxR(j) = rad2deg(G.nodes{G.motionPath(i)}.costRotError);
end
plot(xAx,yAxR);
xlim([0, max(xAx)]);
xlabel('Node on Path');
ylabel('Rotation Error');
title('Error Growth along Path');
hold off
filename = pwd;
if algorithmToUse == 0
    filename = strcat(filename,'\RRT\','W2ErrorAlongPath.png');
elseif algorithmToUse == 1
    filename = strcat(filename,'\RRTstar\','W2ErrorAlongPath.png');
elseif algorithmToUse == 2
    filename = strcat(filename,'\RRTSharp\','W2ErrorAlongPath.png');
end
saveas(fig,filename);
%
if algorithmToUse == 0
fileID = fopen('W2RRTErr.txt','w');
elseif algorithmToUse == 1
    fileID = fopen('W2RRTStarErr.txt','w');
elseif algorithmToUse == 2
    fileID = fopen('W2RRTSharpErr.txt','w');
end
fprintf(fileID,'%i\n',size(G.motionPath,2));
fprintf(fileID,'%f\n',yAxP);
fprintf(fileID,'%f\n',yAxR);
fclose(fileID);

%% Run This after running all 3 algorithms:
ErrorComparison();