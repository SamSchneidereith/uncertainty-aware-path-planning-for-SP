clear variables;
close all;
dbstop if error
warning('off', 'MATLAB:bvp4c:RelTolNotMet');   % turn off this particular warning about convergence tolerances not being met (we just throw out points of which this happens)
% these are the start/goal positions (note that they are reset
% to the closest nodes in the graph if the graph is loaded
% from a file), if instead we want to use a node ID, then
% that can be done with the following option

problemDefinition = 2;  % 1: Stewart Platform [x,y,z,Xrot,Yrot,Zrot]
                        % 2: 2D Example [x,y]  
%------------------------- BEGIN USER INPUT ------------------------------%
Tol = 0.1;              % Goal position tolerance          
algorithmToUse = 2;     % 0: RRT
                        % 1: RRT*
                        % 2: RRT#
SP.height = 326.2957;   % Base height from bottom plate to top plate [mm]

drawEachStep = true;   % Set to true to draw each step when plotting.
maxNodes = 3000;       % Max number of nodes in the graph structure.
goalBias = 0.05;        % This fraction of the time sample the goal
Loop = 0;
tend = 60;
%%% If using obstacles, add initial stuff here.

savePath = false;
pathFileName = 'SP_RRT.txt'; % Only used if savePath is true

if problemDefinition == 1
    numDims = 6;    % Number of dimensions in the C-space


    fprintf('Trial Num %2i\n', TrialNum);
    % Start and Goal Positions:
    startPosition = [0, 0, SP.height, 0, 0, 0];
    goalPosition = [50, 50, 50+SP.height, deg2rad(5), deg2rad(5), deg2rad(5)];

    % Dimension Settings:
    dimensionMins = [-100, -100, SP.height, deg2rad(-15), deg2rad(-15), deg2rad(-15)];
    dimensionMaxs = [100, 100, 200+SP.height, deg2rad(15), deg2rad(15), deg2rad(15)];

    % Position Check:
    PC = @PosCheck;
    
    wPos = norm(dimensionMaxs(1:3)-dimensionMins(1:3));
    wRot = norm(dimensionMaxs(4:6)-dimensionMins(4:6));
    W = 0.5*wPos/wRot; % Weight on Rotation terms in distance functions



    delta = 10;         % Distance to expand tree toward random point
    ballConstant = 100; % Affects the size of the relative neighborhood




    steer = @SPSteer; % steering function to use
    costAdd = @(PosCost,RotCost) (PosCost+W*RotCost);
    CostCompare = @(PosLHS,RotLHS,PosRHS,RotRHS) (PosLHS < PosRHS && RotLHS < RotRHS);
    
    
elseif problemDefinition == 2
    numDims = 2;
    
    startPosition = initState([80; 50])';
    goalPosition = initState([150; 100])';
    
    % The min and max dimensions based on the tested range of the
    % actuators, however they include positions impossible to be reached,
    % therefore a check function is needed.
    dimensionMins = [-145, 490]; 
    dimensionMaxs = [145, 790];
    
    % Point check function:
    PC = @PosCheck2D;
    
    % Error calculation Function:
    % ErrFun = @(PMat) trace(PMat);
    ErrFun = @VolEllipsoid;
    
    W = 1; % No weighting in this case, just needed for continuity
    delta = 10;
    ballConstant = 100;
    steer = @Steer2D_V2;
    
end

%--------------------------- END USER INPUT ------------------------------%
% Set up stopping Criteria function:
stoppingCriteriaType = algorithmToUse;

if stoppingCriteriaType == 0
    % Assumes forward search, and that we set cost to start to be non-inf
    % when it is connected to the graph (used with RRT).
    stopingCriteriaMet = @(graph) (~isinf(norm(graph.goalNode.traceErr)) || graph.n >= graph.maxNodesAllowed);
    % stopingCriteriaMet = @(graph) (graph.n >= graph.maxNodesAllowed);
    
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

if problemDefinition == 1
    % Set up parameters of topology for KD-tree and other functions:
    kdTreeDistFunct = @(inputA, inputB) norm(inputA(1:3)-inputB(1:3))+W*norm(inputA(4:6)-inputB(4:6));
    kdTree = KDTree(numDims, kdTreeDistFunct);

    % No wrapping dimensions so none need to be set up (otherwise do it here).

    % Set up trajectory distance function:
    trajectoryLength = @TrajLength;
    

    
    % Remember axis limits for plotting:
    ax_bds = [dimensionMins(1:3) rad2deg(dimensionMins(4:6)); dimensionMaxs(1:3) rad2deg(dimensionMaxs(4:6))];
    ax_bds = ax_bds(:)';
elseif problemDefinition == 2
    
    
    kdTreeDistFunct = @(inputA, inputB) norm(inputA-inputB);
    kdTree = KDTree(numDims, kdTreeDistFunct);
    trajectoryLength = @TrajLength2D;
    
    ax_bds = [dimensionMins; dimensionMaxs];
    ax_bds = ax_bds(:)';
    
end

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
G.startNode.err = 0;
% G.startNode.travFromStart = 0;

G.goalNode.err = inf;
% G.goalNode.travFromStart = inf;
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
            GoodPoint = PC(randPoint'); % Position Check random point
        end
    end
    
    % Find the nearest point that is already in the graph
    [closestNode, distToclosestNode] = kdFindNearestPayload(kdTree, randPoint);
    
    % Now sample a new point found by steering from closestNode toward randPoint
    % note that the following function will return a local path from
    % closestNode's position to the new point
    [localTrajectory, newPMat] = steer(closestNode.position', randPoint', G.delta,W,Tol, closestNode.PMat);
    if isempty(localTrajectory)
        %   warning('could not connect')
        continue
    end
    
    newPoint = localTrajectory(end, :);
    newErr = ErrFun(newPMat);

    % Taking Error as the new Cost:
    
    parentNode = closestNode;                % we will change later if we find a better one (if using RRT*, etc)
    % localTrav = trajectoryLength(localTrajectory) + parentNode.travFromStart;
    
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
            % elseif kdTreeDistFunct(neighborNode.position, newPoint) + neighborNode.travFromStart < localTrav
            elseif neighborNode.err < newErr
                % the above check is a heuristic check, if we are here than
                % the neighbor node MAY be better, but now we need to do the
                % harder actual 2pbvp check to make sure
                
                
                [neighborLocalTrajectory,neighborPMat] = steer(neighborNode.position', newPoint', delta,W,Tol,neighborNode.PMat);
                if isempty(neighborLocalTrajectory)
                    % warning('could not connect from neighbor')
                    continue
                end
                
                % thisTrav = trajectoryLength(neighborLocalTrajectory);
                neighborErr = ErrFun(neighborPMat); 
                
                if neighborErr < newErr
                    % we have found a better parent
                    
                    newErr = neighborErr;
                    newPMat = neighborPMat;
                    % localTrav = thisTrav + neighborNode.travFromStart;
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
        disp(['hyperball rad: ' num2str(range) ', nodes in graph: ' num2str(G.n)])
                
        kNN = kdFindWithinRange(kdTree, range, parentNode.position);
        numPredecessors = length(kNN);
        
        % unlike the slighly sloppy (but slightly faster) way we did it 
        % for RRT*, here for RRT# will will explicitly name the best 
        % trajectories as we loop over the neighbor set
        % at this point we search for predecessor neighbors
        
        bestErr = Inf;
        bestPredecessorIndex = -1;
        
        predecessorNodes = cell(numPredecessors,1);
        trajectoriesFromPredecessors = cell(numPredecessors,1);
        % travFromPredecessor = inf(numPredecessors,1);
        PMatFromPredecessor = cell(numPredecessors,1);
        errFromPredecessor = inf(numPredecessors,1);
        numValidPredecessors = 0;  % incriments up as neighbors are added
        
        for n_i = 1:numPredecessors
            predecessorNode = kNN{n_i}.payload;
             
            if closestNode.id == predecessorNode.id
               % we've already calculated trajectroy from this node to new
               % point (it was the first thing in localTrajectory)
               % so just copy the following values over
               predecessorLocalTrajectory = localTrajectory;
               thisErr = newErr;
               thisPMat = newPMat;

            else
                %  now we need to do the 2pbvp check to make sure there is
                %  a valid trajectory from this node to its neighbor
              
              
                [predecessorLocalTrajectory,predecessorPMat] = steer(predecessorNode.position', newPoint', delta,W,Tol, predecessorNode.PMat);
                if isempty(predecessorLocalTrajectory)
                  % warning('could not connect from neighbor')
                  continue
                end
                
                
                thisPMat = predecessorPMat;
                thisErr = ErrFun(predecessorPMat);
            end      
            
            % now we add this incomming edge (from the predessor neighbor)
            % to a temporary list that we will use later if this becomes an
            % actual node in the graph
            numValidPredecessors = numValidPredecessors + 1;
            predecessorNodes{numValidPredecessors,1} = predecessorNode;
            trajectoriesFromPredecessors{numValidPredecessors,1} = predecessorLocalTrajectory;
            errFromPredecessor(numValidPredecessors,1) = thisErr;
            PMatFromPredecessor{numValidPredecessors,1} = thisPMat;
            
            if thisErr < bestErr
                % we have found a better parent
                
                bestErr = newErr;
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
        % newNode.travFromStart = localTrav;
        newNode.err = newErr;
        newNode.PMat = newPMat;
        
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
            newNode.errFromPredecessors = [newNode.errFromPredecessors ; errFromPredecessor(uniqueIndsInds,1)];  
            newNode.PMatFromPredecessors = [newNode.PMatFromPredecessors ; PMatFromPredecessor(uniqueIndsInds,1)];
            newNode.trajectoriesFromPredecessors = [newNode.trajectoriesFromPredecessors ; trajectoriesFromPredecessors(uniqueIndsInds,1)];     
            
            % update the indicies to reflect the new predecessors actually stored in the node's list 
            uniqueIndsInds = oldLength+1:length(newNode.predecessors);
            
            % now we need to see if any of the new nodes yield a better cost from start 
            bestPredecessorIndex = -1;
            bestErr = inf;

            for n_i = 1:length(newNode.predecessors)
                thisPredecessor = newNode.predecessors{n_i};
                thisErr = newNode.errFromPredecessors(n_i,1);
                
                if thisErr < bestErr
                    bestErr = thisErr;
                    bestPredecessorIndex = n_i;
                end
            end
            
            
            if  bestPredecessorIndex > 0
                % only update the goal's parent if it actually helps
                parentNode = newNode.predecessors{bestPredecessorIndex,1};  
              
                newNode.lmc = bestErr;
                newNode.parentID = parentNode.id;
                newNode.parentNode = parentNode;
                newNode.parentTraj = newNode.trajectoriesFromPredecessors{bestPredecessorIndex,1};
                newNode.PMat = newNode.PMatFromPredecessors{bestPredecessorIndex,1};
              
            end
            
        else
            
            parentNode = predecessorNodes{bestPredecessorIndex};
            
            
            % for normal nodes, we just init these things as required
            newNode.lmc = newErr;
            newNode.parentID = parentNode.id;
            newNode.parentNode = parentNode;
            newNode.predecessors = predecessorNodes(1:numValidPredecessors);
            newNode.errFromPredecessors = errFromPredecessor(1:numValidPredecessors,1);  
            newNode.PMatFromPredecessors = PMatFromPredecessor(1:numValidPredecessors,1);
            newNode.trajectoriesFromPredecessors = trajectoriesFromPredecessors(1:numValidPredecessors,1);
            newNode.parentTraj = trajectoriesFromPredecessors{bestPredecessorIndex,1};
            newNode.PMat = PMatFromPredecessor{bestPredecessorIndex,1};
            
            uniqueIndsInds = 1:numValidPredecessors;

        end 
        

        % now we need to update the successor lists of all these predessors
        % to include the new node and trajectory
        
        for n_i_i = 1:length(uniqueIndsInds)
          n_i = uniqueIndsInds(n_i_i);
          predecessorNode = newNode.predecessors{n_i};
           
          numValidSuccessors = length(predecessorNode.neighbors) + 1;
          
          
          % this is a place for potential speed improvement, since right
          % now we're growing these on the fly without any explicit
          % prealocation of memory
          predecessorNode.neighbors{numValidSuccessors,1} = newNode; 
          predecessorNode.errToNeighbors(numValidSuccessors,1) = newNode.errFromPredecessors(n_i,1);
          predecessorNode.trajectoriesToNeighbors{numValidSuccessors,1} = newNode.trajectoriesFromPredecessors{n_i,1};
          predecessorNode.PMatToNeighbors{numValidSuccessors,1} = newNode.PMatFromPredecessors{n_i,1};
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

             
            if newNode.err < neighborNode.err
                % the above check is a heuristic check, if we are here than
                % the neighbor node MAY do better to use the new node as 
                % its parent, but now we need to do the harder actual 2pbvp 
                % check to make sure
              
              
                [neighborLocalTrajectory,neighborPMat] = steer(newNode.position', neighborNode.position', delta, W, Tol, newNode.PMat);
                if isempty(neighborLocalTrajectory)
                   % warning('could not connect to neighbor')
                  continue
                end
 
                tolCheck = norm(neighborLocalTrajectory(end,:)-neighborNode.position);
                % thisTrav = trajectoryLength(neighborLocalTrajectory);
                thisErr = ErrFun(neighborPMat);
                if thisErr <= neighborNode.err && tolCheck <= Tol
                  % the new node is a better parent to the neighbor than it
                  % current parent
                  
                  neighborNode.parentID = newNode.id;
                  neighborNode.parentNode = newNode;
                  neighborNode.parentTraj = neighborLocalTrajectory;
                 % neighborNode.travFromStart = thisTrav + newNode.travFromStart;
                  neighborNode.err = thisErr;
                  neighborNode.PMat = neighborPMat;
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
            errToSuccessor = inf(numSuccessors,1);
            PMatToSuccessor = cell(numSuccessors,1);
            numValidSuccessors = 0;  % incriments up as neighbors are added
            
            
            for n_i = 1:length(kNN)
            
                neighborNode = kNN{n_i}.payload;
             
                if neighborNode.id == newNode.id
                    % we do not want to rewire the new node to itself
                    % (though, under normal circumstance I do not think
                    % this case actually happens)
                    continue 
                end
          
                [neighborLocalTrajectory,neighborPMat] = steer(newNode.position' , neighborNode.position', delta,W,Tol,newNode.PMat);
                if isempty(neighborLocalTrajectory)
                   % warning('could not connect to neighbor')

                   continue
                end
                tolCheck = norm(neighborLocalTrajectory(end,:)-neighborNode.position);
                if tolCheck < Tol
                    thisErr = ErrFun(neighborPMat);
                    thisPMat = neighborPMat;
                    % thisTrav = trajectoryLength(neighborLocalTrajectory);


                    numValidPredecessors = length(neighborNode.predecessors) + 1;
                    neighborNode.predecessors{numValidPredecessors,1} = neighborNode; 
                    neighborNode.errFromPredecessors(numValidPredecessors,1) = thisErr;
                    neighborNode.PMatFromPredecessors{numValidPredecessors,1} = thisPMat;
                    neighborNode.trajectoriesFromPredecessors{numValidPredecessors,1} = neighborLocalTrajectory;


                    numValidSuccessors = numValidSuccessors + 1;
                    sucessorNodes{numValidSuccessors,1} = neighborNode;
                    trajectoriesToSuccessors{numValidSuccessors,1} = neighborLocalTrajectory;
                    errToSuccessor(numValidSuccessors,1) = thisErr;
                    PMatToSuccessor{numValidSuccessors,1} = thisPMat;
                end
            end  
            
            
            % now copy over the successors into the neighbor spots
            newNode.neighbors = sucessorNodes(1:numValidSuccessors,1);
            newNode.errToNeighbors = errToSuccessor(1:numValidSuccessors,1);
            newNode.PMatToNeighbors = PMatToSuccessor(1:numValidSuccessors,1);
            newNode.trajectoriesToNeighbors = trajectoriesToSuccessors(1:numValidSuccessors,1);
            
        end   
        
        % now add the newly created node to the queue (or update its position if
        % if it is the goal node and already in there... update() handles 
        % the case the node is already in the queue automatically)
        update(Q, newNode, min(newNode.err,newNode.lmc));
        
        % now we do the replan (i.e. the make consistant) step
        while(~isempty(topKey(Q)) )   % && topKey(Q) <= min(G.goalNode.costFromStart, G.goalNode.lmc) )
          % while the heap is nonempty and the top nodes is at a lower level set than the goal
        
         
          thisNode = pop(Q);                        % get top node
          thisNode.err = thisNode.lmc;    % set cost to lmc
          
          % now we need to see if any of the nodes that can be accessed
          % from this node would like to use this node as their parent
          
          for n_i = 1:length(thisNode.neighbors)
              successorNode = thisNode.neighbors{n_i};
              if successorNode.id == thisNode.parentNode.id  
                  continue  % explicitly avoid parent loops (in cases of numerical rounding error this could be an issue) 
              elseif successorNode.id == G.startNode.id
                  continue  % by definition the start node cannot be improved
              end
              
              errFromStartViaNewNode = thisNode.err;
              if successorNode.lmc > errFromStartViaNewNode
                  successorNode.lmc = errFromStartViaNewNode;
                  successorNode.parentID = thisNode.id;
                  successorNode.parentNode = thisNode;
                  successorNode.parentTraj = thisNode.trajectoriesToNeighbors{n_i,1};
                  successorNode.PMat = thisNode.PMatToNeighbors{n_i,1};
              end
              
              if successorNode.err == successorNode.lmc
                remove(Q, successorNode);
              else
                update(Q, successorNode, min(successorNode.err,successorNode.lmc));
              end            
          end
          
          
          
          
        end
        
    end
    
    
    if algorithmToUse == 0
        disp(num2str(G.n))
    end
    
    
    
    % ------------------- plot stuff -----------------
    if problemDefinition == 1
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
    elseif problemDefinition == 2
            fig = figure(1);
            fig.Units = 'normalized';
            fig.Position = [0 0 1 1];
        %         G.plotEdges('Color',[0.7,0.7,0.7])  % draws edges gray
        %         hold on
        G.calculateParentEdgesForPlotting()     % note that this takes a bit of time, only use for demo
        G.plotParentEdges('r', 'LineWidth', 2)
        
        axis(ax_bds)
        axis equal
        hold on
        
        %         G.plotNodesWithInListVal(0, '.k', 'MarkerSize', 3)  % unvisited
        %         G.plotNodesWithInListVal(1, 'or', 'MarkerSize', 3)  % open
        %         G.plotNodesWithInListVal(2, 'ok', 'MarkerSize', 3)  % closed
        
        G.plotStartNode2DPosition('ob', 'MarkerSize', 10, 'LineWidth', 3)
        G.plotGoalNode2DPosition( 'xb', 'MarkerSize', 10, 'LineWidth', 3)
        
        corners = [-141.421356237310, 639.736528676648;
                    0, 781.157884913957;
                    141.421356237310, 639.736528676648;
                    0, 498.315172439338;
                    -141.421356237310, 639.736528676648];
        plot(corners(:,1),corners(:,2),'--k','linewidth',2)
                    
        hold off
        axis equal
        axis(ax_bds)
        pause(.000)
    end
    % ------------------- end plot stuff -----------------
    pause(0.0)
    %}
    tcur=toc(tstart);
end


%%
% Extract Path
G.extractMotionPath()

if savePath
    csvwrite(pathFileName,G.motionPath')
end

calculatePlottingData(G)
trajForPlotting = getParentTrajectoriesForPlotting(G,numDims);
bestPathTrajForPlotting = getBestPathTrajectoriesForPlotting(G,numDims);


% ------------------- plot stuff -----------------

fig = figure(2);
fig.Units = 'normalized';
fig.Position = [0 0 1 1];
if numDims > 2
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
else
    G.plotEdges('Color',[0.7,0.7,0.7])  % draws edges gray
    hold on
    G.calculateParentEdgesForPlotting()
    
    G.plotParentEdges('r','LineWidth', 2)
    
    G.plotNodesWithInListVal(0, '.k', 'MarkerSize', 3)  % unvisited
    G.plotNodesWithInListVal(1, 'or', 'MarkerSize', 3)  % open
    G.plotNodesWithInListVal(2, 'ok', 'MarkerSize', 3)  % closed
    
    %G.plotMotionPath('b',  'LineWidth', 4)
    
    G.plotStartNode('ob', 'MarkerSize', 10, 'LineWidth', 3)
    G.plotGoalNode( 'xb', 'MarkerSize', 10, 'LineWidth', 3)
    grid on
    hold off
    axis equal
    axis(ax_bds(1:4))
    
    xlabel('X')
    ylabel('Y')
    zlabel('Z')
    title('Posistion')
end


% now plot just the search tree
fig = figure(3);
fig.Units = 'normalized';
fig.Position = [0 0 1 1];
if numDims > 2
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
else
    G.plotParentEdges('r', 'LineWidth', 2)
    
    hold on

    G.plotMotionPath('b',  'LineWidth', 4)

    G.plotStartNode('ob', 'MarkerSize', 10, 'LineWidth', 3)
    G.plotGoalNode( 'xb', 'MarkerSize', 10, 'LineWidth', 3)
    grid on
    hold off
    axis equal
    axis(ax_bds(1:4))
    xlabel('X')
    ylabel('Y')
    zlabel('Z')
    title('Posistion')
end 
%{ 
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
%}
fig = figure(4);
fig.Units = 'normalized';
fig.Position = [0 0 1 1];
if numDims > 2
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
else
    plot(trajForPlotting(:,1), trajForPlotting(:,2),'m', 'LineWidth', 2)
    axis(ax_bds(1:4))
    axis equal
    hold on
    plot(bestPathTrajForPlotting(:,1), bestPathTrajForPlotting(:,2), 'b', 'LineWidth', 2)
    G.plotStartNode('ok', 'MarkerSize', 10, 'LineWidth', 1)
    G.plotGoalNode( 'xk', 'MarkerSize', 10, 'LineWidth', 1)
    grid on
    hold off;
    axis equal
    xlabel('X')
    ylabel('Y')
    zlabel('Z')
    title('Posistion')
end
%{
filename = pwd;
if algorithmToUse == 0
    filename = strcat(filename,'\RRT\','W2ChosenPathErrBall.png');
elseif algorithmToUse == 1
    filename = strcat(filename,'\RRTstar\','W2ChosenPathErrBall.png');
elseif algorithmToUse == 2
    filename = strcat(filename,'\RRTSharp\','W2ChosenPathErrBall.png');
end
saveas(fig,filename);
%}
% Plot Cost from Start along the Path:
fig = figure(5);
if numDims > 2
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
else
    j = 0;
    for i = size(G.motionPath,2):-1:1
        j=j+1;
        xAx(j) = j;
        yAx1(j) = G.nodes{G.motionPath(i)}.err;
    end
    plot(xAx,yAx1);
    hold on
    plot(xAx,yAx2);
    hold off
    xlim([0, max(xAx)]);
    xlabel('Node on Path');
    ylabel('Error');
    title('Error Growth along Path');
    legend('X Error','Y Error')

end
    
    
%{
filename = pwd;
if algorithmToUse == 0
    filename = strcat(filename,'\RRT\','W2ErrorAlongPath.png');
elseif algorithmToUse == 1
    filename = strcat(filename,'\RRTstar\','W2ErrorAlongPath.png');
elseif algorithmToUse == 2
    filename = strcat(filename,'\RRTSharp\','W2ErrorAlongPath.png');
end
saveas(fig,filename);
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
%}
%% Run This after running all 3 algorithms:
ErrorComparison();