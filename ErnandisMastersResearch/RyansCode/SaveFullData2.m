function SaveFullData2(G,TN,problemDefinition,fileloc)
% Full Data Save Function, indexed by node number:
% First Save Nodes by ID
if problemDefinition == 1
    %fileloc = [pwd,'\Data\SP\'];
    filename = [fileloc,date,'Trial',num2str(TN),'nodes.txt'];
    FID = fopen(filename,'w');
    for i = 1:G.n
        fprintf(FID,'%i\n', G.nodes{i,1}.id);
        fprintf(FID,'%2.4f,%2.4f,%2.4f,%2.4f,%2.4f,%2.4f\n',G.nodes{i,1}.position);
        fprintf(FID,'%i\n',G.nodes{i,1}.parentID);
        fprintf(FID,'%2.8f\n',G.nodes{i,1}.err);
    end
else
    %fileloc = [pwd,'\Data\2D\'];
    filename = [fileloc,date,'Trial',num2str(TN),'nodes.txt'];
    
    FID = fopen(filename,'w');
    for i = 1:G.n
        fprintf(FID,'%i\n', G.nodes{i,1}.id);
        fprintf(FID,'%2.4f,%2.4f\n',G.nodes{i,1}.position);
        fprintf(FID,'%i\n',G.nodes{i,1}.parentID);
        fprintf(FID,'%2.4f\n',G.nodes{i,1}.err);
    end
end
fclose(FID);

% Second, Save all the Edges based on node ID:
% First Node is the starting node, second node is the ending node:
Edges = NaN(G.n*2,2);
k = 1;
for i = 1:G.n
    thisNode = G.nodes{i};
    for j = 1:length(thisNode.neighbors)
        neighborNode = thisNode.neighbors{j};
        if thisNode.id < neighborNode.id
            Edges(k,1) = thisNode.id;
            Edges(k,2) = neighborNode.id;
            k = k+1;
        end
    end
end
if problemDefinition == 1
    %fileloc = [pwd,'\Data\SP\'];
    filename = [fileloc,date,'Trial',num2str(TN),'edges.txt'];
    FID = fopen(filename,'w');
    for i = 1:size(Edges,1)
        fprintf(FID,'%i\t%i\n',Edges(i,1),Edges(i,2));
    end
else
    %fileloc = [pwd,'\Data\2D\'];
    filename = [fileloc,date,'Trial',num2str(TN),'edges.txt'];
    FID = fopen(filename,'w');
    for i = 1:G.n
        fprintf(FID,'%i\t%i\n',Edges(i,1),Edges(i,2));
    end
end
fclose(FID);
end