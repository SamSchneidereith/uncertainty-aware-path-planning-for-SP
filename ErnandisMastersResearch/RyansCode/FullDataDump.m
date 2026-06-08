function FullDataDump(Data,fileloc)
% Dump all the data from all the Trials:
% Print the Paths
for TN = 1:size(Data,2)
    filename = [fileloc,date,'FullData',num2str(TN),'Paths.txt'];
    FID = fopen(filename,'w');
    for j = 1:size(Data(TN).Path,2)
        for i = 1:size(Data(TN).Path(j).IT,2)
            fprintf(FID,'%f\t',Data(TN).Path(j).IT(i));
        end
        fprintf(FID,'\n');
    end
    fclose(FID);
end
% Print the Error of each node
for TN = 1:size(Data,2)
    filename = [fileloc,date,'FullData',num2str(TN),'Errors.txt'];
    FID = fopen(filename,'w');
    for j = 1:size(Data(TN).Err,2)
        for i = 1:size(Data(TN).Err(j).IT,1)
            fprintf(FID,'%f\t',Data(TN).Err(j).IT(i));
        end
        fprintf(FID,'\r\n');
    end
    fclose(FID);
end
% Print the Time and number of Nodes
for TN = 1:size(Data,2)
    filename = [fileloc,date,'FullData',num2str(TN),'Time&Nodes.txt'];
    FID = fopen(filename,'w');
    for j = 1:size(Data(TN).Time,1)
        fprintf(FID,'%f\t %f\n',Data(TN).Time(j),Data(TN).NumNodes(j));
    end
    fclose(FID);
end

end