function SaveParaData(G,QMat)

% Create or Add to existing Parameters File:
if problemDefinition == 1
    fileloc = [pwd,'\Data\SP\'];
    filename = [fileloc,'Parameters.txt'];
    % Check if Parameters File exists:
    if isfile(filename)
        % If it exists, append to the end of it
        FID = fopen(filename,'a+');
        fprintf(FID,'Trail #%i\n',TN);
        fprintf(FID,'%2.4f,%2.4f,%2.4f,%2.4f,%2.4f,%2.4f\n',clock);
        fprintf(FID,'%2.4f,%2.4f,%2.4f,%2.4f,%2.4f,%2.4f\n',startPosition);
        fprintf(FID,'%2.4f,%2.4f,%2.4f,%2.4f,%2.4f,%2.4f\n',goalPosition);
        dlmwrite(filename,G.startNode.PMat,'-append');
        dlmwrite(filename,QMat,'-append');
    else
        % If it doesnt exists, create it
        FID = fopen(filename,'w');
        fprintf(FID,'Trail #%i\n',TN);
        fprintf(FID,'%2.4f,%2.4f,%2.4f,%2.4f,%2.4f,%2.4f\n',clock);
        fprintf(FID,'%2.4f,%2.4f,%2.4f,%2.4f,%2.4f,%2.4f\n',startPosition);
        fprintf(FID,'%2.4f,%2.4f,%2.4f,%2.4f,%2.4f,%2.4f\n',goalPosition);
        dlmwrite(filename,G.startNode.PMat,'-append');
        dlmwrite(filename,QMat,'-append');
    end
else
    fileloc = [pwd,'\Data\2D\'];
    filename = [fileloc,'Parameters.txt'];
    % Check if Parameters File exists:
    if isfile(filename)
        % If it exists, append to the end of it
        FID = fopen(filename,'a+');
        fprintf(FID,'Trail #%i\n',TN);
        fprintf(FID,'%2.4f,%2.4f,%2.4f,%2.4f,%2.4f,%2.4f\n',clock);
        fprintf(FID,'%2.4f,%2.4f\n',startPosition);
        fprintf(FID,'%2.4f,%2.4f\n',goalPosition);
        dlmwrite(filename,G.startNode.PMat,'-append');
        dlmwrite(filename,QMat,'-append');
        % If it doesnt exists, create it
    else
        FID = fopen(filename,'w');
        fprintf(FID,'Trail #%i\n',TN);
        fprintf(FID,'%2.4f,%2.4f,%2.4f,%2.4f,%2.4f,%2.4f\n',clock);
        fprintf(FID,'%2.4f,%2.4f\n',startPosition);
        fprintf(FID,'%2.4f,%2.4f\n',goalPosition);
        dlmwrite(filename,G.startNode.PMat,'-append');
        dlmwrite(filename,QMat,'-append');
    end
end
fclose(FID);

end


