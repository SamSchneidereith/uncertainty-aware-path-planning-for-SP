% Read in Data:
clear variables;
Direct = [pwd,'\Plots\SP'];

%% Load Data for Ellipse:
% Ellipse cases 1-4:
clear Data.C;
Date = '11-Aug-2020';
ErrFun = 'Ellipse';
for CaseNum = 1:4
    Fileloc = [Direct,'\',Date,'\NewPositions\',ErrFun,'\',num2str(CaseNum),'\'];
    for TN = 1:5
        FileName = [Date,'Trial',num2str(TN),'nodes.txt'];
        File = strcat(Fileloc,FileName);
        formatspec = '%f %f,%f,%f,%f,%f,%f %f %f';
        FID = fopen(File);
        Data(CaseNum).C(TN,:) = textscan(FID,formatspec,'Delimiter','\r\n');
        fclose(FID);
    end % TN
end % CaseNum

% Ellipse cases 5-10:
Date = '12-Aug-2020';
for CaseNum = 5:10
    Fileloc = [Direct,'\',Date,'\NewPositions\',ErrFun,'\',num2str(CaseNum),'\'];
    for TN = 1:5
        FileName = [Date,'Trial',num2str(TN),'nodes.txt'];
        File = strcat(Fileloc,FileName);
        formatspec = '%f %f,%f,%f,%f,%f,%f %f %f';
        FID = fopen(File);
        Data(CaseNum).C(TN,:) = textscan(FID,formatspec,'Delimiter','\r\n');
        fclose(FID);
    end % TN
end % CaseNum

% Get Important Data:
% Ending Error:
EllErrs = zeros(5,10);
set(0,'DefaultFigureVisible','on');
% Go through each Case:
for i = 1:10
    for j = 1:5
        EllErrs(j,i) = Data(i).C{j,9}(2);
    end
end

EllAccu = zeros(5,10);
for i = 1:10
    for j = 1:5
        EllAccu(j,i) = Data(i).C{j,9}(2)-Data(i).C{j,9}(1);
    end
end

%Box Plot of Errors:
fig = figure;
fig.Units ='inches';
fig.Position = [0 0 6 4];
boxplot(EllAccu);
xlabel('Case');
ylabel('Accumulated Error');
title('Error Accumulation Range per Case');
set(gca,'FontSize',12,'LineWidth',1)

%% Load Data for Trace:

% Trace cases 1-4:
clear Data2.C;
Date = '12-Aug-2020';
ErrFun = 'Trace';
for CaseNum = 1:4
    Fileloc = [Direct,'\',Date,'\NewPositions\',ErrFun,'\',num2str(CaseNum),'\'];
    if CaseNum == 4
        for TN = 1:2
            FileName = [Date,'Trial',num2str(TN),'nodes.txt'];
            File = strcat(Fileloc,FileName);
            formatspec = '%f %f,%f,%f,%f,%f,%f %f %f';
            FID = fopen(File);
            Data2(CaseNum).C(TN,:) = textscan(FID,formatspec,'Delimiter','\r\n');
            fclose(FID);
        end % TN        
    else
        for TN = 1:5
            FileName = [Date,'Trial',num2str(TN),'nodes.txt'];
            File = strcat(Fileloc,FileName);
            formatspec = '%f %f,%f,%f,%f,%f,%f %f %f';
            FID = fopen(File);
            Data2(CaseNum).C(TN,:) = textscan(FID,formatspec,'Delimiter','\r\n');
            fclose(FID);
        end % TN
    end
end % CaseNum

% Trace cases 4-10:
Date = '13-Aug-2020';
for CaseNum = 4:10
    Fileloc = [Direct,'\',Date,'\NewPositions\',ErrFun,'\',num2str(CaseNum),'\'];
    if CaseNum == 4
        for TN = 3:5
            FileName = [Date,'Trial',num2str(TN),'nodes.txt'];
            File = strcat(Fileloc,FileName);
            formatspec = '%f %f,%f,%f,%f,%f,%f %f %f';
            FID = fopen(File);
            Data2(CaseNum).C(TN,:) = textscan(FID,formatspec,'Delimiter','\r\n');
            fclose(FID);
        end % TN
    else
        for TN = 1:5
            FileName = [Date,'Trial',num2str(TN),'nodes.txt'];
            File = strcat(Fileloc,FileName);
            formatspec = '%f %f,%f,%f,%f,%f,%f %f %f';
            FID = fopen(File);
            Data2(CaseNum).C(TN,:) = textscan(FID,formatspec,'Delimiter','\r\n');
            fclose(FID);
        end % TN
    end
end % CaseNum

% Get Important Data:
% Ending Error:
TraErrs = zeros(5,10);
set(0,'DefaultFigureVisible','on');
% Go through each Case:
for i = 1:10
    for j = 1:5
        TraErrs(j,i) = Data2(i).C{j,9}(2);
    end
end

TraAccu = zeros(5,10);
for i = 1:10
    for j = 1:5
        TraAccu(j,i) = Data2(i).C{j,9}(2)-Data2(i).C{j,9}(1);
    end
end

%Box Plot of Errors:
fig = figure;
fig.Units ='inches';
fig.Position = [0 0 6 4];
boxplot(TraAccu);
xlabel('Case');
ylabel('Accumulated Error');
title('Error Accumulation Range per Case');
set(gca,'FontSize',12,'LineWidth',1)