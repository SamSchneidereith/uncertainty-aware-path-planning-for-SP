clear variables;
% Data Comparison for 2D Case:
% Array: [Start Error; Goal Error; Changing Value (0=P,1=Q,2=R); Multiplier
% (0.1,1,10), Trial # (1,2,3), Num Nodes in the graph]
Data2DTrace = [
    % 0.1P Trials
    0.2000, 1.1385, 0, 0.1, 1, 7127;
    0.2000, 1.0342, 0, 0.1, 2, 7121;
    0.2000, 1.1385, 0, 0.1, 3, 7152;
    % 1P Trials
    2.0000, 2.9385, 0, 1, 1, 7077;
    2.0000, 2.9385, 0, 1, 2, 7044;
    2.0000, 2.9385, 0, 1, 3, 7030;
    % 10P Trials
    20.0000, 20.9385, 0, 10, 1, 6972;
    20.0000, 21.0428, 0, 10, 2, 6958;
    20.0000, 20.9385, 0, 10, 3, 7058;
    %0.1Q Trials
    2.0000, 2.4559, 1, 0.1, 1, 7079;
    2.0000, 2.4103, 1, 0.1, 2, 7097;
    2.0000, 2.3647, 1, 0.1, 3, 7141;
    %1Q Trials
    2.0000, 2.9385, 1, 1, 1, 7402;
    2.0000, 2.9385, 1, 1, 2, 7188;
    2.0000, 2.9385, 1, 1, 3, 7089;
    %10Q Trials
    2.0000, 3.3909, 1, 10, 1, 6966;
    2.0000, 3.3909, 1, 10, 2, 6922;
    2.0000, 3.3909, 1, 10, 3, 6968;
    %0.1R Trials
    2.0000, 2.1391, 2, 0.1, 1, 6565;
    2.0000, 2.1236, 2, 0.1, 2, 6956;
    2.0000, 2.1391, 2, 0.1, 3, 6567;
    %1R Trials
    2.0000, 2.9385, 2, 1, 1, 6887;
    2.0000, 3.0428, 2, 1, 2, 7108;
    2.0000, 2.9385, 2, 1, 3, 7097;
    %10R Trials
    2.0000, 6.5587, 2, 10, 1, 7627;
    2.0000, 6.5587, 2, 10, 2, 7532;
    2.0000, 6.5587, 2, 10, 3, 7568;
];
Data2DVE = [
     % 0.1P Trials
    0.3142, 1.7884, 0, 0.1, 1, 7209;
    0.3142, 1.7884, 0, 0.1, 2, 7318;
    0.3142, 1.7884, 0, 0.1, 3, 7246;
    % 1P Trials
    3.1416, 4.7796, 0, 1, 1, 7415;
    3.1416, 4.7796, 0, 1, 2, 7175;
    3.1416, 4.7796, 0, 1, 3, 7110;
    % 10P Trials
    31.4159, 32.8901, 0, 10, 1, 7217;
    31.4159, 33.0539, 0, 10, 2, 7243;
    31.4159, 33.0539, 0, 10, 3, 7233;
    %0.1Q Trials
    3.1416, 3.7861, 1, 0.1, 1, 7484;
    3.1416, 3.8577, 1, 0.1, 2, 7578;
    3.1416, 3.7145, 1, 0.1, 3, 7370;
    %1Q Trials
    3.1416, 4.6158, 1, 1, 1, 7174;
    3.1416, 4.6158, 1, 1, 2, 7356;
    3.1416, 4.6158, 1, 1, 3, 7429;
    %10Q Trials
    3.1416, 5.3265, 1, 10, 1, 7325;
    3.1416, 5.3265, 1, 10, 2, 7388;
    3.1416, 5.3265, 1, 10, 3, 7352;
    %0.1R Trials
    3.1416, 3.3601, 2, 0.1, 1, 7259;
    3.1416, 3.2873, 2, 0.1, 2, 7227;
    3.1416, 3.3601, 2, 0.1, 3, 6974;
    %1R Trials
    3.1416, 4.7796, 2, 1, 1, 7062;
    3.1416, 4.6158, 2, 1, 2, 7159;
    3.1416, 4.6158, 2, 1, 3, 7484;
    %10R Trials
    3.1416, 10.3023, 2, 10, 1, 7527;
    3.1416, 10.3023, 2, 10, 2, 7526;
    3.1416, 10.3023, 2, 10, 3, 7607;
];

DataRRTStar2D = [
    %0.1R Trace
    2.0000, 2.1391, 2, 0.1, 1, 12876;
    2.0000, 2.1391, 2, 0.1, 2, 12463;
    2.0000, 2.1391, 2, 0.1, 3, 12632;
    %1R Trace
    2.0000, 3.0428, 2, 1, 1, 12339;
    2.0000, 2.9385, 2, 1, 2, 12249;
    2.0000, 2.9385, 2, 1, 3, 12461;    
    %10R Trace
    2.0000, 6.5587, 2, 10, 1, 12492;
    2.0000, 7.4704, 2, 10, 2, 12558;
    2.0000, 6.5587, 2, 10, 3, 12738;    
    %0.1R VE
    3.1416, 3.3601, 2, 0.1, 1, 12793;
    3.1416, 3.3844, 2, 0.1, 2, 12566;
    3.1416, 3.3601, 2, 0.1, 3, 12793;
    %1R VE
    3.1416, 4.7796, 2, 0.1, 1, 12681;
    3.1416, 4.9434, 2, 0.1, 2, 12726;
    3.1416, 4.7796, 2, 0.1, 3, 12476;    
    %10R VE
    3.1416, 9.5863, 2, 0.1, 1, 12904;
    3.1416, 10.3023, 2, 0.1, 2, 12641;
    3.1416, 10.3023, 2, 0.1, 3, 12480;
    ];


DataSPVE = [
    %0.1P Trials
    0.00516771, 0.00516779, 0, 0.1, 1, 6561;
    0.00516771, 0.00516781, 0, 0.1, 2, 6657;
    0.00516771, 0.00516777, 0, 0.1, 3, 6881;
    %1P Trials
    5.16771278, 5.16771287, 0, 1, 1, 6192;
    5.16771278, 5.16771282, 0, 1, 2, 6273;
    5.16771278, 5.16771284, 0, 1, 3, 6273;
    %10P Trials
    5167.71278005, 5167.71278011, 0, 10, 1, 6323;
    5167.71278005, 5167.71278012, 0, 10, 2, 6243;
    5167.71278005, 5167.71278011, 0, 10, 3, 6362;
    %0.1Q Trials
    5.16771278, 5.16771278, 2, 0.1, 1, 7128;
    5.16771278, 5.16771278, 2, 0.1, 2, 7164;
    5.16771278, 5.16771278, 2, 0.1, 3, 7064;
    %1Q Trials
    5.16771278, 5.16771284, 2, 1, 1, 6634;
    5.16771278, 5.16771282, 2, 1, 2, 6655;
    5.16771278, 5.16771282, 2, 1, 3, 6579;
    %10Q Trials
    5.16771278, 5.16771539, 2, 10, 1, 3197;
    5.16771278, 5.16771509, 2, 10, 2, 2745;
    5.16771278, 5.16771636, 2, 10, 3, 2891;
    %0.1R Trials
    5.16771278, 5.16771278, 2, 0.1, 1, 6888;
    5.16771278, 5.16771278, 2, 0.1, 2, 6830;
    5.16771278, 5.16771278, 2, 0.1, 3, 6859;
    %1R Trials
    5.16771278, 5.16771289, 2, 1, 1, 6738;
    5.16771278, 5.16771283, 2, 1, 2, 6544;
    5.16771278, 5.16771282, 2, 1, 3, 6564;
    %10R Trials
    5.16771278, 5.16771454, 2, 10, 1, 3232;
    5.16771278, 5.16771653, 2, 10, 2, 3377;
    5.16771278, 5.16771420, 2, 10, 3, 3443;
];
    
    
DataSPTrace = [
    %0.1P Trials
    0.60000000, 59.57375393, 0, 0.1, 1, 6884;
    0.60000000, 30.08748571, 0, 0.1, 2, 5958;
    0.60000000, 30.08625722, 0, 0.1, 3, 6076;
    %1P Trials
    6.00000000, 59.07195131, 0, 1, 1, 6332;
    6.00000000, 29.58839313, 0, 1, 2, 6923;
    6.00000000, 41.38689953, 0, 1, 3, 6812;
    %10P Trials
    60.00000000, 107.17051893, 0, 10, 1, 6567;
    60.00000000, 107.18974513, 0, 10, 2, 5945;
    60.00000000, 101.28363571, 0, 10, 3, 6368;
    %0.1Q Trials\$
    6, 9.59309202, 1, 0.1, 1, 6968;
    6, 10.19194611, 1, 0.1, 2, 6791;
    6, 10.19197571, 1, 0.1, 3, 6795;
    %1Q Trials
    6, 29.58848023, 1, 1, 1, 6372;
    6, 53.16677817, 1, 1, 2, 6457;
    6, 47.27592670, 1, 1, 3, 6772;
    %10Q Trials
    6, 535.94725510, 1, 10, 1, 2985;
    6, 535.50387664, 1, 10, 2, 3033;
    6, 323.72122193, 1, 10, 3, 3171;
    %0.1R Trials
    6, 58.84020778, 2, 0.1, 1, 6806;
    6, 27.17093264, 2, 0.1, 2, 6834;
    6, 27.17084533, 2, 0.1, 3, 6775;
    %1R Trials
    6, 64.94912157, 2, 1, 1, 6346;
    6, 29.58831367, 2, 1, 2, 6471;
    6, 59.07340663, 2, 1, 3, 6346;
    %10R Trials
    6, 35.94316396, 2, 10, 1, 3277;
    6, 35.94280680, 2, 10, 2, 3331;
    6, 53.90752334, 2, 10, 3, 3598;
    ];

DataRRTStarSP = [
    %0.1R Trace
    6.0000, 37.73856256, 2, 0.1, 1, 10208;
    6.0000, 43.04292580, 2, 0.1, 2, 10742;
    6.0000, 37.77721087, 2, 0.1, 3, 10812;
    %1R Trace
    6.0000, 41.38051469, 2, 1, 1, 9807;
    6.0000, 47.27354147, 2, 1, 2, 9941;
    6.0000, 29.58691397, 2, 1, 3, 9907;    
    %10R Trace
    6.0000, 35.94200323, 2, 10, 1, 5156;
    6.0000, 71.87241595, 2, 10, 2, 4533;
    6.0000, 41.93095068, 2, 10, 3, 5167;    
    %0.1R VE
    5.16771278, 5.16771278, 2, 0.1, 1, 10287;
    5.16771278, 5.16771278, 2, 0.1, 2, 10219;
    5.16771278, 5.16771278, 2, 0.1, 3, 10431;
    %1R VE
    5.16771278, 5.16771283, 2, 0.1, 1, 9946;
    5.16771278, 5.16771288, 2, 0.1, 2, 9697;
    5.16771278, 5.16771283, 2, 0.1, 3, 9946;    
    %10R VE
    5.16771278, 5.16771543, 2, 0.1, 1, 4645;
    5.16771278, 5.16771520, 2, 0.1, 2, 5380;
    5.16771278, 5.16771543, 2, 0.1, 3, 4645;
    ];

%% 2D Error Comparison, Start and Finish:
Trace2D.DelErr = Data2DTrace(:,2)-Data2DTrace(:,1);
Trace2D.AvgPDel = mean(Trace2D.DelErr(1:9));
Trace2D.AvgQDel = mean(Trace2D.DelErr(10:18));
Trace2D.AvgRDel = mean(Trace2D.DelErr(19:27));
Trace2D.AvgDel = mean(Trace2D.DelErr);

Trace2D.AvgPEnd = mean(Data2DTrace(1:9,2));
Trace2D.AvgQEnd = mean(Data2DTrace(10:18,2));
Trace2D.AvgREnd = mean(Data2DTrace(19:27,2));
Trace2D.AvgEnd = mean(Data2DTrace(:,2));

% Error Comparison, Start and Finish:
VE2D.DelErr = Data2DVE(:,2)-Data2DVE(:,1);
VE2D.AvgPDel = mean(VE2D.DelErr(1:9));
VE2D.AvgQDel = mean(VE2D.DelErr(10:18));
VE2D.AvgRDel = mean(VE2D.DelErr(19:27));
VE2D.AvgDel = mean(VE2D.DelErr);

VE2D.AvgPEnd = mean(Data2DVE(1:9,2));
VE2D.AvgQEnd = mean(Data2DVE(10:18,2));
VE2D.AvgREnd = mean(Data2DVE(19:27,2));
VE2D.AvgEnd = mean(Data2DVE(:,2));



fig = figure;
fig.Units = 'inches';
fig.Position = [0 0 6 4];
j = 1;
EndErrorTrace = zeros(3,8);
for i = 1:3:25
    EndErrorTrace(:,j) = Data2DTrace(i:i+2,2);
    j = j+1;
end
EndErrorTrace2 = [EndErrorTrace(:,1:2),EndErrorTrace(:,4:9)];
boxplot(EndErrorTrace2,'Labels',{'0.1*P','1*P','0.1*Q','1*Q','10*Q','0.1*R','1*R','10*R'});
xlabel('Test Cases')
ylabel('Error')
title('Ending Error of Each Test')
set(gca,'FontSize',12,'LineWidth',1)

fig = figure;
fig.Units = 'inches';
fig.Position = [0 0 6 4];
j = 1;

boxplot(EndErrorTrace(:,3),'Labels',{'10*P'});
xlabel('Test Cases')
ylabel('Error')
title('Ending Error of Each Test')
set(gca,'FontSize',12,'LineWidth',1)
%
%%
fig = figure;
fig.Units = 'inches';
fig.Position = [0 0 6 4];
j = 1;
EndErrorVE = zeros(3,8);
for i = 1:3:25
    EndErrorVE(:,j) = Data2DVE(i:i+2,2);
    j = j+1;
end
EndErrorVE2=[EndErrorVE(:,1:2),EndErrorVE(:,4:9)];
boxplot(EndErrorVE2,'Labels',{'0.1*P','1*P','0.1*Q','1*Q','10*Q','0.1*R','1*R','10*R'});
xlabel('Test Cases')
ylabel('Error')
title('Ending Error of Each Test')
set(gca,'FontSize',12,'LineWidth',1)
fig = figure;
fig.Units = 'inches';
fig.Position = [0 0 6 4];
boxplot(EndErrorVE(:,3),'Labels',{'10*P'});
xlabel('Test Cases')
ylabel('Error')
title('Ending Error of Each Test')
set(gca,'FontSize',12,'LineWidth',1)

%% 2D Error Comparison, Start and Finish:
RRT2D.DelErr = DataRRTStar2D(:,2)-DataRRTStar2D(:,1);
RRT2D.AvgTRDel = mean(RRT2D.DelErr(1:9));
RRT2D.AvgVEDel = mean(RRT2D.DelErr(10:18));
RRT2D.AvgDel = mean(RRT2D.DelErr);

RRT2D.AvgTREnd = mean(DataRRTStar2D(1:9,2));
RRT2D.AvgVEEnd = mean(DataRRTStar2D(10:18,2));

%
fig = figure;
fig.Units = 'inches';
fig.Position = [0 0 6 4];
j = 1;
EndErrorRRT = zeros(3,6);
for i = 1:3:16
    EndErrorRRT(:,j) = DataRRTStar2D(i:i+2,2);
    j = j+1;
end
boxplot(EndErrorRRT,'Labels',{'0.1*R TR','1*R TR','10*R TR','0.1*R VE','1*R VE','10*R VE'});
xlabel('Test Cases')
ylabel('Error')
title('Ending Error of Each RRT^* Test')
set(gca,'FontSize',12,'LineWidth',1)

%%

RT = [mean(DataRRTStar2D(1:3,2)), mean(Data2DTrace(19:21,2));
mean(DataRRTStar2D(4:6,2)), mean(Data2DTrace(22:24,2));
mean(DataRRTStar2D(7:9,2)), mean(Data2DTrace(25:27,2))];
RV = [mean(DataRRTStar2D(10:12,2)), mean(Data2DVE(19:21,2));
mean(DataRRTStar2D(13:15,2)), mean(Data2DVE(22:24,2));
mean(DataRRTStar2D(16:18,2)), mean(Data2DVE(25:27,2))];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% SP Error Comparison, Start and Finish:
TraceSP.DelErr = DataSPTrace(:,2)-DataSPTrace(:,1);
TraceSP.AvgPDel = mean(TraceSP.DelErr(1:9));
TraceSP.AvgQDel = mean(TraceSP.DelErr(10:18));
TraceSP.AvgRDel = mean(TraceSP.DelErr(19:27));
TraceSP.AvgDel = mean(TraceSP.DelErr);

TraceSP.AvgPEnd = mean(DataSPTrace(1:9,2));
TraceSP.AvgQEnd = mean(DataSPTrace(10:18,2));
TraceSP.AvgREnd = mean(DataSPTrace(19:27,2));
TraceSP.AvgEnd = mean(DataSPTrace(:,2));

% Error Comparison, Start and Finish:
VESP.DelErr = DataSPVE(:,2)-DataSPVE(:,1);
VESP.AvgPDel = mean(VESP.DelErr(1:9));
VESP.AvgQDel = mean(VESP.DelErr(10:18));
VESP.AvgRDel = mean(VESP.DelErr(19:27));
VESP.AvgDel = mean(VESP.DelErr);

VESP.AvgPEnd = mean(DataSPVE(1:9,2));
VESP.AvgQEnd = mean(DataSPVE(10:18,2));
VESP.AvgREnd = mean(DataSPVE(19:27,2));
VESP.AvgEnd = mean(DataSPVE(:,2));


TRmean = [mean(DataSPTrace(1:3,6)), mean(DataSPTrace(4:6,6)), mean(DataSPTrace(7:9,6)), mean(DataSPTrace(1:9,6)); 
        mean(DataSPTrace(10:12,6)), mean(DataSPTrace(13:15,6)), mean(DataSPTrace(16:18,6)), mean(DataSPTrace(10:18,6)); 
        mean(DataSPTrace(19:21,6)), mean(DataSPTrace(22:24,6)), mean(DataSPTrace(25:27,6)), mean(DataSPTrace(19:27,6))];
VEmean = [mean(DataSPVE(1:3,6)), mean(DataSPVE(4:6,6)), mean(DataSPVE(7:9,6)), mean(DataSPVE(1:9,6)); 
        mean(DataSPVE(10:12,6)), mean(DataSPVE(13:15,6)), mean(DataSPVE(16:18,6)), mean(DataSPVE(10:18,6)); 
        mean(DataSPVE(19:21,6)), mean(DataSPVE(22:24,6)), mean(DataSPVE(25:27,6)), mean(DataSPVE(19:27,6))];
fig = figure;
fig.Units = 'inches';
fig.Position = [0 0 6 4];
j = 1;
EndErrorTrace = zeros(3,8);
for i = 1:3:25
    EndErrorTrace(:,j) = DataSPTrace(i:i+2,2);
    j = j+1;
end
EndErrorTrace2 = [EndErrorTrace(:,1:5),EndErrorTrace(:,7:9)];
boxplot(EndErrorTrace2,'Labels',{'0.1*P','1*P','10*P','0.1*Q','1*Q','0.1*R','1*R','10*R'});
xlabel('Test Cases')
ylabel('Error')
title('Ending Error of Each Test')
set(gca,'FontSize',12,'LineWidth',1)

fig = figure;
fig.Units = 'inches';
fig.Position = [0 0 6 4];
boxplot(EndErrorTrace(:,6),'Labels',{'10*Q'});
xlabel('Test Cases')
ylabel('Error')
title('Ending Error of Each Test')
set(gca,'FontSize',12,'LineWidth',1)
%%
fig = figure;
fig.Units = 'inches';
fig.Position = [0 0 6 4];

j = 1;
EndErrorVE = zeros(3,8);
for i = 1:3:25
    EndErrorVE(:,j) = DataSPVE(i:i+2,2);
    j = j+1;
end
EndErrorVE2 = [EndErrorVE(:,2),EndErrorVE(:,4:9)];
boxplot(EndErrorVE2,'Labels',{'1*P','0.1*Q','1*Q','10*Q','0.1*R','1*R','10*R'});
xlabel('Test Cases')
ylabel('Error')
title('Ending Error of Each Test')
set(gca,'FontSize',12,'LineWidth',1)

fig = figure;
fig.Units = 'inches';
fig.Position = [0 0 6 4];
boxplot(EndErrorVE(:,3),'Labels',{'10*P'});
xlabel('Test Cases')
ylabel('Error')
title('Ending Error of Each Test')
set(gca,'FontSize',12,'LineWidth',1)

fig = figure;
fig.Units = 'inches';
fig.Position = [0 0 6 4];
boxplot(EndErrorVE(:,1),'Labels',{'0.1*P'});
xlabel('Test Cases')
ylabel('Error')
title('Ending Error of Each Test')
set(gca,'FontSize',12,'LineWidth',1)
%% 2D Error Comparison, Start and Finish:
RRTSP.DelErr = DataRRTStarSP(:,2)-DataRRTStarSP(:,1);
RRTSP.AvgTRDel = mean(RRTSP.DelErr(1:9));
RRTSP.AvgVEDel = mean(RRTSP.DelErr(10:18));
RRTSP.AvgDel = mean(RRTSP.DelErr);

RRTSP.AvgTREnd = mean(DataRRTStarSP(1:9,2));
RRTSP.AvgVEEnd = mean(DataRRTStarSP(10:18,2));

%
fig = figure;
fig.Units = 'inches';
fig.Position = [0 0 6 4];
j = 1;
EndErrorRRT = zeros(3,6);
for i = 1:3:16
    EndErrorRRT(:,j) = DataRRTStarSP(i:i+2,2);
    j = j+1;
end
boxplot(EndErrorRRT,'Labels',{'0.1*R TR','1*R TR','10*R TR','0.1*R VE','1*R VE','10*R VE'});
xlabel('Test Cases')
ylabel('Error')
title('Ending Error of Each RRT^* Test')
set(gca,'FontSize',12,'LineWidth',1)
%%
RT = [mean(DataRRTStarSP(1:3,2)), mean(DataSPTrace(19:21,2));
mean(DataRRTStarSP(4:6,2)), mean(DataSPTrace(22:24,2));
mean(DataRRTStarSP(7:9,2)), mean(DataSPTrace(25:27,2))];
RV = [mean(DataRRTStarSP(10:12,2)), mean(DataSPVE(19:21,2));
mean(DataRRTStarSP(13:15,2)), mean(DataSPVE(22:24,2));
mean(DataRRTStarSP(16:18,2)), mean(DataSPVE(25:27,2))];