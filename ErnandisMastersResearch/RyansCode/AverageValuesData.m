function [Means,StdDev] = AverageValuesData(Data,tend,problemDefinition)
Trials = size(Data,2);

% Interpolate values for each trial based on time data:
% Set the Time range:
if problemDefinition == 1
    Time = 5:5*60:tend;
elseif problemDefinition == 2
    Time = 5:60:tend;
end
interpErr = NaN(size(Time,2),Trials);
% Loop through each data trial:
for DataNum = 1:Trials
    % Save the data all to one array, this will change in future iteration
    % loop over each time value:
    for timeNum = 1:size(Time,2)
        % Clear Search Mat so it Doesn't save the the length of previous
        % arrays:
        clear SearchMat
        SearchMat = Time(timeNum) - Data(DataNum).Time;
        % Check if the goal is reached by the first node, needed if the time
        % check is lower than the starting time at which the goal was reached
        if SearchMat(1) <= 0
            continue
        else
            % Find the times closest to the time wanted to check
            [~,I] = min(abs(SearchMat));
            if SearchMat(I) < 0
                % If the time is on the higher side (i.e. just above the time
                % wanted to check, find the index before it)
                lowInd = I-1;
            elseif I == size(Data(DataNum).Time,1)
                % If the last index is being checked, make sure to use the index
                % before it
                lowInd = I-1;
            elseif I == 1
                lowInd = I;
                I = I+1;
            else
                % take the index after the min
                lowInd = I;
                I = I + 1;
            end
            % Interpolate what the value will be for the time being checked, this
            % is only an approximation
            y1 = Data(DataNum).Err(lowInd).IT(end);
            y2 = Data(DataNum).Err(I).IT(end);
            x1 = Data(DataNum).Time(lowInd);
            x2 = Data(DataNum).Time(I);
            interpErr(timeNum,DataNum) = y1 + (Time(timeNum)-x1)*((y2-y1)/(x2-x1));
        end
    end
end

% Make a boxplot for the error of the trials
fig = figure;
fig.Units = 'inches';
fig.Position = [0 0 3 2];
for i = 1:length(Time)
    boxplot(interpErr',Time)
end
xlabel('Time [s]')
ylabel('Error at Goal')
name = ['Evolution of Goal Error for ',num2str(Trials),' Trials'];
title(name)
% Mean and Std Dev
Means = mean(interpErr,2,'omitnan');
StdDev = std(interpErr,1,2,'omitnan');
end

