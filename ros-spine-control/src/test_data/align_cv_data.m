function [aligned_timestamps, aligned_data] = align_cv_data(timestamps, data, starttime, dt)
%align_cv_data Outputs an array of the computer vision data for the 2D
% spine test, according to the start time and dt.
%   
%   Inputs:
%
%       timestamps = n x 1 array of millisecond timestamps for the data.
%       data = n x m array of the data that should be selected out.
%       starttime = integer greater than min(timestamps) from which to
%           start counting
%       dt = time between indices to select.
%
%   Outputs:
%       aligned timestamps, data = starting at starttime, closest to dt
%           (just after each dt)

% Pick out some constants
n = size(timestamps, 1);

% We'll be counting continuously since starttime.
currtime = starttime;
currindex = 1;

% Initialize the results. We don't know the number of datapoints just yet
% since end time depends on the data array.
% Take the first datapoint in each case.
aligned_timestamps = timestamps(1);
aligned_data = data(1,:);

% Iterate until we reach the end of the array.
% That occurs when currindex > n.
while currindex <= n
    % The time we'll consider here is the "next" time point, which is
    nexttime = currtime + dt;
    % We check if the timestamp at the current index is the one "right
    % after" the time we'd like to sample.
    if timestamps(currindex) >= nexttime
        % Then save the datapoint, first
        aligned_timestamps = [aligned_timestamps; timestamps(currindex)];
        aligned_data = [aligned_data; data(currindex, :)];
        % And increment the timestamp to look for.
        currtime = nexttime;
    end
    % In any case, increment the index.
    currindex = currindex +1;
end

end

