function cv_samples_per_invstat = getCVsamples(errors)
%getCVsamples
%   Returns the computer vision sample at each time point where the inverse
%   statics sample is recorded. Adjusts for delay.
%   
%   Inputs:
%
%       errors = struct that's being passed around
%
%   Outputs:
%       cv_samples_per_invstat = array of size:
%           size(aligned_timestamps_ik, 1) x size(aligned_data_cv, 2)


% % Pick out some constants
n = size(errors.aligned_timestamps_ik, 1);
numStates = size(errors.aligned_data_cv, 2);

% preallocate
cv_samples_per_invstat = zeros(n, numStates);

%%%%%%%%%%%%%%%%%% ABANDONDED... do not use

% 
% % We'll be counting continuously since starttime.
% %currtime = starttime;
% nexttime = starttime;
% currindex = 1;
% 
% % Initialize the results. We don't know the number of datapoints just yet
% % since end time depends on the data array.
% % Take the first datapoint in each case.
% %aligned_timestamps = timestamps(1);
% %aligned_data = data(1,:);
% 
% % This requires that startime be less than the first time in timestamps.
% aligned_timestamps = [];
% aligned_data = [];
% 
% % Iterate until we reach the end of the array.
% % That occurs when currindex > n.
% while currindex <= n
%     % The time we'll consider here is the "next" time point, which is
%     %nexttime = currtime + dt;
%     % We check if the timestamp at the current index is the one "right
%     % after" the time we'd like to sample.
%     if timestamps(currindex) >= nexttime
%         % Then save the datapoint, first
%         aligned_timestamps = [aligned_timestamps; timestamps(currindex)];
%         aligned_data = [aligned_data; data(currindex, :)];
%         % And increment the timestamp to look for.
%         %currtime = nexttime + dt;
%         nexttime = nexttime + dt;
%     end
%     % In any case, increment the index.
%     currindex = currindex +1;
% end

end

