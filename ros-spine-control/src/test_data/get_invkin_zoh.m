function errors = get_invkin_zoh(errors)
%get_invkin_zoh Calculates a zero order hold signal from the inverse
%kinematics inputs that's appropriate for doing data analysis with the
%computer vision results for the 2d spine control test. 2018
%   
%   Inputs:
%
%       errors = strcut that's passed around in the cv data analysis work
%
%   Outputs:
%       errors = same struct but with the ZOH time series appended.

% Pick out some constants.
% Number of computer vision points:
n_cv = size(errors.aligned_timestamps_cv, 1);

% Preallocate the result. We'll want to insert into this array the signal
% from the invkin that's happening at each corresponding time in the cv
% data.
% We've got 3 states (2 com and 1 rotation.)
zoh = zeros(n_cv, 3);

% Loop through and put in the appropriate signal.
for i=1:n_cv
    % Get the exact time for this index
    cv_time = errors.aligned_timestamps_cv(i);
    % Find the index for the corresponding time in the inverse kinematics
    % inputs
    ik_i = 1;
    ik_time = errors.aligned_timestamps_ik(ik_i);
    
    while (ik_time < cv_time) && (ik_i <= size(errors.aligned_timestamps_ik, 1)-1)
        % Check the next step in the ik timestamps
        % increment
        ik_i = ik_i + 1;
        ik_time = errors.aligned_timestamps_ik(ik_i);
        %size(errors.aligned_timestamps_ik, 1)
    end
    %i
    %ik_i
    
    % We now know that the signal at ik_i was applied at cv_time.
    % That's the com_ik and rot_ik.
    zoh(i, :) = [errors.com_ik_inframe(ik_i, :), errors.rot_ik_inframe(ik_i)];
end

errors.zoh = zoh;

end

