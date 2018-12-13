% invkin_test_error_analysis
% Copyright 2018 Andrew P. Sabelhaus and Berkeley Emergent Space Tensegrities Lab
% This script does the error analysis of the 2D single vertebra spine
% control test. 
% To be used with the ros-spine-control packages and related.

function [ errors ] = invkin_test_error_analysis(test_structs, path_to_data_folder)
% Inputs:
%
%   test_structs = a cell array, where each element has a struct in it that
%       has fields per-test. Each struct is for one test. Fields are:
%
%           datetime_cv = date/timestamp of the .csv file for the comp vis
%               data. Needs to be Y-M-D_HMS. Should be a string.
%           datetime_invkin = same, but for the IK log file.
%           start_row = The row within the data file to start parsing in
%               the data. Used because tests don't always start when the
%               computer vision system starts collecting data.
%               INDEXED FROM ZERO.
%           end_row = final row of data. Set to -1 means 'end'.
%               INDEXED FROM ZERO.
%
%   path_to_data_folder = location of the data file to read in. Should NOT
%       include a trailing '/'.
%
% Outputs:
%
%   errors = a struct containing all the types of errors that were calculated 

% For each test, read in the data.
% We'll start storing as a struct to return.
errors = {};

% Iterate over each test.
num_tests = size(test_structs, 2);

for i=1:num_tests
    % Pull out the parameters for this test.
    datetime_cv = test_structs{i}.datetime_cv;
    datetime_invkin = test_structs{i}.datetime_invkin;
    start_row_cv = test_structs{i}.start_row_cv;
    end_row_cv = test_structs{i}.end_row_cv;
    
    % Create the filename for this set of computer vision data.
    % The file name format, as output by cv_datalogger, is
    file_path_cv = strcat(path_to_data_folder, '/cv_datalogger_', ...
        datetime_cv, '.csv');
    % and for the invkin data,
    file_path_invkin = strcat(path_to_data_folder, '/invkin_datalogger_', ...
        datetime_invkin, '.csv');
    
    % Read in the data. The cv_datalogger data starts at the 3rd row,
    % and has four columns.
    % MATLAB INDEXES FROM 0 HERE!!!
    data_cv_i = [];
    if end_row_cv == -1
        % Read the whole thing
        data_cv_i = csvread(file_path_cv, start_row_cv, 0);
        % The timestamps are the first column
        errors{i}.timestamps_cv = data_cv_i(:,1);
        % The CoM is columns 2 and 3
        errors{i}.com_cv = data_cv_i(:,2:3);
        % rotations are last column
        errors{i}.rot_cv = data_cv_i(:,4);
        %errors{i}.data = data_i;
    else
        % Only read up to the specified row.
        % 4 columns when indexed from 0 is 3.
        % csvread(filename, R, C, [R1 C1 R2 C2])
        data_cv_i = csvread(file_path_cv, start_row_cv, 0, [start_row_cv 0 end_row_cv 3]);
        % The timestamps are the first column
        errors{i}.timestamps_cv = data_cv_i(:, 1);
        % The CoM is columns 2 and 3
        errors{i}.com_cv = data_cv_i(:, 2:3);
        % rotations are last column
        errors{i}.rot_cv = data_cv_i(:, 4);
        %errors{i}.data = data_i;
    end
    
    % The inverse kinematics one is easier, since the rows are known.
    % Starts at third row (indexed from zero is 2.)
    data_ik_i = csvread(file_path_invkin, 2, 0);
    % Pick out the data similar to the cv.
    % We're interested in body 2, which is indices 5:7.
    errors{i}.timestamps_ik = data_ik_i(:, 1);
    errors{i}.com_ik = data_ik_i(:, 5:6);
    errors{i}.rot_ik = data_ik_i(:, 7);
    
    % Next, convert the state information between the two frames. 
    % The origin of the MATLAB frame is hard to get in the computer vision
    % frame - lots of calculations from where we put the grid - so here's
    % an estimate within a few mm for now.
    squares_x = 5;
    squares_y = 8.2;
    % Each square is 2 cm so the offsets are
    offset_x = squares_x * 2;
    offset_y = squares_y * 2;
    % In cm, then, 
    errors{i}.com_ik_inframe = errors{i}.com_ik * 100 + [offset_x, offset_y];
    
    % A plot of the data.
    fontsize = 14;
    errfig = figure;
    hold on;
    % Set up the window
    set(gca, 'FontSize', fontsize);
    set(errfig,'Position',[100,100,500,350]);
    set(errfig,'PaperPosition',[1,1,5.8,3.5]);
    % Plot the data itself
    plot(errors{i}.com_cv(:,1), errors{i}.com_cv(:,2), 'b', 'LineWidth', 3);
    plot(errors{i}.com_ik_inframe(:,1), errors{i}.com_ik_inframe(:,2), 'r', 'LineWidth', 3);
    % Annotate the plot
    title('Spine Position Inverse Kinematics Test ');
    ylabel('Spine CoM, Y (cm)');
    xlabel('Spine CoM, X (cm)');
    legend('Test (Computer Vision)', 'Predicted State', 'Location', 'Best');
    % Set the limits:
    xlim([10 20]);
    ylim([11 23]);
    
    % A test. Get a set of aligned data. Let's do a small sampling rate,
    % like 0.1 sec. We don't want to do too small or else the sampling rate
    % is meaningless (the frames per sec on the CV is low.)
    % Data is in millisec, so 0.1 sec = 100.
    % That's actually pretty inaccurate still. Maybe 2hz. Looks better now.
    dt = 500;
    % start at the first index of the data, since we've already pulled out
    % the set of data we want (THIS WILL CHANGE LATER.)
    starttime = errors{i}.timestamps_cv(1);
    % For the data, we concatenate the center of mass and rotations.
    data_cv_foralignment = [errors{i}.com_cv, errors{i}.rot_cv];
    [aligned_timestamps, aligned_data] = align_cv_data(errors{i}.timestamps_cv, ...
        data_cv_foralignment, starttime, dt);
    % Save the aligned data in the result.
    errors{i}.aligned_timestamps_cv = aligned_timestamps;
    errors{i}.aligned_data_cv = aligned_data;
end


end


