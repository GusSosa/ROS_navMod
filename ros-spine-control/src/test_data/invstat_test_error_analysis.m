% invkin_test_error_analysis
% Copyright 2018 Andrew P. Sabelhaus and Berkeley Emergent Space Tensegrities Lab
% This script does the error analysis of the 2D single vertebra spine
% control test. 
% To be used with the ros-spine-control packages and related.

function [ errors ] = invstat_test_error_analysis(test_structs, path_to_data_folder)
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
    %% Pull out the parameters for this test.
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
    
    %% Read in the data. The cv_datalogger data starts at the 3rd row,
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
    %%%%% FOR THE UP-DOWN SWING TEST: the upward swing actually starts at
    %%%%% row 42, indexed from zero is 41.
    data_ik_i = csvread(file_path_invkin, 41, 0);
    
    % Starts at third row (indexed from zero is 2.)
%     data_ik_i = csvread(file_path_invkin, 2, 0);
    % Pick out the data similar to the cv.
    % We're interested in body 2, which is indices 5:7.
    errors{i}.timestamps_ik = data_ik_i(:, 1);
    errors{i}.com_ik = data_ik_i(:, 5:6);
    errors{i}.rot_ik = data_ik_i(:, 7);
    
    %% Next, convert the state information between the two frames. 
%     % The origin of the MATLAB frame is hard to get in the computer vision
%     % frame - lots of calculations from where we put the grid - so here's
%     % an estimate within a few mm for now.
%     squares_x = 5;
%     squares_y = 8.2;
%     % Each square is 2 cm so the offsets are
%     offset_x = squares_x * 2;
%     offset_y = squares_y * 2;
%     % In cm, then, 
%     errors{i}.com_ik_inframe = errors{i}.com_ik * 100 + [offset_x, offset_y];
    
    % ^ 2019-05-16: The CV script and MATLAB script now have the same
    % frame, but errors need to be scaled still.
    errors{i}.com_ik_inframe = errors{i}.com_ik * 100;
    
    % The rotation also needs to be converted to degrees.
    errors{i}.rot_ik_inframe = errors{i}.rot_ik * 180/pi;
    
    %% A plot of the data.
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
    title('Spine Inverse Statics Control Test ');
    ylabel('Spine CoM, Y (cm)');
    xlabel('Spine CoM, X (cm)');
    legend('Test (Computer Vision)', 'Predicted State', 'Location', 'Best');
    % Set the limits:
%     xlim([10 20]);
%     ylim([11 23]);
    xlim([20 35]);
    ylim([7 27]);
    
    %% Get a set of aligned data. 
    % This is necessary so we can zero-order-hold the inverse kinematics
    % inputs, with the right timestamps and indices, for a time error 
    % analysis (subtraction!).
    
    % Let's do a small sampling rate,
    % like 0.1 sec. We don't want to do too small or else the sampling rate
    % is meaningless (the frames per sec on the CV is low.)
    % Data is in millisec, so 0.1 sec = 100.
    % That's actually pretty inaccurate still. Maybe 2hz. Looks better now.
    dt = 100;
    % start at the first index of the data, since we've already pulled out
    % the set of data we want (THIS WILL CHANGE LATER.)
    %starttime = errors{i}.timestamps_cv(1);
    % When the IK and CV data represents the same test, we can take the
    % first time index for the IK as the start of the computer vision.
    starttime = errors{i}.timestamps_ik(1);
    % For the data, we concatenate the center of mass and rotations.
    data_cv_foralignment = [errors{i}.com_cv, errors{i}.rot_cv];
    [aligned_timestamps, aligned_data] = align_cv_data(errors{i}.timestamps_cv, ...
        data_cv_foralignment, starttime, dt);
    % Save the aligned data in the result.
    errors{i}.aligned_timestamps_cv = aligned_timestamps;
    errors{i}.aligned_data_cv = aligned_data;
    
    % adjust all the timestamps so they're relative. This makes the math
    % easier.
    time_offset = errors{i}.timestamps_ik(1);
    errors{i}.aligned_timestamps_ik = errors{i}.timestamps_ik - time_offset;
    errors{i}.aligned_timestamps_cv = errors{i}.aligned_timestamps_cv - time_offset; 
    
    % Now, we can do the ZOH signal for the inverse kinematics.
    errors{i} = get_invkin_zoh(errors{i});
    
    % Finally, we should be able to just subtract to get the state error.
    errors{i}.state_error = errors{i}.aligned_data_cv - errors{i}.zoh;
    
    %% Make a plot of the errors. Adapted from Drew's script for the MPC
    % results of summer 2018.
    
    % A scaled x-axis, for timestamps. Make it in sec.
    time_axis = floor(errors{i}.aligned_timestamps_cv / 1000);

    % Create the handle for the overall figure
    % Also, use the OpenGL renderer so that symbols are formatted correctly.
    %errors_handle = figure('Renderer', 'opengl');
    errors_handle = figure;
    hold on;
    set(gca,'FontSize',fontsize);
    % This figure will have 3 smaller plots, so make it larger than my
    % usual window dimensions.
    set(errors_handle,'Position',[100,100,500,350]);
    %set(errfig,'Position',[100,100,500,350]);
    %set(errfig,'PaperPosition',[1,1,5.8,3.5]);
    
    % Start the first subplot
    subplot_handle = subplot(3, 1, 1);
    hold on;
    % Plot the X errors
    plot(time_axis, errors{i}.state_error(:,1), 'Color', 'm', 'LineWidth', 2);
    % Plot the zero line
    %plot(t, zero_line, 'b-', 'LineWidth','1');
    refline_handle = refline(0,0);
    set(refline_handle, 'LineStyle', '--', 'Color', 'k', 'LineWidth', 0.5);
    %xlabel('Time (msec)');
    ylabel('X (cm)');
    % Only create a title for the first plot, that will serve for all the others too.
    %title('Tracking Errors in X Y Z  \theta \gamma \psi');
    title('   State Errors, Inverse Statics Control Test');
    set(gca,'FontSize',fontsize);
    % Scale the plot. A good scale here is...
    ylim([-0.6 0.6]);
    
    % Make the legend
    %nodisturblabel = sprintf('No Noise');
    %disturblabel = sprintf('With Noise');
    %legend_handle = legend('Hardware Test  'Location', 'North', 'Orientation', 'horizontal');

    hold off;

    % Plot the Y errors
    subplot_handle = subplot(3,1,2);
    hold on;
    % Plot the X errors
    plot(time_axis, errors{i}.state_error(:,2), 'Color', 'm', 'LineWidth', 2);
    % Plot the zero line
    %plot(t, zero_line, 'b-', 'LineWidth','1');
    refline_handle = refline(0,0);
    set(refline_handle, 'LineStyle', '--', 'Color', 'k', 'LineWidth', 0.5);
    %legend('Vertebra 1 (Bottom)', 'Vertebra 2 (Middle)', 'Vertebra 3 (Top)');
    %xlabel('Time (msec)');
    ylabel('Y (cm)');
    %title('Tracking Error in Y');
    % Adjust by roughly the amount we scaled the disturbances: 1/6 of the
    % length. Plus a small change to make the numbers prettier, about -(1/3)+0.05
    %ylim([-0.2, (7/12)-0.1]); 
    ylim([-1.5, 1.5]);    
    set(gca,'FontSize',fontsize);
    
    hold off;
    
    % Plot the gamma errors
    subplot_handle = subplot(3,1,3);
    hold on;
    plot(time_axis, errors{i}.state_error(:,3), 'Color', 'm', 'LineWidth', 2);
    % Plot the zero line
    %plot(t, zero_line, 'b-', 'LineWidth','1');
    refline_handle = refline(0,0);
    set(refline_handle, 'LineStyle', '--', 'Color', 'k', 'LineWidth', 0.5);
    %legend('Vertebra 1 (Bottom)', 'Vertebra 2 (Middle)', 'Vertebra 3 (Top)');
    %xlabel('Time (msec)');
    ylabel('\theta (deg)');
    %title('Tracking Error in Y');
    ylim([-6 6]);
    % Move the plot very slightly to the left
    % For these lower figures, move them upwards a bit more.
    %P = get(subplot_handle,'Position')
    %set(subplot_handle,'Position',[P(1)-0.06 P(2)+0.07 P(3)+0.01 P(4)-0.04])
    
    % Finally, a label in X at the bottom
    xlabel('Time (sec)');
    set(gca,'FontSize',fontsize);

    hold off;
end


end


