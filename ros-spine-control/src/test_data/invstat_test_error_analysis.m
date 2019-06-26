% invkin_test_error_analysis
% Copyright 2018 Andrew P. Sabelhaus and Berkeley Emergent Space Tensegrities Lab
% This script does the error analysis of the 2D single vertebra spine
% control test. 
% To be used with the ros-spine-control packages and related.

function [ errors, means ] = invstat_test_error_analysis(test_structs, path_to_data_folder)
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
%   means = some statistics about the total dataset

% For each test, read in the data.
% We'll start storing as a struct to return.
errors = {};

% Iterate over each test.
num_tests = size(test_structs, 2);

% Declare a total number of points to have in the analysis. This should be
% greater than the total number of all time axes.
numNewPts = 700;

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
    
    % ^ 2019-05-16: The CV script and MATLAB script now have the same
    % frame, but errors need to be scaled still.
    errors{i}.com_ik_inframe = errors{i}.com_ik * 100;
    
    % The rotation also needs to be converted to degrees.
    errors{i}.rot_ik_inframe = errors{i}.rot_ik * 180/pi;
    
    
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

    % Actually, what we *really* want is the CV data at the point at which
    % each inverse statocs input is applied.
    % (even more so - the CV result at a timestep is for the *previous*
    % invstat command, since the system takes a moment to respond.)
%     errors{i}.cv_samples_per_invstat = getCVsamples(errors{i});
    % ^ Abandoned, do not use.
    
    % Finally, we should be able to just subtract to get the state error.
    errors{i}.state_error = errors{i}.aligned_data_cv - errors{i}.zoh;
    
    %% Correlate the number of points and poses
    
    % A scaled x-axis, for timestamps. Make it in sec.
    time_axis = floor(errors{i}.aligned_timestamps_cv / 1000);
    % save this for use later.
    errors{i}.time_axis = time_axis;
    
    % there are this many timesteps
    numSteps_i = size(time_axis, 1);
    % recreate the time axis then
    time_axis = (1:numSteps_i)';
    % get a set of new sampling points so we can scale to the correct
    % corresponding points between tests
    new_times = linspace(1, size(time_axis,1), numNewPts);
    % interpolate to get values at each of these points
    interpCV = interp1(time_axis, errors{i}.aligned_data_cv(:,:), new_times);
    % for the IS too
    interpIS = interp1(time_axis, errors{i}.zoh, new_times);
    % save both
    errors{i}.interpCV = interpCV;
    errors{i}.interpIS = interpIS;
    
    % and calculate the state error per new number of points.
    errors{i}.interpStateErr = interpCV - interpIS;
    
    %% Plot in the X-Y plane
    
    fontsize = 14;
    errfig = figure;
    hold on;
    % Set up the window
    set(gca, 'FontSize', fontsize);
    set(errfig,'Position',[100,100,500,350]);
    set(errfig,'PaperPosition',[1,1,5.8,3.5]);
    % Plot the data itself
%     plot(errors{i}.aligned_data_cv(:,1), errors{i}.aligned_data_cv(:,2), 'b', 'LineWidth', 3);
%     plot(errors{i}.zoh(:,1), errors{i}.zoh(:,2), 'r', 'LineWidth', 3);
    plot(errors{i}.interpCV(:,1), errors{i}.interpCV(:,2), 'b', 'LineWidth', 3);
    plot(errors{i}.interpIS(:,1), errors{i}.interpIS(:,2), 'r', 'LineWidth', 3);
    % Annotate the plot
    title('Spine Inverse Statics Control Test ');
    ylabel('Spine CoM, Y (cm)');
    xlabel('Spine CoM, X (cm)');
    legend('Test (Computer Vision)', 'Predicted State', 'Location', 'Best');
    xlim([20 35]);
    ylim([7 27]);

    %% Plot the errors individually
    
    %% Make a plot of the errors. Adapted from Drew's script for the MPC
    % results of summer 2019.

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
%     plot(time_axis, errors{i}.state_error(:,1), 'Color', 'm', 'LineWidth', 2);
    plot(errors{i}.interpStateErr(:,1), 'Color', 'm', 'LineWidth', 2)
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
%     plot(time_axis, errors{i}.state_error(:,2), 'Color', 'm', 'LineWidth', 2);
    plot(errors{i}.interpStateErr(:,2), 'Color', 'm', 'LineWidth', 2);
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
%     plot(time_axis, errors{i}.state_error(:,3), 'Color', 'm', 'LineWidth', 2);
    plot(errors{i}.interpStateErr(:,3), 'Color', 'm', 'LineWidth', 2);
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

%% 2019-06-25: Want to do a combined plot of all the above.

% clear out all the windows (this is easier than comment/uncomment blocks
% of code above
close all;

% First, reorganize into vectors of points at each timestep.
% There are numNewPoints samples and num_tests datapoints.
x_all_CV = zeros(numNewPts, num_tests);
y_all_CV = zeros(numNewPts, num_tests);
theta_all_CV = zeros(numNewPts, num_tests);

x_all_IS = zeros(numNewPts, num_tests);
y_all_IS = zeros(numNewPts, num_tests);
theta_all_IS = zeros(numNewPts, num_tests);

x_all_err = zeros(numNewPts, num_tests);
y_all_err = zeros(numNewPts, num_tests);
theta_all_err = zeros(numNewPts, num_tests);

% distance errors are sqrt(errX^2 + errY^2)
distErr = zeros(numNewPts, num_tests);


% Fill them in per test
% this is really inefficient
for i=1:num_tests
    x_all_CV(:, i) = errors{i}.interpCV(:,1);
    y_all_CV(:, i) = errors{i}.interpCV(:,2);
    theta_all_CV(:, i) = errors{i}.interpCV(:,3);
    
    x_all_IS(:, i) = errors{i}.interpIS(:,1);
    y_all_IS(:, i) = errors{i}.interpIS(:,2);
    theta_all_IS(:, i) = errors{i}.interpIS(:,3);
    
    x_all_err(:, i) = errors{i}.interpStateErr(:,1);
    y_all_err(:, i) = errors{i}.interpStateErr(:,2);
    theta_all_err(:, i) = errors{i}.interpStateErr(:,3);
    
    % the distance error for this test, all timesteps, is
    distErr(:,i) = sqrt(x_all_err(:,i).^2 + y_all_err(:,i).^2);
end

% Get the mean for each signal.
meansCV = zeros(numNewPts, 3);
meansIS = zeros(numNewPts, 3);
meansErr = zeros(numNewPts, 3);
% we'll be plotting little circles of the average distance error at each
% pose.
meansDistErr = zeros(numNewPts, 1);

meansCV(:,1) = mean(x_all_CV, 2);
meansCV(:,2) = mean(y_all_CV, 2);
meansCV(:,3) = mean(theta_all_CV, 2);

meansIS(:,1) = mean(x_all_IS, 2);
meansIS(:,2) = mean(y_all_IS, 2);
meansIS(:,3) = mean(theta_all_IS, 2);

meansErr(:,1) = mean(x_all_err, 2);
meansErr(:,2) = mean(y_all_err, 2);
meansErr(:,3) = mean(theta_all_err, 2);

% meansDistErr = mean(distErr, 2);

%%%%%%%%%%%%%%%%%%%%%
% actually, this distance error is not what we want.
%
% What we want to talk about is the distance between the AVERAGE position
% and each trajectory.
% Because there's a trend in the distances: for example, all Y are
% negative.
% Thus we don't want to plot a circle that implies possible positive Y
% errors, since we didn't observe any.
%
% The right number to use will be:
% CV error from mean in each direction at each point
% distance of THAT number
% mean of THAT number
% plotted around the CV mean 

means.meansCv = meansCV;
means.meansIS = meansIS;
means.meansErr = meansErr;
means.meansDistErr = meansDistErr;

%% Useful Statistics

% We're going to do the following plots:
%
%   Individual subplots:
%       - standard deviation for each timestep / SEM
%       - mean +/- SEM for each plot, to shade a region. (MATLAB's 'area'.)

% standard errors are (stdev)/sqrt(num_tests)
% we're interested in the stddev of the mean
stddev_x = (std(x_all_err'))';
stddev_y = (std(y_all_err'))';
stddev_theta = (std(theta_all_err'))';

dof = sqrt(num_tests);
sem_x = stddev_x./dof;
sem_y = stddev_y./dof;
sem_theta = stddev_theta./dof;

% Get the bounding curves for each plot. adjust the mean by +/- the sem
meansErrUpper = zeros(numNewPts, 3);
meansErrLower = zeros(numNewPts, 3);
% do each individually.
% meansErrUpper(:,1) = meansErr(:,1) + sem_x;
% meansErrUpper(:,2) = meansErr(:,2) + sem_y;
% meansErrUpper(:,3) = meansErr(:,3) + sem_theta;
% meansErrLower(:,1) = meansErr(:,1) - sem_x;
% meansErrLower(:,2) = meansErr(:,2) - sem_y;
% meansErrLower(:,3) = meansErr(:,3) - sem_theta;

%%%%%%%%%%%%%%%%% IT APPEARS that the SEM isn't what to use here. It's the
%%%%%%%%%%%%%%%%% standard deviation itself that makes more sense.
meansErrUpper(:,1) = meansErr(:,1) + stddev_x;
meansErrUpper(:,2) = meansErr(:,2) + stddev_y;
meansErrUpper(:,3) = meansErr(:,3) + stddev_theta;
meansErrLower(:,1) = meansErr(:,1) - stddev_x;
meansErrLower(:,2) = meansErr(:,2) - stddev_y;
meansErrLower(:,3) = meansErr(:,3) - stddev_theta;

% We need a corrected time axis for the horizontal here.
% All tests had minor variations, BUT we do know when the IS commands were
% sent: 0 to 80 sec, one command per second. 
% So, do an axis for the idealized/corrected case.
correctedTimeAxis = linspace(1, 80, numNewPts);

% We'll need a time axis for the fill.
% newtimeaxis = 1:numNewPts;
% and concatenate to some new points 
% (thanks to
% https://www.mathworks.com/matlabcentral/answers/180829-shade-area-between-graphs)
% timeFill = [newtimeaxis, fliplr(newtimeaxis)];
timeFill = [correctedTimeAxis, fliplr(correctedTimeAxis)];

% ...though this is a bit wishy washy with our interpolation so the data is
% correlated to each other, so numNewPts doesn't have statistical meaning.

%% Now, make new plots with the mean in it.

%% Plot in the X-Y plane

fontsize = 14;
errfig = figure;
hold on;
% Set up the window
set(gca, 'FontSize', fontsize);
set(errfig,'Position',[100,100,500,350]);
set(errfig,'PaperPosition',[1,1,5.8,3.5]);

% circle color
% circleColor = 'g';
circleColor = [255 163 163];
% need to scale it between 0 and 255.
circleColor = circleColor./255;

% Plot some circles centered at each IS point representing the average
% region that each pose exists in.
for i=1:numNewPts
    % center point
    circle_x = meansCV(i,1);
    circle_y = meansCV(i,2);
    % radius is
    circle_r = meansDistErr(i);
    circles(circle_x, circle_y, circle_r, 'FaceColor', circleColor, 'EdgeColor', circleColor, 'HandleVisibility', 'off');
end

% Plot the data itself
plot(meansIS(:,1), meansIS(:,2), 'b', 'LineWidth', 4);
plot(meansCV(:,1), meansCV(:,2), 'r', 'LineWidth', 3);
% Annotate the plot
title('Spine Inverse Statics Control Test ');
ylabel('Spine CoM, Y (cm)');
xlabel('Spine CoM, X (cm)');
legend('Reference Traj.', 'Mean Test Traj. (C.V.)', 'Location', 'Best');
xlim([20 35]);
ylim([7 27]);
hold off;

%% Make a plot of the errors. Adapted from Drew's script for the MPC
% results of summer 2019.

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
% Fill in area for the SEM 
inBetween = [meansErrLower(:,1)', fliplr(meansErrUpper(:,1)')];
fill(timeFill, inBetween, circleColor, 'EdgeColor', circleColor);
% Plot the X errors
% plot(meansErrUpper(:,1), 'Color', 'b', 'LineWidth', 2)
% plot(meansErr(:,1), 'Color', 'r', 'LineWidth', 2)
plot(correctedTimeAxis, meansErr(:,1), 'Color', 'r', 'LineWidth', 2)
% plot(meansErrLower(:,1), 'Color', 'g', 'LineWidth', 2)

% Plot the zero line
%plot(t, zero_line, 'b-', 'LineWidth','1');
refline_handle = refline(0,0);
set(refline_handle, 'LineStyle', '--', 'Color', 'k', 'LineWidth', 0.5);
%xlabel('Time (msec)');
ylabel('X (cm)');
% Only create a title for the first plot, that will serve for all the others too.
%title('Tracking Errors in X Y Z  \theta \gamma \psi');
title(' State Errors, Inverse Statics Test');
set(gca,'FontSize',fontsize);
% Scale the plot. A good scale here is...
% ylim([-0.6 0.6]);
ylim([-0.3 0.3]);
% xlim([0 705]);
xlim([0 82]);

% Make the legend
%nodisturblabel = sprintf('No Noise');
%disturblabel = sprintf('With Noise');
%legend_handle = legend('Hardware Test  'Location', 'North', 'Orientation', 'horizontal');

hold off;

% Plot the Y errors
subplot_handle = subplot(3,1,2);
hold on;
% Fill in area for the SEM 
inBetween = [meansErrLower(:,2)', fliplr(meansErrUpper(:,2)')];
fill(timeFill, inBetween, circleColor, 'EdgeColor', circleColor);
% Plot the Y errors
% plot(meansErr(:,2), 'Color', 'r', 'LineWidth', 2);
plot(correctedTimeAxis, meansErr(:,2), 'Color', 'r', 'LineWidth', 2);

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
% ylim([-1.5, 1.5]);    
ylim([-0.8, 0.8]);
% xlim([0 705]);
xlim([0 82]);

set(gca,'FontSize',fontsize);

hold off;

% Plot the gamma errors
subplot_handle = subplot(3,1,3);
hold on;
% Fill in area for the SEM 
inBetween = [meansErrLower(:,3)', fliplr(meansErrUpper(:,3)')];
fill(timeFill, inBetween, circleColor, 'EdgeColor', circleColor);
% plot(meansErr(:,3), 'Color', 'r', 'LineWidth', 2);
plot(correctedTimeAxis, meansErr(:,3), 'Color', 'r', 'LineWidth', 2);

% Plot the zero line
%plot(t, zero_line, 'b-', 'LineWidth','1');
refline_handle = refline(0,0);
set(refline_handle, 'LineStyle', '--', 'Color', 'k', 'LineWidth', 0.5);
%legend('Vertebra 1 (Bottom)', 'Vertebra 2 (Middle)', 'Vertebra 3 (Top)');
%xlabel('Time (msec)');
ylabel('\theta (deg)');
%title('Tracking Error in Y');
% ylim([-6 6]);
ylim([-2.8, 2.8]);
% xlim([0 705]);
xlim([0 82]);

% Move the plot very slightly to the left
% For these lower figures, move them upwards a bit more.
%P = get(subplot_handle,'Position')
%set(subplot_handle,'Position',[P(1)-0.06 P(2)+0.07 P(3)+0.01 P(4)-0.04])

% Finally, a label in X at the bottom
xlabel('Time (sec)');
% xlabel('Pose number (index $\propto$ time)');

% Here's a bit of an approximation for physical understanding purposes.
% It's 

set(gca,'FontSize',fontsize);

hold off;

end

