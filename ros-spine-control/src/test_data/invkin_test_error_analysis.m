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
%           datetime = date/timestamp of the .csv file. Needs to be
%               Y-M-D_HMS. Should be a string.
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
    datetime = test_structs{i}.datetime;
    start_row = test_structs{i}.start_row;
    end_row = test_structs{i}.end_row;
    
    % Create the filename for this set of data.
    % The file name format, as output by cv_datalogger, is
    file_path = strcat(path_to_data_folder, '/cv_datalogger_', ...
        datetime, '.csv');
    
    % Read in the data. The cv_datalogger data starts at the 3rd row,
    % and has four columns.
    % MATLAB INDEXES FROM 0 HERE!!!
    data_i = [];
    if end_row == -1
        % Read the whole thing
        data_i = csvread(file_path, start_row, 0);
        % The timestamps are the first column
        errors{i}.timestamps = data_i(:,1);
        % The CoM is columns 2 and 3
        errors{i}.com = data_i(:,2:3);
        % rotations are last column
        errors{i}.rot = data_i(:,4);
        %errors{i}.data = data_i;
    else
        % Only read up to the specified row.
        % 4 columns when indexed from 0 is 3.
        % csvread(filename, R, C, [R1 C1 R2 C2])
        data_i = csvread(file_path, start_row, 0, [startrow 0 endrow 3]);
        % The timestamps are the first column
        errors{i}.timestamps = data_i(:,1);
        % The CoM is columns 2 and 3
        errors{i}.com = data_i(:,2:3);
        % rotations are last column
        errors{i}.rot = data_i(:,4);
        %errors{i}.data = data_i;
    end
    
    % A plot of the data.
    figure;
    hold on;
    plot(errors{i}.com(:,1), errors{i}.com(:,2), '.');

end


end


