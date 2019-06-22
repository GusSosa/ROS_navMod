% invkin_cv_analysis.m
% Analyzes the hardware test's computer vision and inverse kinematics
% outputs.
% Copyright A.P. Sabelhaus and BEST Lab 2018
% This script is the base that calls the other functions.

% set up
clear all;
close all;
clc;

% The folder where the data files are. 
% Needs to be the same for all files (as of now at least - to do make
% modular)
% filepath = '.';

% June 2019: moved into the summer2019 folder.
filepath = './summer2019/';

% The cell array to put everything in
test_structs = {};

% % Earlier test - data did not have aligned timestamps
% struct1.datetime_cv = '2018-12-12_120256';
% struct1.datetime_invkin = '2018-12-12_163130';
% struct1.start_row_cv = 2161;
% struct1.end_row_cv = -1;
% % note that the rows for IK are pre-specified: just 2 to end.
% % store it
% test_structs{1} = struct1;

% % Test 1 - wrong homography
% struct1.datetime_cv = '2018-12-12_185559';
% struct1.datetime_invkin = '2018-12-12_185556';
% struct1.start_row_cv = 1050;
% struct1.end_row_cv = -1;
% % note that the rows for IK are pre-specified: just 2 to end.
% % store it
% test_structs{1} = struct1;

% % Test 2 - wrong homography
% struct2.datetime_cv = '2018-12-12_190241';
% struct2.datetime_invkin = '2018-12-12_190238';
% struct2.start_row_cv = 1050;
% struct2.end_row_cv = -1;
% % note that the rows for IK are pre-specified: just 2 to end.
% % store it
% test_structs{2} = struct2;

% Test 1 - corrected homography
% struct1.datetime_cv = '2018-12-12_192239';
% struct1.datetime_invkin = '2018-12-12_192238';
% %struct1.start_row_cv = 102;
% struct1.start_row_cv = 3;
% struct1.end_row_cv = -1;
% % note that the rows for IK are pre-specified: just 2 to end.
% % store it
% test_structs{1} = struct1;

% % Test on 2018-12-13: moving the camera so the vertebra is in the center of
% % the frame, with as little distortion as possible.
% struct1.datetime_cv = '2018-12-13_121644';
% struct1.datetime_invkin = '2018-12-13_121638';
% %struct1.start_row_cv = 102;
% struct1.start_row_cv = 3;
% struct1.end_row_cv = -1;
% % note that the rows for IK are pre-specified: just 2 to end.
% % store it
% test_structs{1} = struct1;

% Combination of tests on 2018-12-13. Here's what was going on:
% Somehow, we introduced an offset error since the good test on the 12th.
% Since all we needed anyway was a timestep correlation, and since the
% inverse kinematics inputs/states are always the same, we did the
% following:
% 1) Took the timestamps from -10 rows since the first response of the
% sweep test from the correlated test
% 2) Plugged them into the older test data
% 3) Run the analysis with the "new" state timestamps and "old" data. This
% is actually OK, modulo some potential millisec errors in the response,
% since the "new" state timestamps are the same frequency as the "old".
% There's just potentiall a small offset.
% struct1.datetime_cv ='2018-12-12_120256_timeadjusted';
% struct1.datetime_invkin = '2018-12-13_121638';
% %struct1.start_row_cv = 102;
% struct1.start_row_cv = 3;
% struct1.end_row_cv = -1;
% % note that the rows for IK are pre-specified: just 2 to end.
% % store it
% test_structs{1} = struct1;

% ...accidentally deleted the parameters for the May 16th test, look at
% past commits if needed.

% For the June 21st, 2019 test with some errors in it (4:15pm):
struct1.datetime_cv ='2019-6-21_161626';
struct1.datetime_invkin = '2019-6-21_161621';
%struct1.start_row_cv = 102;
struct1.start_row_cv = 20;
% struct1.end_row_cv = -1;
struct1.end_row_cv = 850;
% note that the rows for IK are pre-specified: just 2 to end.
% store it
test_structs{1} = struct1;

% For the June 21st, 2019 test (5:20pm):
struct1.datetime_cv ='2019-6-21_171326';
struct1.datetime_invkin = '2019-6-21_171327';
%struct1.start_row_cv = 102;
struct1.start_row_cv = 15;
% struct1.end_row_cv = -1;
struct1.end_row_cv = 870;
% note that the rows for IK are pre-specified: just 2 to end.
% store it
test_structs{1} = struct1;

%%%% For those tests, need to ISOLATE the upward swing!
% Right now, we're accidentally doing both up and down.

% Call the parser
errors = invkin_test_error_analysis(test_structs, filepath);




