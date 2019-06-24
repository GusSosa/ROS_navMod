% invstat_test_analysis.m
% Analyzes the hardware test's computer vision and inverse kinematics
% outputs.
% Copyright A.P. Sabelhaus and BEST Lab 2019
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


% Note that since the inverse statics recording upward-swing start time is
% automatically determined by index into that log file, we don't actually
% need to specify a start time for the cv data... for the numerical
% analysis.

% We DO need to specify a start row in the CV data for the plotting.
% but we can always do -1 for the end row.

end_row_cv = -1;

% % For the June 21st, 2019 test with some errors in it (4:15pm):
% struct1.datetime_cv ='2019-6-21_161626';
% struct1.datetime_invkin = '2019-6-21_161621';
% %struct1.start_row_cv = 102;
% struct1.start_row_cv = 20;
% % struct1.end_row_cv = -1;
% struct1.end_row_cv = 850;
% % note that the rows for IK are pre-specified: just 2 to end.
% % store it
% test_structs{1} = struct1;

%%%%%% GOOD
% For the June 21st, 2019 test (5:20pm):
struct2.datetime_cv ='2019-6-21_171326';
struct2.datetime_invkin = '2019-6-21_171327';
struct2.start_row_cv = 293;
% struct2.end_row_cv = 870;
struct2.end_row_cv = end_row_cv;
% store it
% test_structs{end+1} = struct2;


%%%%%% GOOD
% For the June 23rd, 2019 test (5:00pm):
struct3.datetime_cv ='2019-6-23_170052';
struct3.datetime_invkin = '2019-6-23_170050';
struct3.start_row_cv = 230;
struct3.end_row_cv = end_row_cv;
% store it
% test_structs{end+1} = struct3;

% % For the 2019-06-24, 10:06am:
% struct4.datetime_cv ='2019-6-24_100625';
% struct4.datetime_invkin = '2019-6-24_100623';
% struct4.start_row_cv = 292;
% struct4.end_row_cv = end_row_cv;
% % store it
% test_structs{end+1} = struct4;

%%%%%%% GOOD
% For the 2019-06-24, 10:19am:
struct4.datetime_cv ='2019-6-24_101957';
struct4.datetime_invkin = '2019-6-24_101955';
struct4.start_row_cv = 303;
struct4.end_row_cv = end_row_cv;
% store it
% test_structs{end+1} = struct4;

%%%%%%% GOOD
% For the 2019-06-24, 10:31am:
struct5.datetime_cv ='2019-6-24_103132';
struct5.datetime_invkin = '2019-6-24_103130';
struct5.start_row_cv = 261;
struct5.end_row_cv = end_row_cv;
% store it
% test_structs{end+1} = struct5;

%%%%%%% GOOD
% For the 2019-06-24, 10:42am:
struct5.datetime_cv ='2019-6-24_104218';
struct5.datetime_invkin = '2019-6-24_104219';
struct5.start_row_cv = 313;
struct5.end_row_cv = end_row_cv;
% store it
test_structs{end+1} = struct5;


%%%% For those tests, need to ISOLATE the upward swing!
% Right now, we're accidentally doing both up and down.

% Call the parser
errors = invstat_test_error_analysis(test_structs, filepath);




