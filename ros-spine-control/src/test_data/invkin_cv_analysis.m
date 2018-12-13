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
filepath = '.';

% The cell array to put everything in
test_structs = {};

% Test 1
struct1.datetime_cv = '2018-12-12_120256';
struct1.datetime_invkin = '2018-12-12_163130';
struct1.start_row_cv = 2161;
struct1.end_row_cv = -1;
% note that the rows for IK are pre-specified: just 2 to end.
% store it
test_structs{1} = struct1;

% Call the parser
erros = invkin_test_error_analysis(test_structs, filepath);