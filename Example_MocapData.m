
%% Example Code : Load and display CMU Mocap data

% made by Terry Taewoong Um (terry.t.um@gmail.com)
% Adaptive Systems Lab., University of Waterloo

% 0. Download motion capture data from CMU Graphics Lab website
%    http://mocap.cs.cmu.edu/search.php
%    Download ASF and AMC files of the motions that you want to display
% 1. Name the ASF and AMC files you want to display
% 2. Load a human model and motions from asf and amc files, repectively
% 3. Display the loaded motion

close all;  clearvars; 
addpath('./SE3_Operations');

% 1. Name the ASF and AMC files you want to display
AsfFilename = 'MocapData\10.asf';
AmcFilename = ['MocapData\10_01.amc'; 'MocapData\10_02.amc'; 'MocapData\10_03.amc'; ...
                'MocapData\10_05.amc'; 'MocapData\10_06.amc';];

nFiles = size(AmcFilename,1);   % number of motions
nData = zeros(nFiles,1);        % Length of each motion
% mdl_subject{kk,1} : cell{root, torso, rightArm, leftArm, rlightLeg, leftLeg}
mdl_subject = cell(nFiles,1);

for ii_file = 1:nFiles 
% 2. Load a human model and motions from asf and amc files, repectively
    mdl_subject{ii_file,1} = LoadFromAsf(AsfFilename);
    [mdl_subject{ii_file,1} nData(ii_file,1)] = LoadFromAmc(AmcFilename(ii_file,:), mdl_subject{ii_file,1});

% 3. Display the motion capture data using plot3
    bShowFrame = 1;     % 1:Show the frames  0: Do not show them
    bEETraj = [1 0 0 0 1 1];      % 1:Show the end-effector trajectories  0 : Do not show them 
%   bEETraj: [1_root, 2_torso, 3_rightArm, 4_leftArm, 5_rightLeg, 6_leftLeg]
    axisRange = FindAxisRange(mdl_subject{ii_file,1});             % Find appropriate axes for plotting motion               
    DisplayModel(mdl_subject{ii_file,1}, axisRange, bShowFrame, bEETraj);   % Display the motion
end
