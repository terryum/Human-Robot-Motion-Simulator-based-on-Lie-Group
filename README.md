# Human/Robot Motion Simulator based on Lie Group
For more details, please refer the book ["A Mathematical Introduction to Robot Manipulation"](http://www.cds.caltech.edu/~murray/mlswiki/?title=First_edition) 
or ["A First Course in Robot Mechanics"](http://robotics.snu.ac.kr/fcp/files/_pdf_files_publications/a_first_coruse_in_robot_mechanics.pdf)

코드에 대한 배경 지식은 ["YouTube 로봇강좌(한국어)"](https://www.youtube.com/playlist?list=PL0oFI08O71gK02q3Sar5dg5vMLcae319p)에서 배우실 수 있으며, 관련 내용을 ["T-Robotics 블로그"](http://t-robotics.blogspot.kr/)에서도 찾아보실 수 있습니다. 

## Dependencies

These codes can be run independently, that is, there is no dependency for these codes.

However, in the example code **Example_6DOF_Puma_withRTB.m**, [Robotics toolbox](http://petercorke.com/Robotics_Toolbox.html) has been used for providing comparative results. If you do not want to install the Robotics toolbox, please run **Example_3DOF_Puma_withoutRTB.m** or **Example_TwoLink.m** instead of **Example_6DOF_Puma_withRTB.m**

Also, if you want to simulate motion capture data, you can download the data from [CMU Graphics Lab Motion Capture Database](http://mocap.cs.cmu.edu/). An example of simulation motion capture data can be found in **Example_MocapData.m**

## Instruction

You may be able to understand how to use the codes if you look into the example codes. As presented in the examples code, the procedure for simulation a serial robot is as follows.

1. Load a model from Robotics toolbox or manually build a model
2. Set desired trajectory you want
3. Call the forward kinematics or inverse kinemtacis function
4. Plot the results

If you want to simulate motion capture data downloaded from CMU Mocap dataset, you can do it as follows.

1. Name the ASF and AMC files you want to simulate
2. Load the files
3. Plot the motions

## Example_3DOF_Puma_withoutRTB.m
```matlab
%% 1. Load a model from Robotics Toolbox or manually build a model
robotModel = Model_3DOF_Puma560();  

%% 2. Set a desired trajectory
[q_query dq_query ddq_query] = SetTrajectory(robotModel.nLink-1);    
[nData nDim] = size(q_query);

%% 3. Call the forward kinematics function or inverse kinematic function
for ii=1:nData
    % Forward Kinematics
    % T_Result_Lie : Trajectory of the EE,       T_AllJointTraj : Trajectories of the all joints    
    [T_Result_Lie(:,:,ii) T_AllJointTraj(:,:,:,ii)]=FwdKin_Serial(robotModel, q_query(ii,:));   
    % Inverse Dynamics
    tau_Result_Lie(ii,:) = InvDyn_Serial(robotModel, q_query(ii,:), dq_query(ii,:), ddq_query(ii,:));  
end
```

## Example_MocapData.m
```matlab
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
```

## Files
**Folders**
* /SE3_Operations       :  Lie group / Lie algebra operators
* /MocapData            :  Motion capture data downloaded from CMU Graphics Lab webpage

**For robot motions**
* Example_6DOF_Puma_withRTB.m     :  Puma560(6DOF) example (Robotics toolbox is required)
* Example_3DOF_Puma_withoutRTB.m  :  Puma560(3DOF) example (Robotics toolbox is not required)
* Example_TwoLink.m     :  Two link plannar robot(2DOF) example (Robotics toolbox is not required)
* FwdKin_Serial.m       :  Forward kinematics for a grounded serial robot
* InvDyn_Serial.m       :  Inverse dynamics for a grounded serial robot (no external force except for gravity force)
* SetTrajectory.m       :  Set a trajectory as you want
* Model_3DOF_Puma560.m  :  Hand-crafted model - Puma560(3DOF)
* Model_TwoLink.m       :  Hand-crafted model - TwoLink(2DOF)
* Model_from_RTB.m      :  Transform a robotics toolbox robot model to a Lie group-based robot model

**For mocap data**
* Example_MocapData.m   :  Using motion capture data example (Robotics toolbox is not required)
* LoadFromAsf.m         :  Load a human model from the asf file
* LoadFromAmc.m         :  Load motions from the amc files

**For the both**
* DisplayModel.m        :  Animate the human/robot motions
* FindAxisRange.m       :  Find appropriate axis ranges for displaying the model


## Contact
* If you find any errors or have any questions, please email to terry.t.um@gmail.com 


