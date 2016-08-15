
%% Example Code : Puma560 (6 DOF, Robotics Toolbox required) 

% made by Terry Taewoong Um (terry.t.um@gmail.com)
% Adaptive Systems Lab., University of Waterloo

% - All 6 by 1 spatial vectors are represented as [angular motion; linear motion]
%   (e.g. V = [w1; w2; w3; v1; v2; v3])
% - Link 1 is reserved for the ground link

% Example Code
% 1. Load a model from Robotics Toolbox or build a model manually
% 2. Set the desired trajectory q, q', q"
% 3. Call the forward kinematics function or inverse kinematic function
% 4. Plot the results

close all;  clearvars; 
addpath('./SE3_Operations');

%% 1. Load a model from Robotics Toolbox or manually build a model
% In this example, we will load the Puma560 model from the robotics toolbox
mdl_puma560;        % Robotics toolbox folders should be added in the path
robotRTB = p560;
robotModel = Model_from_RTB(robotRTB);      % Get a Lie group based robot model from the robotics toolbox model

%% 2. Set a desired trajectory
[q_query dq_query ddq_query] = SetTrajectory(robotModel.nLink-1);     % Set the desired trajectory as you want
[nData nDim] = size(q_query);

%% 3. Call the forward kinematics function or inverse kinematic function
for ii=1:nData
    % Forward Kinematics
    % T_Result_Lie : Trajectory of the EE       T_AllJointTraj : Trajectories of the all joints    
    [T_Result_Lie(:,:,ii) T_AllJointTraj(:,:,:,ii)]= FwdKin_Serial(robotModel, q_query(ii,:));   % using lie group dynamics (_Lie)
    T_Result_RTB(:,:,ii) = robotRTB.fkine(q_query(ii,:));     % using robotics toolbox   (_RTB)
    [ang_diff(ii,:) pos_diff(ii,:)]  = Diff_SE3(T_Result_Lie(:,:,ii), T_Result_RTB(:,:,ii));

    % Inverse Dynamics
    tau_Result_Lie(ii,:) = InvDyn_Serial(robotModel, q_query(ii,:), dq_query(ii,:), ddq_query(ii,:));   % using lie group dynamics (_Lie)
    tau_Result_RTB(ii,:) = robotRTB.rne(q_query(ii,:), dq_query(ii,:), ddq_query(ii,:));       % using robotics toolbox   (_RTB)
    tau_diff = tau_Result_Lie - tau_Result_RTB;
end
robotModel.T_Moving_Home = T_AllJointTraj;  % Store the trajectory in the robot model

%% 4. Plot the results
% Get (X,Y,Z) positions
for ii=1:nData
    xx_Lie(ii,1) = T_Result_Lie(1,4,ii);    yy_Lie(ii,1) = T_Result_Lie(2,4,ii);        zz_Lie(ii,1) = T_Result_Lie(3,4,ii);
    xx_RTB(ii,1) = T_Result_RTB(1,4,ii);    yy_RTB(ii,1) = T_Result_RTB(2,4,ii);        zz_RTB(ii,1) = T_Result_RTB(3,4,ii);
end

%  Plot the results of the forward kinematics and inverse dynamics
figure('position', [50 20 1600 800])
subplot(2,3,1);
scatter3(xx_Lie,yy_Lie,zz_Lie,'.');
title('Fwd. Kin. using LIE');   xlabel('Samples'); ylabel('EE Position');

subplot(2,3,2);
scatter3(xx_RTB,yy_RTB,zz_RTB,'.');
title('Fwd. Kin. using RTB');   xlabel('Samples'); ylabel('EE Position');

subplot(2,3,3);
plot(1:nData, ang_diff); hold on;
plot(1:nData, pos_diff); hold off;
title('Difference b/w the LIE & RTB');
xlabel('Samples'); ylabel('EE Position');   legend('Ang. Diff.','Pos. Diff.');

subplot(2,3,4);
plot(1:nData,tau_Result_Lie);
title('Inv. Dyn. using LIE');   xlabel('time(s)'); ylabel('torque(Nm)');
legend('Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6');

subplot(2,3,5);
plot(1:nData,tau_Result_RTB);
title('Inv. Dyn. using RTB');   xlabel('time(s)'); ylabel('torque(Nm)');
legend('Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6');

subplot(2,3,6);
plot(1:nData, tau_diff);
title('Diff. b/w Lie Dyn & N-E Dyn.');  xlabel('time(s)'); ylabel('torque(Nm)');
legend('Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6');

%% 5. Simulating robot motions
% Robot animation using RTB
% figure();   robotRTB.plot(q_query);

% Robot animation using plot3
% Put the robot model as a cell array
myModel = cell(1,1);        myModel{1,1} = robotModel;
bShowFrame = 1;     % 1:Show the frames  0: Do not show them
bEETraj = [1];      % 1:Show the end-effector trajectories  0 : Do not show them 
% axisRange = [-0.8 0.8 -0.8 0.8 -0.8 0.8];       % Axis range for plotting
axisRange = FindAxisRange(myModel);             % Find appropriate axes for plotting motion               
DisplayModel(myModel, axisRange, bShowFrame, bEETraj);   % Display the motion
