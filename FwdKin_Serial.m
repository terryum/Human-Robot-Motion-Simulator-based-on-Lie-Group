
%% Forward Kinematics of a grounded open chain

% made by Terry Taewoong Um (terry.t.um@gmail.com)
% Adaptive Systems Lab., University of Waterloo

% [input] q_query : n*1 angle input 
% [output] T_out : 4*4 SE(3) for the end-effector

function [T_EE T_AllJoint] = FwdKin_Serial(robotModel, q_query)
if size(q_query,2) ~= robotModel.nLink-1
    error('dim(q, qd, qdd) is not consistent with the DOF of the robot model');
    return;
end    
%   T = e^S1q1 * e^S2q2 * ... * e^Snqn * T_FromBaseToEE  
    T_out_temp = eye(4);
    T_AllJoint(:,:,1) = robotModel.T_JointHome(:,:,1);
    for ii=1:robotModel.nLink-1
        T_out_temp = T_out_temp * exp_se3(robotModel.S_Home(:,ii)*q_query(1,ii));
        T_AllJoint(:,:,ii+1) = T_out_temp * robotModel.T_JointHome(:,:,ii+1);
    end
    T_EE = T_AllJoint(:,:,robotModel.nLink);  
end

    
    