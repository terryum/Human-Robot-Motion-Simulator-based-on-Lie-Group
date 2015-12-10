
%% Hand-written Robot Model : Puma560_3DOF

% made by Terry Taewoong Um (terry.t.um@gmail.com)
% Adaptive Systems Lab., University of Waterloo

% - All 6 by 1 spatial vectors consist of [angular motion; linear motion]
%   (e.g. V = [w1; w2; w3; v1; v2; v3])
% - Link 1 is reserved for the ground link

% 1. Set model parameters
% 2. Others will be automatically calculated 

function robotModel = Model_3DOF_Puma560()  
    %% 1. Set model parameters
    % nLink, VecGravity, T_JointJoint, S_Home, T_Inertial
    nLink = 4;
    robotModel = struct('nLink', nLink, 'VecGravity', zeros(6,1), 'T_JointHome', zeros(4,4,nLink), 'T_JointJoint', zeros(4,4,nLink-1), ...
                        'T_COMCOM', zeros(4,4,nLink-1), 'T_JointCOM', zeros(4,4,nLink-1), 'S_Home', zeros(6,1,nLink-1), 'Inertia', zeros(6,6,nLink-1));  
    robotModel.VecGravity = [0 0 0 0 0 9.81];     % if gravity exists
    % robotModel.VecGravity = zeros(6,1);       % if no gravity
    
    % Transformation matrices between joints (The last joint is EE)
    robotModel.T_JointJoint(:,:,1) = eye(4);
    robotModel.T_JointJoint(:,:,2) = RP01([1 0 0; 0 0 -1; 0 1 0], zeros(3,1));
    robotModel.T_JointJoint(:,:,3) = RP01(eye(3), [0.4318; 0; 0;]);
    robotModel.T_JointJoint(:,:,4) = RP01([1 0 0; 0 0 1; 0 -1 0], [0.0203; 0; 0.15;]);
    
    % Set the joint axes at the home position (seen from {base})
    % [w v] w:rotation axis, v:cross(q,w) where q is a point on the axis
    robotModel.S_Home(:,:,1) = [0; 0; 1; 0; 0; 0];
    robotModel.S_Home(:,:,2) = [0; -1; 0; 0; 0; 0];
    robotModel.S_Home(:,:,3) = [0; -1; 0; 0; 0; -0.4318];   
      
    % Set the generalized inertia which is 6 by 6 matrix [ I 0; 0 m1 ] 
    % Link1 is reserved for the ground
    robotModel.Inertia(:,:,2) = diag([0 0 0.35 0 0 0]);
    robotModel.Inertia(:,:,3) = diag([0.13 0.524 0.539 17.4 17.4 17.4]);
    robotModel.Inertia(:,:,4) = diag([0.066 0.0125 0.086 4.8 4.8 4.8]);
    
    % Set the vectors from each joint to each COM
    robotModel.T_JointCOM(:,:,1) = eye(4);
    robotModel.T_JointCOM(:,:,2) = RP01(eye(3), [0.0680; 0.0060; 0.2275]);
    robotModel.T_JointCOM(:,:,3) = RP01(eye(3), [0; 0.07; 0.1641]);
    
%% 2. Automatic calculation for other parameters
    robotModel.T_JointHome(:,:,1) = eye(4);
    for ii=1:nLink-1
        robotModel.T_JointHome(:,:,ii+1) = robotModel.T_JointHome(:,:,ii) * robotModel.T_JointJoint(:,:,ii+1);
    end
    robotModel.T_COMCOM(:,:,1) = robotModel.T_JointHome(:,:,1)*robotModel.T_JointCOM(:,:,1); 
    for ii=2:robotModel.nLink-1
        robotModel.T_COMCOM(:,:,ii) = invSE3(robotModel.T_JointHome(:,:,ii-1)*robotModel.T_JointCOM(:,:,ii-1)) ... 
                                * robotModel.T_JointHome(:,:,ii)*robotModel.T_JointCOM(:,:,ii); 
    end
end
