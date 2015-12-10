
%% Model_from_RTB : Get a Lie group based robot model from the robotics toolbox model

% made by Terry Taewoong Um (terry.t.um@gmail.com)
% Adaptive Systems Lab., University of Waterloo

% [variables]
%     robotModel = struct('nLink', nLink, 'VecGravity', VecGravity, 'T_JointHome', zeros(4,4,nLink), 'T_JointJoint', zeros(4,4,nLink-1),...
%                         ,'T_COMCOM', zeros(4,4,robotModel.nLink-1), 'T_JointCOM', zeros(4,4,robotModel.nLink-1), 'S_Home', zeros(6,1,robotModel.nLink-1), 'Inertia', zeros(6, 6, robotModel.nLink-1));  

% - All 6 by 1 spatial vectors consist of [angular motion; linear motion]
%   (e.g. V = [w1; w2; w3; v1; v2; v3])
% - Link 1 is reserved for the ground link

% nLink        : # of link including ground (e.g. two-bar linkage = 3)
% VecGravity   : 6 by 1 Gravity vector or Initial accelerlation vector (e.g. [0;0;0;0;0;9.81])

% T_JointHome[nLink] : Frame of each joint (including the EE frame) at the home position seen from {base}
% [1]:1st joint, [2]:2nd joint, ..., [nLink-1]:the last joint, [nLink]: End effector(EE) frame

% T_JointJoint[nLink] : Transformation matrices between joints seen from {i}
% [1]:from ground to 1st joint, ..., [nLink]:from the last joint to the end effector frame

% T_COMCOM[nLink-1] : Transformation matrices between center of masses(COMs) seen from {i}
% [1]:from ground to 1st link's COM, ..., [nLink-1]:from (n-1)th link's COM to (n)th link's COM

% T_JointCOM[nLink-1] : Transformation matrices from each each joint to the link's COM
% [1]:from 1st joint to the 1st link's COM, ..., [robotModel.nLink-1]: from the last joint to the last link's COM

% S_Home[robotModel.nLink-1] : 6 by 1 se(3) of each joint seen from {base}
% Inertia                    : 6 by 6 generalized inertia seen from {i} 

% 1. Initialize robot model variables
% 2. Get SE(3) from the DH parameters of the robotics toolbox model
% 3. Calculate the generalized inertia

%% 1. Initialize robot model variables
function robotModel = Model_from_RTB(robot)
    nLink = robot.n + 1;        % links + ground

    robotModel = struct('nLink', nLink, 'VecGravity', zeros(6,1), 'T_JointHome', zeros(4,4,nLink), 'T_JointJoint', zeros(4,4,nLink-1), ...
                        'T_COMCOM', zeros(4,4,nLink-1), 'T_JointCOM', zeros(4,4,nLink-1), 'S_Home', zeros(6,1,nLink-1), 'Inertia', zeros(6,6,nLink-1));  
    robotModel.VecGravity = [0 0 0 robot.gravity(1,1) robot.gravity(2,1) robot.gravity(3,1)];     % if gravity exists
    % robotModel.VecGravity = zeros(6,1);       % if no gravity exists

%% Get SE(3) from the DH parameters of the robotics toolbox model

    T_move = eye(4);
    robotModel.T_JointHome(:,:,1) = eye(4);
    robotModel.T_JointJoint(:,:,1) = eye(4); 
    A_se3 = [0;0;1;0;0;0];      % All joint axes are z-direction w.r.t the local frame
    T_temp = eye(4);

    for ii=1:robotModel.nLink-1
        %   theta = robot.theta(1,ii);      % when there is offset in theta
        theta = 0;                      
        dd = robot.d(1,ii);
        aa = robot.a(1,ii);             
        alpha = robot.alpha(1,ii);

        % SE(3) from the joint {ii} to the joint {ii+1}                      
        % original DH (1955)
        robotModel.T_JointJoint(:,:,ii+1) = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) aa*cos(theta);
                          sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) aa*sin(theta);
                          0   sin(alpha)  cos(alpha)  dd;
                          0 0 0 1]; 
                      
        T_temp = T_temp * robotModel.T_JointJoint(:,:,ii);  % T_temp = From the base to the joint
        robotModel.S_Home(:,ii) = Adj(T_temp, A_se3);       % Axis w.r.t. {i} -> Axis w.r.t. {base}
 
        % modified DH (1989)
    %     T_DH(:,:,ii+1) = [cos(theta) -sin(theta) 0 aa;
    %                       sin(theta)*cos(aa) cos(theta)*cos(alpha) -sin(alpha) -dd*sin(alpha);
    %                       sin(theta)*sin(alpha) cos(theta)*sin(alpha) cos(alpha)  dd*cos(alpha);
    %                       0 0 0 1];  

% T_JointHome / T_JointCOM / T_COMCOM can be calculated from T_JointHome and T_JointJoint
        robotModel.T_JointHome(:,:,ii+1) = robotModel.T_JointHome(:,:,ii) * robotModel.T_JointJoint(:,:,ii+1);
        robotModel.T_JointCOM(:,:,ii) = robotModel.T_JointJoint(:,:,ii+1) * RP01(eye(3),robot.links(ii).r');
        robotModel.T_JointCOM(1:3,1:3,ii) = eye(3);     % The orientation of the COM frame is same as that of the joint frame
    end
    robotModel.T_COMCOM(:,:,1) = robotModel.T_JointHome(:,:,1)*robotModel.T_JointCOM(:,:,1); 
    for ii=2:robotModel.nLink-1
        robotModel.T_COMCOM(:,:,ii) = invSE3(robotModel.T_JointHome(:,:,ii-1)*robotModel.T_JointCOM(:,:,ii-1)) ... 
                                * robotModel.T_JointHome(:,:,ii)*robotModel.T_JointCOM(:,:,ii); 
    end


%% 3. Calculate the generalized inertia 
% The generalized inertia is 6 by 6 matrix of the form [ I 0; 0 m1 ] 
% Link1 is reserved for the ground
    for ii=2:robotModel.nLink
        II = robotModel.T_JointJoint(1:3,1:3,ii)'*robot.links(ii-1).I*robotModel.T_JointJoint(1:3,1:3,ii);
        robotModel.Inertia(:,:,ii) = [ II zeros(3); 
                         zeros(3) robot.links(ii-1).m*eye(3)];
    end
end
