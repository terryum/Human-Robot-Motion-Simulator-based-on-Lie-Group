
%% Inverse dynamics of a grounded open chain
% External forces except for gravitational force are not considered in this case

% made by Terry Taewoong Um (terry.t.um@gmail.com)
% Adaptive Systems Lab., University of Waterloo

% [input] q_query, dq_query, ddq_query : n*1 pos/vel/acc. input
% [output] tau_out : n*1 torque output

% - All 6 by 1 spatial vectors are represented as [angular motion; linear motion]
%   (e.g. V = [w1; w2; w3; v1; v2; v3])
% - Link 1 is reserved for the ground link

% 1. Initialize local variables

function tau_out = InvDyn_Serial(robotModel, q_query, dq_query, ddq_query)
%% 1. Initialize local variables
    V = zeros(6, 1, robotModel.nLink);    % spatial velocity (w.r.t the body frame)
    dV = zeros(6, 1, robotModel.nLink);   % spatial accelleration (w.r.t the body frame)
    F = zeros(6, 1, robotModel.nLink);    % spatial force [moment; force]
    tau = zeros(robotModel.nLink-1,1);      % torque of each joint (scalar)
    T_adj = zeros(4, 4, robotModel.nLink);  % T from i-1 joint to i after moving q_i
    inv_T_adj = zeros(4, 4, robotModel.nLink);   % inverse of T_adj

% Set initial velocity and acceleration
    V(:, :, 1) = zeros(6,1);
    dV(:, :, 1) = [0; 0; 0; 0; 0; 9.81];        % gravity
%     dV(:, :, 1) = zeros(6,1);                   % no gravity

%% 2. Forward recursion : Propagating veclocities and acceleration
    for ii = 2 : robotModel.nLink
        % Represent se(3) in local frame  
        w_axis = [0;0;1];                               % Z axes always indicate the rotational axes in local frame  
        q_onAxis = -robotModel.T_JointCOM(1:3,4,ii-1);  % Joints are located at -Dist_COM from the COM
        A_axis(:,ii-1) = [w_axis; -cross(w_axis, q_onAxis)];        
        % Calculating velocities and accelerations
        inv_T_adj(:, :, ii) = invSE3(robotModel.T_COMCOM(:, :, ii-1) * LargeSE3(A_axis(:,ii-1) * q_query(ii-1)));
        V(:, :, ii) = Adj(inv_T_adj(:, :, ii), V(:, :, ii-1)) + A_axis(:,ii-1) * dq_query(ii-1);
        dV(:, :, ii) = A_axis(:,ii-1) * ddq_query(ii-1) + Adj(inv_T_adj(:, :, ii), dV(:, :, ii-1)) + ...
                        ad(V(:, :, ii), A_axis(:,ii-1) * dq_query(ii-1));
    end
%% 3. Backward recursion : Back propagating forces and torques
% External forces except for gravitational force are not considered in this case
    for ii = robotModel.nLink:-1:2
        F(:, :, ii) = robotModel.Inertia(:, :, ii) * dV(:, :, ii) - dad(V(:, :, ii), robotModel.Inertia(:, :, ii) * V(:, :, ii));
        if ii ~= robotModel.nLink
            F(:, :, ii) = F(:, :, ii) + dAdj(inv_T_adj(:, :, ii+1), F(:, :, ii+1));
        end
        tau(ii-1) = A_axis(:,ii-1)' * F(:, :, ii);
    end
    tau_out = tau;
end

