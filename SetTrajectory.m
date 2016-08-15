
%% SetTrajectory : Set the desired trajectory as you want

% made by Terry Taewoong Um (terry.t.um@gmail.com)
% Adaptive Systems Lab., University of Waterloo

function [q_query dq_query ddq_query] = SetTrajectory(nDOF)

    % In the example, we will use 1001 sinusoidal samples (10s, 100Hz)
    t_total = 10;              
    freq = 100;
    nData = t_total*freq + 1;
    tt = linspace(0, t_total, nData)';
    dt = tt(2,1)-tt(1,1);

    A = [0.3; 0.5; 0.4; 0.2; 0.7; 0.1];  % arbitrary amplitude
    w = [1.2; 0.8; 1.2; 0.4; 0.9; 0.1];   % arbitrary frequency

    q_query = zeros(nData,nDOF);    dq_query = zeros(nData,nDOF);    ddq_query = zeros(nData,nDOF);
    bSine = 1;      % Use sine and cosine functions alternatively
    for ii=1:nDOF
        if bSine == 1;
            q_query(:,ii) = A(ii,1)*sin(w(ii,1)*tt);
            dq_query(:,ii) = w(ii,1)*A(ii,1)*cos(w(ii,1)*tt);
            ddq_query(:,ii) = -w(ii,1)*w(ii,1)*A(ii,1)*sin(w(ii,1)*tt);
        else
            q_query(:,ii) = A(ii,1)*cos(w(ii,1)*tt);
            dq_query(:,ii) = -w(ii,1)*A(ii,1)*sin(w(ii,1)*tt);
            ddq_query(:,ii) = -w(ii,1)*w(ii,1)*A(ii,1)*cos(w(ii,1)*tt);
        end
        bSine = -1*bSine;
    end
            
%% You can also use other trajectories
% e.g. Simple trajectory  q1' = 1, others = 0
    % q_query = [linspace(0, t_total,nData)' zeros(nData,nDOF-1)];
    % dq_query = [ones(1001,1) zeros(1001,nDOF-1)];
    % ddq_query = zeros(1001,nDOF);
end