
%% Make a SE(3) with a rotation matrix and a position vector

% made by Terry Taewoong Um (terry.t.um@gmail.com)
% Adaptive Systems Lab., University of Waterloo

% [input] R : 3*3 rotation matrix, P: 3*1 position vector
% [output] T : 4*4 SE(3)

function T_out = RP01(R, P)
if size(R)==[3,3] & size(P)==[3,1]
    T_out = [R(1,1) R(1,2) R(1,3) P(1,1); ...
        R(2,1) R(2,2) R(2,3) P(2,1); ...
        R(3,1) R(3,2) R(3,3) P(3,1); ...
        0 0 0 1];
elseif size(R)==[3,3] & size(P)==[1,3]
    T_out = [R(1,1) R(1,2) R(1,3) P(1,1); ...
        R(2,1) R(2,2) R(2,3) P(1,2); ...
        R(3,1) R(3,2) R(3,3) P(1,3); ...
        0 0 0 1];
else
    error('dimension error');
end
    
    