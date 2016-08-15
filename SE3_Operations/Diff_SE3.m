
%% Return the angular and position difference of two SE(3)

% made by Terry Taewoong Um (terry.t.um@gmail.com)
% Adaptive Systems Lab., University of Waterloo

% [input] T1, T2 : 4*4 SE(3) 
% [output] ang_diff, pos_diff : scalars

function [ang_diff pos_diff] = Diff_SE3(T1, T2)

if ((size(T1) == [4,4]) & size(T2) == [4,4])
    ang_diff = norm(logm(T1(1:3, 1:3)'*T2(1:3, 1:3)), 2);
    pos_diff = norm(T1(1:3,4) - T2(1:3, 4), 2);
else
    error('dimension error');
end
