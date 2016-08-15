
%% Dual large adjoint mapping

% made by Terry Taewoong Um (terry.t.um@gmail.com)
% Adaptive Systems Lab., University of Waterloo
% Modified from the SNU robotics lab's code (Syungkwon Ra (dearLenin@gmail.com))

% [input] T : 4*4 SE(3). s : 6*1 se(3) 
% [output] S_out : 6*1 se(3)

function S_out = dAdj(T, s)
    if (size(T) == [4,4] & size(s) == [6,1])
        tmp = [ s(1) - T(2,4) * s(6) + T(3,4) * s(5); 
                s(2) - T(3,4) * s(4) + T(1,4) * s(6); 
                s(3) - T(1,4) * s(5) + T(2,4) * s(4) ];
        S_out = [   T(1,1) * tmp(1) + T(2,1) * tmp(2) + T(3,1) * tmp(3);
                T(1,2) * tmp(1) + T(2,2) * tmp(2) + T(3,2) * tmp(3);
                T(1,3) * tmp(1) + T(2,3) * tmp(2) + T(3,3) * tmp(3);
                T(1,1) * s(4) + T(2,1) * s(5) + T(3,1) * s(6);
                T(1,2) * s(4) + T(2,2) * s(5) + T(3,2) * s(6);
                T(1,3) * s(4) + T(2,3) * s(5) + T(3,3) * s(6) ];
        return;
    else
        error('dimension error');
    end
end