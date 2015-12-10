
%% Dual small adjoint mapping

% made by Terry Taewoong Um (terry.t.um@gmail.com)
% Adaptive Systems Lab., University of Waterloo
% Modified from the SNU robotics lab's code (Syungkwon Ra (dearLenin@gmail.com))

% [input] s1, s2 : 6*1 se(3) 
% [output] S_out : 6*1 se(3)

function S_out = dad(s, t)

    if (size(s) == [6,1] & size(t) == [6,1])
        S_out = [   t(2) * s(3) - t(3) * s(2) + t(5) * s(6) - t(6) * s(5);
                t(3) * s(1) - t(1) * s(3) + t(6) * s(4) - t(4) * s(6);
                t(1) * s(2) - t(2) * s(1) + t(4) * s(5) - t(5) * s(4);
                t(5) * s(3) - t(6) * s(2);
                t(6) * s(1) - t(4) * s(3);
                t(4) * s(2) - t(5) * s(1) ];
        return; 
    else
        error('dimension error');
    end
end