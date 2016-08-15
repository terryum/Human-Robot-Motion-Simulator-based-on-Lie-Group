
%% Small adjoint mapping

% made by Terry Taewoong Um (terry.t.um@gmail.com)
% Adaptive Systems Lab., University of Waterloo
% Modified from the SNU robotics lab's code (Syungkwon Ra (dearLenin@gmail.com))

% [input] s1, s2 : 6*1 se(3) 
% [output] S_out : 6*1 se(3)

function S_out = ad(s1, s2)
    if (size(s1) == [6,1] & size(s2) == [6,1])
        S_out = [   s1(2) * s2(3) - s1(3) * s2(2);
                s1(3) * s2(1) - s1(1) * s2(3);
                s1(1) * s2(2) - s1(2) * s2(1);
                s1(2) * s2(6) - s1(3) * s2(5) - s2(2) * s1(6) + s2(3) * s1(5);
                s1(3) * s2(4) - s1(1) * s2(6) - s2(3) * s1(4) + s2(1) * s1(6);
                s1(1) * s2(5) - s1(2) * s2(4) - s2(1) * s1(5) + s2(2) * s1(4) ];
        return;
    else
        error('dimension error');
    end
end