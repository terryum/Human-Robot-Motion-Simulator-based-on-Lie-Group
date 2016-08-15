
%% Large adjoint mapping

% made by Terry Taewoong Um (terry.t.um@gmail.com)
% Adaptive Systems Lab., University of Waterloo
% Modified from the SNU robotics lab's code (Syungkwon Ra (dearLenin@gmail.com))

% [input] T : 4*4 SE(3), s : 6*1 se(3) 
% [output] R : 3*3 SO(3)

function R_out = Adj(T, s)
    if (size(T) == [4,4] & size(s) == [6,1])
        T_temp = [     T(1,1) * s(1) + T(1,2) * s(2) + T(1,3) * s(3); 
                    T(2,1) * s(1) + T(2,2) * s(2) + T(2,3) * s(3); 
                    T(3,1) * s(1) + T(3,2) * s(2) + T(3,3) * s(3) ];
        R_out = [   T_temp(1); T_temp(2); T_temp(3);
                T(2,4) * T_temp(3) - T(3,4) * T_temp(2) + T(1,1) * s(4) + T(1,2) * s(5) + T(1,3) * s(6);
                T(3,4) * T_temp(1) - T(1,4) * T_temp(3) + T(2,1) * s(4) + T(2,2) * s(5) + T(2,3) * s(6);
                T(1,4) * T_temp(2) - T(2,4) * T_temp(1) + T(3,1) * s(4) + T(3,2) * s(5) + T(3,3) * s(6)];
        return;
    else
        error('dimension error');
    end
end
            
