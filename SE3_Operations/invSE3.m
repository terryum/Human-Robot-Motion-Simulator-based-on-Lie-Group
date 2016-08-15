
%% Inversion of SE(3)

% made by Terry Taewoong Um (terry.t.um@gmail.com)
% Adaptive Systems Lab., University of Waterloo
% Modified from the SNU robotics lab's code (Syungkwon Ra (dearLenin@gmail.com))

% [input] T_in : 4*4 SE(3) 
% [output] T_out : 4*4 SE(3)

function T_out = invSE3(T_in)
    R = T_in(1:3,1:3);
    P = T_in(1:3,4);
    T_out=[transpose(R) -transpose(R)*P;
            0 0 0 1];
if(size(T_in)==[4,4])
    
else
    error('dimension error');
end
end
