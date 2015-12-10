
%% Z-axis rotation

% made by Terry Taewoong Um (terry.t.um@gmail.com)
% Adaptive Systems Lab., University of Waterloo

% [input] theta : scalar 
% [output] R_out : 3*3 SO(3)

function R_out = rotZ(theta)
if(size(theta)==[1,1])
    R_out = [cos(theta) -sin(theta) 0;
             sin(theta) cos(theta)  0
                0           0       1];
else
    error('dimension error');
end
end

    
    