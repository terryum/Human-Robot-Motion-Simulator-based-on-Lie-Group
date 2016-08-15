
%% Y-axis rotation

% made by Terry Taewoong Um (terry.t.um@gmail.com)
% Adaptive Systems Lab., University of Waterloo

% [input] theta : scalar 
% [output] R_out : 3*3 SO(3)

function R_out = rotY(theta)
if(size(theta)==[1,1])
    R_out = [cos(theta) 0 sin(theta);
                0       1       0;
            -sin(theta) 0 cos(theta)];
else
    error('dimension error');
end
end

    
    