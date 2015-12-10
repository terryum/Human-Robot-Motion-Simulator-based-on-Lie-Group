
%% X-axis rotation

% made by Terry Taewoong Um (terry.t.um@gmail.com)
% Adaptive Systems Lab., University of Waterloo

% [input] theta : scalar 
% [output] R_out : 3*3 SO(3)

function R_out = rotX(theta)
if(size(theta)==[1,1])
    R_out = [1      0       0
            0 cos(theta) -sin(theta);
            0 sin(theta) cos(theta)];
else
    error('dimension error');
end
end

    
    