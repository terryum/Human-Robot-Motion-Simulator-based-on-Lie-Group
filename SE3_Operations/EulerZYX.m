
%% Euler Angle Rotation : Z - Y - X

% made by Terry Taewoong Um (terry.t.um@gmail.com)
% Adaptive Systems Lab., University of Waterloo
% Modified from the SNU robotics lab's code (Syungkwon Ra (dearLenin@gmail.com))

% [input] q_z, q_y, q_x : rotation angles 
% [output] R : 3*3 SO(3)

function R = EulerZYX(q_z, q_y, q_x);

cz = cos(q_z);      sz = sin(q_z);
cy = cos(q_y);      sy = sin(q_y);
cx = cos(q_x);      sx = sin(q_x);

R = [ cz * cy  cz * sy * sx - sz * cx  cz * sy * cx + sz * sx;
      sz * cy  sz * sy * sx + cz * cx  sz * sy * cx - cz * sx;
      - sy     cy * sx                 cy * cx                ];
