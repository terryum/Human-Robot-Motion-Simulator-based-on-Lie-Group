
%% Euler Angle Rotation : X - Y - Z

% made by Terry Taewoong Um (terry.t.um@gmail.com)
% Adaptive Systems Lab., University of Waterloo
% Modified from the SNU robotics lab's code (Syungkwon Ra (dearLenin@gmail.com))

% [input] q_x, q_y, q_z : rotation angles 
% [output] R : 3*3 SO(3)

function R = EulerXYZ(q_x, q_y, q_z);

cz = cos(q_z);  sz = sin(q_z);  
cy = cos(q_y);  sy = sin(q_y);  
cx = cos(q_x);  sx = sin(q_x);

R = [ cy*cz  -cy*sz sy;
      cx*sz+cz*sx*sy cx*cz-sx*sy*sz -cy*sx;
      sx*sz-cx*cz*sy cz*sx+cx*sy*sz cx*cy ];

% R = [cx*cy cx*sy*sz-cz*sx   sx*sz+cx*cz*sy;
%      cy*sx cx*cz+sx*sy*sz   cz*sx*sy-cx*sz;
%        -sy cy*sz            cy*cz];


end