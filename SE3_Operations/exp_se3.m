
%% Exponential mapping for se(3)

% made by Terry Taewoong Um (terry.t.um@gmail.com)
% Adaptive Systems Lab., University of Waterloo
% Modified from the SNU robotics lab's code (Syungkwon Ra (dearLenin@gmail.com))

% [input] S : 6*1 se(3) 
% [output] T_out : 4*4 SE(3)

function T_out = exp_se3(S)
if(size(S)==[6,1])

    theta_close_zero = 1e-14;

    s2= [S(1)^2 ; S(2)^2 ; S(3)^2];
    theta = sqrt(s2(1) + s2(2) + s2(3));

    if ( theta < theta_close_zero )
        st_t = 1 - theta * theta / 6.0;
        ct_t = 0.5 - theta * theta / 24.0;
        vt_t = (S(1) * S(4) + S(2) * S(5) + S(3) * S(6)) * (1 - theta * theta / 20.0) / 6.0;
    else
        itheta = 1 / theta;
        st_t = sin(theta) * itheta;
        itheta = itheta*itheta;
        ct_t = (1 - cos(theta)) * itheta;
        vt_t = (S(1) * S(4) + S(2) * S(5) + S(3) * S(6)) * (1 - st_t) * itheta;
    end

    T = zeros(4,4);

    T(1,1) = 1 - ct_t * (s2(2) + s2(3));
    T(2,1) = ct_t * S(1) * S(2) + st_t * S(3);
    T(3,1) = ct_t * S(1) * S(3) - st_t * S(2); 
    T(1,2) = ct_t * S(1) * S(2) - st_t * S(3);
    T(2,2) = 1 - ct_t * (s2(1) + s2(3));
    T(3,2) = ct_t * S(2) * S(3) + st_t * S(1); 
    T(1,3) = ct_t * S(1) * S(3) + st_t * S(2);
    T(2,3) = ct_t * S(2) * S(3) - st_t * S(1);
    T(3,3) = 1 - ct_t * (s2(1) + s2(2));
    T(1,4) = st_t * S(4) + vt_t * S(1) + ct_t * (S(2) * S(6) - S(3) * S(5));
    T(2,4) = st_t * S(5) + vt_t * S(2) + ct_t * (S(3) * S(4) - S(1) * S(6));
    T(3,4) = st_t * S(6) + vt_t * S(3) + ct_t * (S(1) * S(5) - S(2) * S(4));
    T(4,4) = 1;
        
    T_out = T;
else
    error('dimension error');
end
    
    