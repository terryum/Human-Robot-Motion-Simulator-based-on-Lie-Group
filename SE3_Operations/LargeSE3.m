
%% Exponential mapping for se(3)

% made by Terry Taewoong Um (terry.t.um@gmail.com)
% Adaptive Systems Lab., University of Waterloo
% Modified from the SNU robotics lab's code (Syungkwon Ra (dearLenin@gmail.com))

% [input] S : 6*1 se(3) 
% [output] T : 4*4 SE(3)

function T = LargeSE3(S)
if(size(S)==[6,1] | size(S)==[1,6])

    SCALAR_ONE=1;
    SCALAR_HALF=0.5;
    LIE_EPS=1e-14;

    s2= [S(1)^2 ; S(2)^2 ; S(3)^2];
    theta = sqrt(s2(1) + s2(2) + s2(3));

    if ( theta < LIE_EPS )
        st_t = SCALAR_ONE - theta * theta / 6.0;
        ct_t = SCALAR_HALF - theta * theta / 24.0;
        vt_t = (S(1) * S(4) + S(2) * S(5) + S(3) * S(6)) * (SCALAR_ONE - theta * theta / 20.0) / 6.0;
    else
        itheta = SCALAR_ONE / theta;
        st_t = sin(theta) * itheta;
        itheta = itheta*itheta;
        ct_t = (SCALAR_ONE - cos(theta)) * itheta;
        vt_t = (S(1) * S(4) + S(2) * S(5) + S(3) * S(6)) * (SCALAR_ONE - st_t) * itheta;
    end

    T=zeros(4,4);

        T(1,1) = SCALAR_ONE - ct_t * (s2(2) + s2(3));
        T(2,1) = ct_t * S(1) * S(2) + st_t * S(3);
        T(3,1) = ct_t * S(1) * S(3) - st_t * S(2); 
        T(1,2) = ct_t * S(1) * S(2) - st_t * S(3);
        T(2,2) = SCALAR_ONE - ct_t * (s2(1) + s2(3));
        T(3,2) = ct_t * S(2) * S(3) + st_t * S(1); 
        T(1,3) = ct_t * S(1) * S(3) + st_t * S(2);
        T(2,3) = ct_t * S(2) * S(3) - st_t * S(1);
        T(3,3) = SCALAR_ONE - ct_t * (s2(1) + s2(2));
        T(1,4) = st_t * S(4) + vt_t * S(1) + ct_t * (S(2) * S(6) - S(3) * S(5));
        T(2,4) = st_t * S(5) + vt_t * S(2) + ct_t * (S(3) * S(4) - S(1) * S(6));
        T(3,4) = st_t * S(6) + vt_t * S(3) + ct_t * (S(1) * S(5) - S(2) * S(4));
        T(4,4) = 1;
else
    error('SE3(s) input s is not 6 x 1');
end
    
    