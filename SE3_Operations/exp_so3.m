
%% Exponential mapping for so(3)

% made by Terry Taewoong Um (terry.t.um@gmail.com)
% Adaptive Systems Lab., University of Waterloo
% Modified from the SNU robotics lab's code (Syungkwon Ra (dearLenin@gmail.com))

% [input] w : 3*1 so(3) 
% [output] R : 3*3 SO(3)

function R = exp_so3(w)

TINY=1e-14;

    if(size(w,1)==1 || size(w,2)==1)
        theta = norm(w);
        if ( abs(theta) < TINY ) 
            R=eye(3);
            return 
        end
        w=w/theta;
        st = sin(theta);
        ct = cos(theta);
        vt = 1.0 - ct;
        t0 = w(3) * st;
        t1 = w(2) * st;
        t2 = w(1) * st;

        w0=w(1);w1=w(2);w2=w(3);
        R(1,1)=w0 * w0 * vt + ct;
        R(2,1)=w0 * w1 * vt + t0;
        R(3,1)=w0 * w2 * vt - t1;
        R(1,2)=w0 * w1 * vt - t0;
        R(2,2)=w1 * w1 * vt + ct;
        R(3,2)=w1 * w2 * vt + t2;
        R(1,3)=w0 * w2 * vt + t1;
        R(2,3)=w1 * w2 * vt - t2;
        R(3,3)=w2 * w2 * vt + ct;
        
    elseif(size(w,1)==3 || size(w,2)==3)
        nData = size(w,1); 
        R = zeros(3,3,nData);
        for ii=1:nData
            ww = w(ii,:);
            theta = norm(ww);
            if ( abs(theta) < TINY ) 
                R=eye(3);
                return 
            end
            ww=ww/theta;
            st = sin(theta);
            ct = cos(theta);
            vt = 1.0 - ct;
            t0 = ww(3) * st;
            t1 = ww(2) * st;
            t2 = ww(1) * st;

            w0=ww(1);w1=ww(2);w2=ww(3);
            R(1,1,ii)=w0 * w0 * vt + ct;
            R(2,1,ii)=w0 * w1 * vt + t0;
            R(3,1,ii)=w0 * w2 * vt - t1;
            R(1,2,ii)=w0 * w1 * vt - t0;
            R(2,2,ii)=w1 * w1 * vt + ct;
            R(3,2,ii)=w1 * w2 * vt + t2;
            R(1,3,ii)=w0 * w2 * vt + t1;
            R(2,3,ii)=w1 * w2 * vt - t2;
            R(3,3,ii)=w2 * w2 * vt + ct;
        end
    else
        error('SO3(w) w is not 3 x 1')
    end

end