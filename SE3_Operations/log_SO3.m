
% made by Terry Taewoong Um (terry.t.um@gmail.com)
% Adaptive Systems Lab., University of Waterloo

function output = log_SO3(input)

    if (size(input) == [3,3])
        theta = acos((trace(input)-1)/2);
        if abs(theta) < 10^-10
            output = zeros(3,1);
        elseif trace(input) == -1
            output = [(input(1,1)+1)/2; (input(2,2)+1)/2; (input(3,3)+1)/2];
            output = sqrt(output);
            output = output/norm(output)*pi;
        else
            w_hat = (input-input')/(2*sin(theta))*theta;
            output(1,1) = w_hat(3,2);
            output(2,1) = w_hat(1,3);
            output(3,1) = w_hat(2,1);
        end    
        return;
    else
        error('check the size of matrix');
    end
end