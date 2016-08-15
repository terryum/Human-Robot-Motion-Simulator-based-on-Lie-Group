
% made by Terry Taewoong Um (terry.t.um@gmail.com)
% Adaptive Systems Lab., University of Waterloo

% Error when nPi cases

function output = log_SO3(input)
    
    % For a single pose     
    if (size(size(input),2)==2 && size(input,1) == 3 &&  size(input,2) == 3)
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
    
    % For a pose trajectory
    % (Works only when the input data are smoothly passing 2n*pi points)
    elseif ((size(size(input),2)==3) && size(input,1) == 3 && size(input,2) == 3)
        nData = size(input,3);
        output = zeros(nData,3);
        theta = zeros(nData,1);
 
        input2 = input(:,:,1);
        theta(1,1) = acos((trace(input2)-1)/2);          
        if abs(theta(1,1)) < 10^-10
            output(1,:) = zeros(1,3);
        elseif trace(input2) == -1
            % Dim Error
            output2 = [(input2(1,1)+1)/2; (input2(2,2)+1)/2; (input2(3,3)+1)/2];
            output2 = sqrt(output2);
            output2 = output/norm(output)*pi;
            output(1,:) = output2';
        else
            w_hat = (input2-input2')/(2*sin(theta(1,1)))*theta(1,1);
            output(1,1) = w_hat(3,2);
            output(1,2) = w_hat(1,3);
            output(1,3) = w_hat(2,1);
        end 
        
        theta_offset = 0;
        output2 = zeros(3,1);
        for ii = 2:nData
            if ii==100
                aa=0;
            end
            input2 = input(:,:,ii);
            if mod(theta_offset,2)==0
                theta(ii,1) = acos((trace(input2)-1)/2) + theta_offset*pi; 
            else
                 theta(ii,1) = pi-acos((trace(input2)-1)/2) + theta_offset*pi;
            end
            if abs(mod(theta(ii,1),2*pi)) < 10^-10
                output(ii,:) = zeros(1,3);
            elseif trace(input2) == -1
                output2 = [(input2(1,1)+1)/2; (input2(2,2)+1)/2; (input2(3,3)+1)/2];
                output2 = sqrt(output(ii,:));
                output2 = output(ii,:)/norm(output(ii,:))*pi;
                output(ii,:) = output2';
            else
                w_hat = (input2-input2')/(2*sin(theta(ii,1)))*theta(ii,1);
                output2(1,1) = w_hat(3,2);
                output2(2,1) = w_hat(1,3);
                output2(3,1) = w_hat(2,1);
                if (output(ii-1,:)*output2 > -0.2)
                    output(ii,:) = output2';
                else
                    if mod(theta(ii,1),pi)> pi/2  
                        theta_offset = theta_offset+1;
                    else
                        theta_offset = theta_offset-1;   
                    end
                    
                    if mod(theta_offset,2)==0
                        theta(ii,1) =  acos((trace(input2)-1)/2) + theta_offset*pi; 
                    else
                        theta(ii,1) = pi - acos((trace(input2)-1)/2) + theta_offset*pi;
                    end
                        w_hat = (input2-input2')/(2*sin(theta(ii,1)))*theta(ii,1);
                end
                output(ii,1) = w_hat(3,2);
                output(ii,2) = w_hat(1,3);
                output(ii,3) = w_hat(2,1);
            end 
        end

    else
        error('check the size of matrix');
    end
end