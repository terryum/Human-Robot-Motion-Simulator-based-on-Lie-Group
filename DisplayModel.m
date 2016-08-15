
%% DisplayRobot : Display the robot's / human's movement

% made by Terry Taewoong Um (terry.t.um@gmail.com)
% Adaptive Systems Lab., University of Waterloo

function DisplayModel(robotModel, axisRange, bShowFrame, bShowEETraj)

    figure();
    time_pause = 0.001;                     % pause between frames

    nData = size(robotModel{1,1}.T_Moving_Home,4);  % length of the motion
    nBody = size(robotModel,1);     % Number of open chains
    for ii_frame = 1:nData
        for kk=1:nBody      
            myModel = robotModel{kk,1};
            Data = zeros(3,myModel.nLink);  % xyz position of the joints
            for ii=1:myModel.nLink
                Data(:,ii) = myModel.T_Moving_Home(1:3,4,ii,ii_frame); % positions of each joint at the time ii_frame
            end
            h_model(kk) = plot3(Data(1,:), Data(2,:), Data(3,:), 'ko-');       hold on;

            % Show the trajectory of the End-effector         
            if bShowEETraj(kk) == 1
                TargetLink = myModel.nLink;
                scatter3(Data(1,TargetLink),Data(2,TargetLink),Data(3,TargetLink), 5, 'r.'); hold on;
            end
            % Show the axes of each joint
            if bShowFrame == true
                for ii=1:myModel.nLink
                    origin = myModel.T_Moving_Home(1:3,4,ii,ii_frame);
                    alpha = 0.05;    % Length of the axes

        % Uncomment the below comments (including deleting parts) if you want to also show the x & y axes          
        %            x_end = origin+alpha*myModel.T_Moving_Home(1:3,1,ii,ii_frame);
        %            h_Xaxis = plot3([origin(1) x_end(1)], [origin(2) x_end(2)], [origin(3) x_end(3)], 'r'); hold on;
        %            y_end = origin+alpha*myModel.T_Moving_Home(1:3,2,ii,ii_frame);
        %            h_Yaxis = plot3([origin(1) y_end(1)], [origin(2) y_end(2)], [origin(3) y_end(3)], 'g'); hold on;
                    z_end = origin+alpha*myModel.T_Moving_Home(1:3,3,ii,ii_frame);
                    h_Zaxis(kk,ii) = plot3([origin(1) z_end(1)], [origin(2) z_end(2)], [origin(3) z_end(3)], 'b'); hold on;
                end
            end    
        end
        axis(axisRange);     grid on;  
        xlabel('x');                ylabel('y');            zlabel('z');

        if ii_frame == 1
            title('Press any key to start');
            pause(); 
            delete(h_model(:));   
            if bShowFrame == true
                delete(h_Zaxis(:,:));
    %            delete(h_Xaxis(:,:));            delete(h_Yaxis(:,:));
            end
        elseif ii_frame == nData
            title([num2str(ii_frame) '/' num2str(nData)]);
        else
            title([num2str(ii_frame) '/' num2str(nData)]);
            pause(time_pause);  
            delete(h_model(:));               
            if bShowFrame == true
                delete(h_Zaxis(:,:));
    %            delete(h_Xaxis(:,:));            delete(h_Yaxis(:,:));
            end   
        end
    end
end
