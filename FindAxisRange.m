
%% FindAxisRange : Find an appropriate range of the axes for plotting robot motions

% made by Terry Taewoong Um (terry.t.um@gmail.com)
% Adaptive Systems Lab., University of Waterloo

function axisRange = FindAxisRange(robotModel, myRange)

    axisRange = zeros(1,6);
    nBody = size(robotModel,1);
    INF = 99999999;
    xMax_Prev = -INF; yMax_Prev = -INF; zMax_Prev = -INF; 
    xMin_Prev = INF;  yMin_Prev =  INF; zMin_Prev = INF; 
    
    for kk=1:nBody
         myModel = robotModel{kk,1};
        
        % If the range of motion idices for plotting is not specified, myRange = [1 End] 
        if nargin == 1  
            myRange = [1 size(myModel.T_Moving_Home, 4)];
        end
        
        % Find the workspace of the robot motion
        for ii_Link=1:myModel.nLink
            xMax(kk) = max(myModel.T_Moving_Home(1,4,ii_Link,myRange(1,1):myRange(1,2)));
            yMax(kk) = max(myModel.T_Moving_Home(2,4,ii_Link,myRange(1,1):myRange(1,2)));
            zMax(kk) = max(myModel.T_Moving_Home(3,4,ii_Link,myRange(1,1):myRange(1,2)));
            if xMax(kk) > xMax_Prev
                xMax_Prev(kk) = xMax(kk);
            end
            if yMax(kk) > yMax_Prev
                yMax_Prev(kk) = yMax(kk);
            end
            if zMax(kk) > zMax_Prev
                zMax_Prev(kk) = zMax(kk);
            end
           
            xMin(kk) = min(myModel.T_Moving_Home(1,4,ii_Link,myRange(1,1):myRange(1,2)));
            yMin(kk) = min(myModel.T_Moving_Home(2,4,ii_Link,myRange(1,1):myRange(1,2)));
            zMin(kk) = min(myModel.T_Moving_Home(3,4,ii_Link,myRange(1,1):myRange(1,2)));
            if xMin(kk) < xMin_Prev
                xMin_Prev(kk) = xMin(kk);
            end
            if yMin(kk) < yMin_Prev
                yMin_Prev(kk) = yMin(kk);
            end
            if zMin(kk) < zMin_Prev
                zMin_Prev(kk) = zMin(kk);
            end
        end
    end
    
%   Make the range of each axis equal  
    xMid = (max(xMax_Prev)+min(xMin_Prev))/2;
    yMid = (max(yMax_Prev)+min(yMin_Prev))/2;
    zMid = (max(zMax_Prev)+min(zMin_Prev))/2;
    Range = max([max(xMax_Prev)-min(xMin_Prev); max(yMax_Prev)-min(yMin_Prev); max(zMax_Prev)-min(zMin_Prev)]);
    
    axisRange = [xMid-Range/2 xMid+Range/2 yMid-Range/2 yMid+Range/2 zMid-Range/2 zMid+Range/2];
end