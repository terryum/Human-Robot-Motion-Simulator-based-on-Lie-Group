
%% LoadFromAmc : Load motions from amc files

% made by Terry Taewoong Um (terry.t.um@gmail.com)
% Adaptive Systems Lab., University of Waterloo

function [myModel nData] = LoadFromAmc(AmcFilename, myModel)
 
    nDataMax = 15000;        % Maximum length of data (Need to modify)
    nBody = 6;              % Number of open chains
    T_Body = zeros(4,4,29,nDataMax);    % Euler angles -> Rotation matrices (4 by 4, but P=0)
    EulerAngles_Total =  zeros(nDataMax, 62);
    
    % nDof_Offset indicates the starting index of the Euler angles for the corresponding joint.
    nDof_Offset = zeros(29,1);   
    nDof_Offset(1,1) = 1;
    for ii=2:29
        nDof_Offset(ii,1) = nDof_Offset(ii-1,1)+myModel{1,1}.nDof_AllBody(ii-1);
    end
    EulerAngle_Temp = zeros(62,1);
    % Open the amc file
    fid=fopen(AmcFilename, 'rt');
    if fid == -1,
        fprintf('File Open Error, %s.\n', AmcFilename);
        return;
    end;
    % Skipping unnecessary parts     
    line=fgetl(fid);
    while ~strcmp(line,':DEGREES')
      line=fgetl(fid);
    end

    frame=1;
    while ~feof(fid)
        if rem(frame,100) == 0
            disp('Reading frame: ');
            disp(frame);
        end
        
        skip = fscanf(fid,'%s\n',1);
        for i=1:29
        % Mapping keywords with indices
            id = fscanf (fid,'%s',1);
            switch (id)
                case 'root', idx = 1;             case 'lowerback', idx = 2;
                case 'upperback', idx = 3;        case 'thorax', idx = 4; 
                case 'lowerneck', idx = 5;        case 'upperneck', idx = 6; 
                case 'head', idx = 7;

                case 'rclavicle', idx = 8;        case 'rhumerus', idx = 9;
                case 'rradius', idx = 10;         case 'rwrist', idx = 11;
                case 'rhand', idx = 12;           case 'rfingers', idx = 13;
                case 'rthumb', idx = 14;

                case 'lclavicle', idx = 15;       case 'lhumerus', idx = 16;
                case 'lradius', idx = 17;         case 'lwrist', idx = 18;
                case 'lhand', idx = 19;           case 'lfingers', idx = 20;
                case 'lthumb', idx = 21;

                case 'rfemur', idx = 22;          case 'rtibia', idx = 23;
                case 'rfoot', idx = 24;           case 'rtoes', idx = 25;

                case 'lfemur', idx = 26;          case 'ltibia', idx = 27;
                case 'lfoot', idx = 28;           case 'ltoes', idx = 29; 
                    
            otherwise
                fprintf('Error, labels in the amc are not correct.\n');
                return;
            end
            
       % Get transformation matrices from the Euler angles     
            len = myModel{1,1}.nDof_AllBody(idx);   % length of Euler angles (3DOF = 3)
            if len == 6     % The root has orientation(3) and position(3)
                x = fscanf (fid,'%f %f %f %f %f %f\n',6);
                x(1:3,1) = x(1:3,1)/0.45*2.54/100;
                x(4:6,1) = deg2rad(x(4:6,:));
                EulerAngle_Temp(nDof_Offset(idx):nDof_Offset(idx)+len-1,1) = x;
                T = RP01(EulerZYX(x(6,1),x(5,1),x(4,1)),x(1:3,:));
            else
                x_temp = zeros(3,1);    i_temp = 1;
                if len == 3
                    x = fscanf (fid,'%f %f %f\n',3);
                elseif len == 2
                    x = fscanf (fid,'%f %f\n',2);
                elseif len == 1
                    x = fscanf (fid,'%f\n',1);
                end
                x(:,1) = deg2rad(x(:,1));
                EulerAngle_Temp(nDof_Offset(idx):nDof_Offset(idx)+len-1,1) = x;
                % Get ZYX angles (if Dof_AllBody=0, angle=0)
                for jj=1:3
                    if myModel{1,1}.Dof_AllBody(idx,jj)==1
                        x_temp(jj,1) = x(i_temp,1);
                        i_temp = i_temp+1;
                    end
                end
                T = RP01(EulerZYX(x_temp(3,1),x_temp(2,1),x_temp(1,1)), zeros(3,1));    
            end
            T_Body(:,:,idx,frame) = T;  % All transformation matrices
        end
        EulerAngles_Total(frame,:) = EulerAngle_Temp';
        frame = frame + 1;
    end
    T_Body(:,:,:,frame:nDataMax) = [];      % Delete remaining parts   
    frame = frame-1;
    disp('Total number of frames read: ');
    disp(frame);
    fclose(fid);

%   T_Moving_Local : Euler angles -> Rotation matrices (4 by 4, but P=0)
    for kk=1:myModel{1,1}.nLink-1
        for ii=1:frame
            myModel{1,1}.T_Moving_Local(:,:,kk,ii) = eye(4);
        end
    end  
    myModel{1,1}.T_Moving_Local(:,:,:,frame+1:nDataMax) = []; % Delete remaining parts
    
    myModel{2,1}.T_Moving_Local = T_Body(:,:,2:6,:);
    myModel{3,1}.T_Moving_Local = T_Body(:,:,8:10,:);
    myModel{4,1}.T_Moving_Local = T_Body(:,:,15:17,:);
    myModel{5,1}.T_Moving_Local = T_Body(:,:,22:24,:);
    myModel{6,1}.T_Moving_Local = T_Body(:,:,26:28,:);
    nData = frame;
    
    % Euler angles for each open chanin
    myModel{1,1}.EulerAngle = EulerAngles_Total(:,1:6);
    myModel{2,1}.EulerAngle = EulerAngles_Total(:,nDof_Offset(2):nDof_Offset(6)-1);
    myModel{3,1}.EulerAngle = EulerAngles_Total(:,nDof_Offset(9):nDof_Offset(11)-1);
    myModel{4,1}.EulerAngle = EulerAngles_Total(:,nDof_Offset(16):nDof_Offset(18)-1);
    myModel{5,1}.EulerAngle = EulerAngles_Total(:,nDof_Offset(22):nDof_Offset(25)-1);
    myModel{6,1}.EulerAngle = EulerAngles_Total(:,nDof_Offset(26):nDof_Offset(29)-1);
    for kk=1:nBody
        myModel{kk,1}.EulerAngle(frame+1:nDataMax,:) = [];
    end
    % T_Moving_Home : Transformation matrices of each joint seen from {home}
    % T_Moving_Abs  : Relative position/orientation w.r.t the first position/orientation
    for kk=1:nBody
        myModel{kk,1}.T_Moving_Home(:,:,:,nData+1:nDataMax)=[]; % Delete remaining parts
        switch (kk)
            case {1,2}
                myModel{kk,1}.T_Moving_Home(:,:,1,:) =  T_Body(:,:,1,:); 
            case {3,4}
                myModel{kk,1}.T_Moving_Home(:,:,1,:) = myModel{2,1}.T_Moving_Home(:,:,4,:);
            case {5,6}
                myModel{kk,1}.T_Moving_Home(:,:,1,:) = myModel{1,1}.T_Moving_Home(:,:,kk-3,:);
        end   
        
        for ii=1:nData
            myModel{kk,1}.T_Moving_Abs(:,:,1,ii) = eye(4);
            for jj=2:myModel{kk,1}.nLink
                % Previous joint position -> rotation-> constant move from i-1 to i joint
                myModel{kk,1}.T_Moving_Home(:,:,jj,ii) = myModel{kk,1}.T_Moving_Home(:,:,jj-1,ii)*...
                                myModel{kk,1}.T_Moving_Local(:,:,jj-1,ii)*myModel{kk,1}.T_JointJoint(:,:,jj);
                myModel{kk,1}.T_Moving_Abs(:,:,jj,ii) = myModel{kk,1}.T_Moving_Abs(:,:,jj-1,ii)*...
                                myModel{kk,1}.T_Moving_Local(:,:,jj-1,ii)*myModel{kk,1}.T_JointJoint(:,:,jj);
            end 
        end
    end 
end
