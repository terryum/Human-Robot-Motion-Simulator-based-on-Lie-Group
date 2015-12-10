
%% LoadFromAsf : Load the human model from the asf file

% made by Terry Taewoong Um (terry.t.um@gmail.com)
% Adaptive Systems Lab., University of Waterloo

function myModel = LoadFromAsf(AsfFileName)

    nDof =[6 3 3 3 3 3 3 2 3 1 1 2 1 2 2 3 1 1 2 1 2 3 1 2 1 3 1 2 1];  % CMU mocap data structure
%   Dof specifies the DOF of each joint. e.g)  [1 1 0] means the joint has 2DOF(x-rot & y-rot).   
    Dof= zeros(29,3);       Dof(1,:) = [1 1 1]; 
    rootFrames = zeros(4,4,3);
    rootFrames(:, :, 1) = eye(4); 
    LocalFrame = zeros(3,3,29);    LocalFrame(:,:,1) = eye(3);
    closeZero = 10^(-10);

    % Open the asf file
    fid=fopen(AsfFileName, 'rt');
    if fid == -1,
        fprintf('File Open Error, %s.\n', AsfFileName);
        return;
    end;

    BodyLength = zeros(29,1);       % The length of each link
    LinkDir = zeros(29,3);          % The direction of each link
    for kk=1:30
       ii_line = 0;
       while true    
           ii_line = ii_line+1;
           scanWord = fscanf(fid,'%s',1);
           if strcmp(scanWord,'name') == 1 % Skip the intro part of the file
               break;
           end
           if ii_line>100   % if fail to find the starting line
               fprintf('Fail to find the first line, %s\n', FileName);
               return;     
           end       
       end
       
       % The meaning of indies described in the asf file        
       id = fscanf (fid,'%s',1);
       switch (id)
            case 'rhipjoint', idx = 101;          case 'lhipjoint', idx = 102;
            case 'lowerback', idx = 2;            case 'upperback', idx = 3;
            case 'thorax', idx = 4;               case 'lowerneck', idx = 5;
            case 'upperneck', idx = 6;            case 'head', idx = 7;

            case 'rclavicle', idx = 8;            case 'rhumerus', idx = 9;
            case 'rradius', idx = 10;             case 'rwrist', idx = 11;
            case 'rhand', idx = 12;               case 'rfingers', idx = 13;
            case 'rthumb', idx = 14;

            case 'lclavicle', idx = 15;           case 'lhumerus', idx = 16;
            case 'lradius', idx = 17;             case 'lwrist', idx = 18;
            case 'lhand', idx = 19;               case 'lfingers', idx = 20;
            case 'lthumb', idx = 21;

            case 'rfemur', idx = 22;              case 'rtibia', idx = 23;
            case 'rfoot', idx = 24;               case 'rtoes', idx = 25;

            case 'lfemur', idx = 26;              case 'ltibia', idx = 27;
            case 'lfoot', idx = 28;               case 'ltoes', idx = 29;
                
           otherwise
               fprintf('Label errors in the ASF file, %s \n',id);
               return;           
       end
        
       if idx~= 101 && idx~= 102    % The root has no link direction becuase there's no i-1 link
           % Link direction
           skip = fscanf(fid, '%s', 1);
           LinkDir(idx,:) = fscanf(fid,'%f %f %f\n',3);
           for jj=1:3
               if abs(LinkDir(idx,jj)) < closeZero  
                   LinkDir(idx,jj) = 0;
               end
           end

           % Link length
           skip = fscanf(fid, '%s', 1);
           BodyLength(idx,1) = fscanf(fid, '%f', 1);
           % From inch to meter (0.45 is a scale factor)
           BodyLength(idx,1) = BodyLength(idx,1)/0.45*2.54/100;

           % Local Axes
           skip = fscanf(fid, '%s', 1);
           LocalEulerAngle(idx,:) = fscanf(fid,'%f %f %f\n',3);
           LocalEulerAngle(idx,:) = deg2rad(LocalEulerAngle(idx,:));
           % LocalFrame : Orientation of the links (SO(3)) w.r.t. {home}
           LocalFrame(:,:,idx) = EulerZYX(LocalEulerAngle(idx,3),LocalEulerAngle(idx,2),LocalEulerAngle(idx,1));

       % Joint Dof (e.g. [1 0 1] means the joint rotates along the x and z axes.) 
            skip = fscanf(fid, '%s %s', 2);
            for jj=1:nDof(1,idx)
                Dof_temp = fscanf(fid, '%s', 1);
                switch (Dof_temp)
                case 'rx', Dof(idx, 1) = 1;  case 'ry', Dof(idx, 2) = 1;  case 'rz', Dof(idx, 3) = 1;  
                end
            end
            
       else          % For the root frame (rhipjoint, lhipjoint) 
            skip = fscanf(fid, '%s', 1);
            dir_temp = fscanf(fid,'%f %f %f\n',3);
            skip = fscanf(fid, '%s', 1);
            length_temp = fscanf(fid, '%f', 1);
            length_temp = length_temp/0.45*2.54/100;
            dir_temp = length_temp*dir_temp;
            switch (id)
                case 'rhipjoint', idx_temp = 2;
                case 'lhipjoint', idx_temp = 3;
            end     
            rootFrames(:, :, idx_temp) = RP01(eye(3), dir_temp);
       end   
    end 
    fclose(fid);

% Build a model structure    
% myModel = cell{1_root, 2_torso, 3_rightArm, 4_leftArm, 5_rightLeg, 6_leftLeg};
    nDataMax = 1000;        % Maximum length of data     
    nBody = 6;              % Number of open chains
    myModel = cell(nBody,1);
    myModel{1,1} = struct('nLink', 4, 'T_JointHome', zeros(4,4,4), 'T_JointJoint', zeros(4,4,4), 'Dof', zeros(4,3), 'T_Moving_Local', zeros(4,4,3,nDataMax), 'T_Moving_Home', zeros(4,4,3,nDataMax), 'EulerAngle', zeros(nDataMax,6), 'Dof_AllBody', zeros(29,3), 'nDof_AllBody', zeros(29,1));
    myModel{2,1} = struct('nLink', 6, 'T_JointHome', zeros(4,4,6), 'T_JointJoint', zeros(4,4,6), 'Dof', zeros(6,3), 'T_Moving_Local', zeros(4,4,5,nDataMax), 'T_Moving_Home', zeros(4,4,6,nDataMax), 'EulerAngle', zeros(nDataMax,12));
    myModel{3,1} = struct('nLink', 4, 'T_JointHome', zeros(4,4,4), 'T_JointJoint', zeros(4,4,4), 'Dof', zeros(4,3), 'T_Moving_Local', zeros(4,4,3,nDataMax), 'T_Moving_Home', zeros(4,4,4,nDataMax), 'EulerAngle', zeros(nDataMax,4));
    myModel{4,1} = struct('nLink', 4, 'T_JointHome', zeros(4,4,4), 'T_JointJoint', zeros(4,4,4), 'Dof', zeros(4,3), 'T_Moving_Local', zeros(4,4,3,nDataMax), 'T_Moving_Home', zeros(4,4,4,nDataMax), 'EulerAngle', zeros(nDataMax,4));
    myModel{5,1} = struct('nLink', 4, 'T_JointHome', zeros(4,4,4), 'T_JointJoint', zeros(4,4,4), 'Dof', zeros(4,3), 'T_Moving_Local', zeros(4,4,3,nDataMax), 'T_Moving_Home', zeros(4,4,4,nDataMax), 'EulerAngle', zeros(nDataMax,6));
    myModel{6,1} = struct('nLink', 4, 'T_JointHome', zeros(4,4,4), 'T_JointJoint', zeros(4,4,4), 'Dof', zeros(4,3), 'T_Moving_Local', zeros(4,4,3,nDataMax), 'T_Moving_Home', zeros(4,4,4,nDataMax), 'EulerAngle', zeros(nDataMax,6));
 
%   Set transformation matrices for the root frames (waist, left hip joint, right hip joint)  
%   myModel{1,1} : Triangular-shaped root. root-rightHip-leftHip-root 
    myModel{1,1}.T_JointHome(:,:,1:3) = rootFrames; 
    myModel{1,1}.T_JointHome(:,:,4) = rootFrames(:,:,1);   

%   T_JointJoint : Transformation matrices from i-1 to i joint
    nJoint = size(myModel{1,1}.T_JointHome, 3);
    myModel{1,1}.T_JointJoint(:,:,1) = myModel{1,1}.T_JointHome(:,:,1);
    for ii=2:nJoint
        myModel{1,1}.T_JointJoint(:,:,ii) = invSE3(myModel{1,1}.T_JointHome(:,:,ii-1))*myModel{1,1}.T_JointHome(:,:,ii);
    end
        
    nIdxOffset = [0; 1; 7; 14; 21; 25];
    startFrames = zeros(4,4,nBody);
    for kk = 2:nBody
        switch (kk)
            case 2, startFrames(:,:,kk) = rootFrames(:,:,1);            % starting from the root
            case {3,4}
                startFrames(:,:,kk) = myModel{2,1}.T_JointHome(:,:,4);  % starting from the throat
            case {5,6}, startFrames(:,:,kk) = rootFrames(:,:,kk-3);     % starting from the left/right hip    
        end  
        % Assign T_JointHome and Dof
        myModel{kk,1}.T_JointHome(:,:,1) = startFrames(:,:,kk);
        myModel{kk,1}.Dof(1,:) = [1,1,1];  % Roots of each limb has 3 Dof
        for ii=2:myModel{kk,1}.nLink-1
            myModel{kk,1}.T_JointHome(:,:,ii) = RP01(LocalFrame(:,:,ii+nIdxOffset(kk,1)), myModel{kk,1}.T_JointHome(1:3,4,ii-1)+BodyLength(ii+nIdxOffset(kk,1)-1,1)*LinkDir(ii+nIdxOffset(kk,1)-1,:)');
            myModel{kk,1}.Dof(ii,:) = Dof(ii+nIdxOffset(kk,1), :);
        end 
        ii = myModel{kk,1}.nLink;
        myModel{kk,1}.T_JointHome(:,:,ii) = RP01(LocalFrame(:,:,ii), myModel{kk,1}.T_JointHome(1:3,4,ii-1)+BodyLength(ii+nIdxOffset(kk,1)-1,1)*LinkDir(ii+nIdxOffset(kk,1)-1,:)');
        myModel{kk,1}.Dof(ii,:) = [0 0 0];
        
        % Assign T_JointJoint
        nJoint = size(myModel{kk,1}.T_JointHome, 3);
        myModel{1,1}.T_JointJoint(:,:,1) = myModel{kk,1}.T_JointHome(:,:,1);
        for ii=2:nJoint
            myModel{kk,1}.T_JointJoint(:,:,ii) = invSE3(myModel{kk,1}.T_JointHome(:,:,ii-1))*myModel{kk,1}.T_JointHome(:,:,ii);
        end
    end
    myModel{1,1}.Dof_AllBody = Dof;
    myModel{1,1}.nDof_AllBody = nDof;
    
end

%%
