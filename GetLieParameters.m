
%% Extract Lie parameters from SE(3)

% made by Terry Taewoong Um (terry.t.um@gmail.com)
% Adaptive Systems Lab., University of Waterloo

function [mdl_subject] = GetLieParameters(mdl_subject, nData)
    
    CLOSE_ZERO = 10^(-13);
    
%   LieVec for root position  
    for jj=1:mdl_subject{1,1}.nLink-1
        for ii=1:nData
%             mdl_subject{1,1}.LieVec_xyz(jj,:,ii) = (mdl_subject{1,1}.T_Moving_Home(1:3,4,jj,ii+1)-mdl_subject{1,1}.T_Moving_Home(1:3,4,jj,ii-1))';      
%             mdl_subject{1,1}.LieVec_xyz(jj,:,ii) = mdl_subject{1,1}.LieVec_xyz(jj,:,ii)*60;  % velocity of the root 
            mdl_subject{1,1}.LieVec_xyz(jj,:,ii) = mdl_subject{1,1}.T_Moving_Home(1:3,4,jj,ii);
            mdl_subject{1,1}.LieMag_xyz(jj,ii) = norm(mdl_subject{1,1}.LieVec_xyz(jj,:,ii));
            if mdl_subject{1,1}.LieMag_xyz(jj,ii) < CLOSE_ZERO
                mdl_subject{1,1}.LieVec_xyz(jj,:,ii) = [0 0 0];
                mdl_subject{1,1}.LieVecNormed_xyz(jj,:,ii) = mdl_subject{1,1}.LieVecNormed_xyz(jj,:,ii+1);
                mdl_subject{1,1}.LieMag_xyz(jj,ii) = 0; 
            else
                mdl_subject{1,1}.LieVecNormed_xyz(jj,:,ii) = mdl_subject{1,1}.LieVec_xyz(jj,:,ii)/mdl_subject{1,1}.LieMag_xyz(jj,ii);
            end 
        end
    end
    
    for kk=2:6
        for jj=1:mdl_subject{kk,1}.nLink-1
            for ii=1:nData
                mdl_subject{kk,1}.LieVec(jj,:,ii) = Smallso3(mdl_subject{kk,1}.T_Moving_Local(1:3,1:3,jj,ii));
                mdl_subject{kk,1}.LieMag(jj,ii) = norm(mdl_subject{kk,1}.LieVec(jj,:,ii));
                if mdl_subject{kk,1}.LieMag(jj,ii) < CLOSE_ZERO
                    mdl_subject{kk,1}.LieVec(jj,:,ii) = [0 0 0];
                    mdl_subject{kk,1}.LieVecNormed(jj,:,ii) = mdl_subject{kk,1}.LieVec(jj,:,ii);
                    mdl_subject{kk,1}.LieMag(jj,ii) = 0; 
                else
                    mdl_subject{kk,1}.LieVecNormed(jj,:,ii) = mdl_subject{kk,1}.LieVec(jj,:,ii)/mdl_subject{kk,1}.LieMag(jj,ii);
                end
            end
        end
    end
    
    % Delete void parts
    mdl_subject{1,1}.LieVec_xyz(:,:,nData+1:1000)=[]; 
    mdl_subject{1,1}.LieMag_xyz(:,nData+1:1000)=[]; 
    mdl_subject{1,1}.LieVecNormed_xyz(:,:,nData+1:1000)=[];   
    for kk = 1:6
        mdl_subject{kk,1}.LieVec(:,:,nData+1:1000)=[]; 
        mdl_subject{kk,1}.LieMag(:,nData+1:1000)=[]; 
        mdl_subject{kk,1}.LieVecNormed(:,:,nData+1:1000)=[];      
    end
end
 