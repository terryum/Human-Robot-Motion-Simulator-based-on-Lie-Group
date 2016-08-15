 
%% Apply BCH formula

% made by Terry Taewoong Um (terry.t.um@gmail.com)
% Adaptive Systems Lab., University of Waterloo

function BCH = BCH_so3(LieParam1, LieParam2)
    BCH = LieParam1 + LieParam2 + 0.5*cross(LieParam1,LieParam2) + ...
        1/12*(cross(LieParam1,cross(LieParam1,LieParam2))+cross(LieParam2,cross(LieParam2,LieParam1)));
end