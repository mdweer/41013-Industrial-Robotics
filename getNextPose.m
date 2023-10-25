function [r1Pose,r2Pose] = getNextPose(stageCurrent,loopCurrent,basketTr,ballTr,dropTr,initTr)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Stage summary:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Stage 1:
            %Dobot(r1):
                %ee to BasketInitTr(i)
            %TM5(r2):
                %ee to BallInitTr(i)
%-------------------------------------------------------------------------%
        % Stage 2:
            %Dobot:
                %gripper close
            %TM5:
                %gripper close
%-------------------------------------------------------------------------%
        % Stage 3:
            %Dobot:
                %ee to DropTr
            %TM5:
                %ee to DropTr * zDisplace
%-------------------------------------------------------------------------%
        % Stage 4:
            %Dobot:
            %TM5:
                %gripper open
%-------------------------------------------------------------------------%
        % Stage 5:
            %Dobot:
                %ee to BasketInitTr(i)
            %TM5:
%-------------------------------------------------------------------------%
        % Stage 6:
            %Dobot:
                %ee to InitTr
            %TM5:
                %ee to InitTr
%-------------------------------------------------------------------------%
        % Stage 7:
            %UNREACHABLE! THROW ERR
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
r1Pose = zeros(4,4);
r2Pose = zeros(4,4);

% Defining position displacement translations
dropDispl = transl(0,0,0.3);
gripDobotTr = transl(0,0,0);
gripperTM5Tr = transl(0,0,0);

switch stageCurrent
    case {1}
        r1Pose = basketTr{loopCurrent};
        r2Pose = ballTr{loopCurrent};
    case {2}
    case {3}
        r1Pose = dropTr{loopCurrent};
        r2Pose = dropTr{loopCurrent} * dropDispl;
    case {4}
    case {5}
        r1Pose = basketTr{loopCurrent};
        r2Pose = dropTr{loopCurrent} * dropDispl;
    case {6}
        r1Pose = initTr{1};
        r2Pose = initTr{2};
    otherwise
        disp('EXCEEDED STAGE LIMIT')
end

end