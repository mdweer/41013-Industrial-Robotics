function [nextTr,gripClose] = getNextTr(stageCurrent, loopCurrent, basketTr, ballTr, dropTr, initTr)
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
                %ee to DropTr * basketCentreTr
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
nextTr{1} = zeros(4,4);
nextTr{2} = zeros(4,4);
gripClose = [false,false];

% Defining position displacement translations
% dropDispl = transl(0,0,0.3);
gripTM5Tr = transl(0,0,-0.092);
gripDobotTr = transl(0,0,0.025);
basketDobotTr = transl(0,-0.09,0.143);

switch stageCurrent
    case {1}
        nextTr{1} = basketTr{loopCurrent} * gripDobotTr * basketDobotTr;
        nextTr{2} = ballTr{loopCurrent} * gripTM5Tr * trotx(0,'deg');
    case {2}
        gripClose(1) = true;
        gripClose(2) = true;
    case {3}
        nextTr{1} = dropTr{loopCurrent} * gripDobotTr * basketDobotTr;
        nextTr{2} = dropTr{loopCurrent} * gripTM5Tr * trotx(0,'deg');
        gripClose(1) = true;
        gripClose(2) = true;
    case {4}
        gripClose(1) = true;
    case {5}
        nextTr{1} = basketTr{loopCurrent} * gripDobotTr * basketDobotTr;
        nextTr{2} = dropTr{loopCurrent} * gripTM5Tr * trotx(0,'deg');
        gripClose(1) = true;
    case {6}
    case {7}
        nextTr{1} = initTr{1};
        nextTr{2} = initTr{2};
    otherwise
        error('Stage limit for getNextPose exceeded');
end
if ~(isequal(nextTr{1},zeros(4,4))) && ~(isequal(nextTr{2},zeros(4,4)))
    plot3(nextTr{1}(1,4), ...
          nextTr{1}(2,4), ...
          nextTr{1}(3,4),'^-R');
    plot3(nextTr{2}(1,4), ...
          nextTr{2}(2,4), ...
          nextTr{2}(3,4),'^-G');
end
end