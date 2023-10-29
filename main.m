%% Cleaning Workspace
clear all;
close all;
clc

%% Global Variables
global CLOCK_SPEED;
global BASKET_BALL_COUNT;
global WORKSPACE_FLOOR;
stageMax = 7;

SetClockSpeed(0.5);
SetPairCount(1);
SetWorkspaceFloor(0.6);

%% Setting up Workspace
axis([-2 2 -2 2 0 2]);
hold on

%% Init ArmController for Dobot
baseTr{1} = transl(-0.4, 0.2, GetWorkspaceFloor()) * trotz(180,'deg');
ctrlDobot = ArmController(LinearDM(baseTr{1}));

%% Init ArmController for TM5-700
baseTr{2} = transl(0, -1, GetWorkspaceFloor()) * trotz(-90,'deg');
ctrlTM5 = ArmController(TM5(baseTr{2}));

%% Testing EStop armState
% ctrlDobot.EStop(true)
% ctrlTM5.EStop(true)
% Do something (read terminal)

%% Establishing Locations
initTr{1} = ctrlDobot.GetJointPose(ctrlDobot.jointCount());
initTr{2} = ctrlTM5.GetJointPose(ctrlTM5.jointCount());
workspaceCentre = transl([0,0,GetWorkspaceFloor]);

%Basket locations(Closer to Dobot)
basketArray = cell(2,GetPairCount);
basketArray{1,1} = workspaceCentre * transl([0,0.6,0]) * trotx(0,'deg');
for i = 1:length(basketArray(1,:))
    x = basketArray{1,i}(1,4);
    y = basketArray{1,i}(2,4);
    z = basketArray{1,i}(3,4);
    plot3(x,y,z,'|-r');
    basketArray{2,i} = PlaceObject('view/Models/baskets.ply', [x,y,z]);
end

%Ball locations (closer to TM5-700)
ballArray = cell(GetPairCount);
%Object dectection
ballArray{1,1} = workspaceCentre * transl([-0.3,-0.5,0]) * trotx(180,'deg');
for i = 1:length(ballArray(1,:))
    x = ballArray{1,i}(1,4);
    y = ballArray{1,i}(2,4);
    z = ballArray{1,i}(3,4);
    plot3(x,y,z,'|-g');
    ballArray{2,i} = PlaceObject('view/Models/apple.ply', [x,y,z]);
end

%Ball drop locations
dropArray = cell(GetPairCount);
%Object dectection
dropArray{1} = workspaceCentre * transl([0,0,0]);
for i = 1:length(dropArray(1,:))
    plot3(dropArray{i}(1,4), ...
          dropArray{i}(2,4), ...
          dropArray{i}(3,4),'|-y');
end

%% Placing Objects into Workspace
surf([-2,    -2;     2,     2], ...
     [-2,     2;    -2,     2], ...
     [0.01,0.01;    0.01,0.01], ...
     'CData',imread('view/concrete.jpg'),'FaceColor','texturemap');

emergency = PlaceObject('view/Models/emergencyStopWallMounted.ply', [-1,-1.85,1.5]);
extinguisher = PlaceObject('view/Models/fireExtinguisher.ply', [-1.5, -1.8, 0]);

table = PlaceObject('view/Models/table.ply',[0,0,0]);
rotate(table,   [0, 0, 1], 90);

wall = PlaceObject('view/Models/Wall.ply', [-1.99,-1.99,-1]);
wall1 = PlaceObject('view/Models/Wall.ply', [-1.99,-1.99,-1]);
rotate(wall1,   [0, 0, 1], 90);

fence = PlaceObject('view/Models/barrier1.5x0.2x1m.ply', [0.7, 1.5, 0]);
fence1 = PlaceObject('view/Models/barrier1.5x0.2x1m.ply', [0.7, -1.3, 0]);
fence2 = PlaceObject('view/Models/barrier1.5x0.2x1m.ply', [0.8, -1.55, 0]);
fence3 = PlaceObject('view/Models/barrier1.5x0.2x1m.ply', [0.8, 1.5, 0]);
fence4 = PlaceObject('view/Models/barrier1.5x0.2x1m.ply', [-0.7, 1.5, 0]);
fence5 = PlaceObject('view/Models/barrier1.5x0.2x1m.ply', [-0.7, -1.3, 0]);
fence6 = PlaceObject('view/Models/barrier1.5x0.2x1m.ply', [-0.5, -1.55, 0]);
fence7 = PlaceObject('view/Models/barrier1.5x0.2x1m.ply', [-0.5, 1.5, 0]);
rotate(fence2,  [0, 0, 1], 90);
rotate(fence3,  [0, 0, 1], 90);
rotate(fence6,  [0, 0, 1], 90);
rotate(fence7,  [0, 0, 1], 90);

%% Temporary Variables for GUI
startOperationFlag = true;
EStopFlag = false;
qManipDobot = ctrlDobot.qCurrent;
qManipTM5 = ctrlTM5.qCurrent;
gripCloseArr = [false,false];

%% Main loop
for i = 1:GetPairCount
    disp('Basket Array:')
    disp(basketArray{1,i})
    disp(basketArray{2,i})
    disp('Ball Array:')
    disp(ballArray{1,i})
    disp(ballArray{2,i})
    ctrlDobot.SetGrabTarget(basketArray{1,i},basketArray{2,i});
    ctrlTM5.SetGrabTarget(ballArray{1,i},ballArray{2,i});

    for j = 1:stageMax
        if startOperationFlag
            disp('Stage:')
            disp(j)
            [nextTrArr,gripCloseArr] = getNextTr(j,i,basketArray,ballArray,dropArray,initTr);
        
            DobotTr = nextTrArr{1};
            TM5Tr = nextTrArr{2};
            DobotGrip = gripCloseArr(1);
            TM5Grip = gripCloseArr(2);

            disp('gripCloseArr')
            disp(gripCloseArr)

            ctrlDobot.gripperCloseFlag(DobotGrip);
            ctrlTM5.gripperCloseFlag(TM5Grip);

            %Display current joint configuration
            ctrlDobot.GetJointPose(ctrlDobot.jointCount);
            ctrlTM5.GetJointPose(ctrlTM5.jointCount);
    
            if ~(isequal(DobotTr,zeros(4,4))) && ~(isequal(TM5Tr,zeros(4,4)))
                disp('nextLocation')

                disp('DobotTr:')
                disp(DobotTr)
                disp('TM5Tr:')
                disp(TM5Tr)

                qPathDobot = ctrlDobot.genIKPath(DobotTr,'Trap','Global');
                qPathTM5 = ctrlTM5.genIKPath(TM5Tr,'Trap','Global');
                
                % disp('qPathDobot:')
                % disp(qPathDobot)
                % disp('qPathTM5:')
                % disp(qPathTM5)
                
                %Dobot move qPath
                [success1,error] = ctrlDobot.moveToNextPoint(DobotTr,qPathDobot);
                fprintf('IK Error: %d', error);
                
                %TM5 move qPath
                [success2,error] = ctrlTM5.moveToNextPoint(TM5Tr,qPathTM5);
                fprintf('IK Error: %d', error);

                if ~success1 || ~success2
                    % if either arm fails the movement, repeat loop iteration
                    j = j - 1;
                    pause(GetClockSpeed());
                end
            else
                disp('Gripper animation or wait for ball drop')
            end
        else
            j = j - 1;
            ctrlDobot.SetQ(qManipDobot);
            ctrlTM5.SetQ(qManipTM5);
        end
    end
end