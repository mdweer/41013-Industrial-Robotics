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
axis([-4 4 -4 4 0 4]);
hold on

%% Dobot Magician Real Robot Setup
% rosshutdown;
% rosinit('192.168.27.1');
% % rosnode list;
% dobotJointSub = '/dobot_magician/joint_states';
% dobotJointPub = '/dobot_magician/target_joint_states';
% dobotJointMsg = 'trajectory_msgs/JointTrajectoryPoint';
% dobotJointSubscriber = rossubscriber(dobotJointSub);

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

app = testRobotPlotApp_V2_exported;
app.setArmControllerObj(ctrlDobot); 
% or ctrlTM5, depending on which one you want to control
app.setArmControllerObj1(ctrlTM5);
% or ctrlTM5, depending on which one you want to control

%Basket locations(Closer to Dobot)
basketArray = cell(2,GetPairCount);
basketArray{1,1} = workspaceCentre * transl([0,0.6,0]) * trotx(0,'deg');
for i = 1:length(basketArray(1,:))
    x = basketArray{1,i}(1,4);
    y = basketArray{1,i}(2,4);
    z = basketArray{1,i}(3,4);
    plot3(x,y,z,'|-r');
    basketArray{2,i} = PlaceObject('View/Models/baskets.ply', [x,y,z]);
end

%Ball locations (closer to TM5-700)
ballArray = cell(GetPairCount);
%Object dectection
ballArray{1,1} = workspaceCentre * ...
                 transl([-0.3,-0.5,0]) * trotx(180,'deg');
for i = 1:length(ballArray(1,:))
    x = ballArray{1,i}(1,4);
    y = ballArray{1,i}(2,4);
    z = ballArray{1,i}(3,4);
    plot3(x,y,z,'|-g');
    ballArray{2,i} = PlaceObject('View/Models/apple.ply', [x,y,z]);
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
surf([-4,    -4;     4,     4], ...
     [-4,     4;    -4,     4], ...
     [0.01,0.01;    0.01,0.01], ...
     'CData',imread('concrete.jpg'),'FaceColor','texturemap');

human = PlaceObject('View/Models/personMaleOld.ply', [0,-3,0]);
emergency = PlaceObject('View/Models/emergencyStopWallMounted.ply', ...
                        [-1,3.95,1.5]);
rotate(emergency,   [0, 0, 1], 90);



extinguisher = PlaceObject('View/Models/fireExtinguisher.ply', ...
                           [-2.5, 3.5, 0]);
rotate(extinguisher,   [0, 0, 1], 90);

table = PlaceObject('View/Models/table.ply',[0,0,0]);
rotate(table,   [0, 0, 1], 90);

walls = PlaceObject('View/Models/WW.ply', [0,0,-1.2]);
wall = PlaceObject('View/Models/Wall.ply', [-3.99,-3.99,-1]);


fence = PlaceObject('View/Models/barrier1.5x0.2x1m.ply', [0.7, 1.5, 0]);
fence1 = PlaceObject('View/Models/barrier1.5x0.2x1m.ply', [0.7, -1.3, 0]);
fence2 = PlaceObject('View/Models/barrier1.5x0.2x1m.ply', [0.8, -1.55, 0]);
fence3 = PlaceObject('View/Models/barrier1.5x0.2x1m.ply', [0.8, 1.5, 0]);
fence4 = PlaceObject('View/Models/barrier1.5x0.2x1m.ply', [-0.7, 1.5, 0]);
fence5 = PlaceObject('View/Models/barrier1.5x0.2x1m.ply', [-0.7, -1.3, 0]);
fence6 = PlaceObject('View/Models/barrier1.5x0.2x1m.ply', [-0.5, -1.55, 0]);
fence7 = PlaceObject('View/Models/barrier1.5x0.2x1m.ply', [-0.5, 1.5, 0]);
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
            [nextTrArr,gripCloseArr] = getNextTr(j,i,basketArray, ...
                                            ballArray,dropArray,initTr);
        
            DobotTr = nextTrArr{1};
            TM5Tr = nextTrArr{2};
            DobotGrip = gripCloseArr(1);
            TM5Grip = gripCloseArr(2);

            disp('gripCloseArr')
            disp(gripCloseArr)

            if DobotGrip
                ctrlDobot.grabTarget();
            end
            if TM5Grip
                ctrlTM5.grabTarget();
            end

            %Display current joint configuration
            ctrlDobot.GetJointPose(ctrlDobot.jointCount);
            ctrlTM5.GetJointPose(ctrlTM5.jointCount);
    
            if ~(isequal(DobotTr,zeros(4,4))) && ...
               ~(isequal(TM5Tr,zeros(4,4)))
                disp('nextLocation')

                disp('DobotTr:')
                disp(DobotTr)
                disp('TM5Tr:')
                disp(TM5Tr)

                qPathDobot = ctrlDobot.genPath(DobotTr,'Trap','Global');
                qPathTM5 = ctrlTM5.genPath(TM5Tr,'RMRC','Global');
                
                % disp('qPathDobot:')
                % disp(qPathDobot)
                % disp('qPathTM5:')
                % disp(qPathTM5)
                
                %Dobot move qPath
                [success1,error] = ctrlDobot.moveToNextPoint(DobotTr, ...
                                                             qPathDobot);
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