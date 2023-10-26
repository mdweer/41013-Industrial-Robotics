%% Cleaning Workspace
clear all;
close all;
clc

%% Global Variables
global CLOCK_SPEED;
global BASKET_BALL_COUNT;
stageMax = 6;

SetClockSpeed(0.5);
SetPairCount(1);
%% 
axis([-1.5 1.5 -1.5 1.5 0 1.5]);
hold on

%% Init ArmController for Dobot
baseTr{1} = eye(4) * transl(0.25,0,0) * trotz(180,'deg');

% qInit = [-pi/2,-pi/2,-pi/2,-pi/2,-pi/2,-pi/2];
% stepsPerMetre = 100;
% IKErr = 10^-5;
% vMax = 0.05;

ctrlDobot = ArmController(DobotMagician(baseTr{1}));

%% Init ArmController for TM5-700
baseTr{2} = eye(4) * transl(-0.6,0,0) * trotz(180,'deg');

% qInit = [-pi/2,-pi/2,-pi/2,-pi/2,-pi/2,-pi/2];
% stepsPerMetre = 100;
% IKErr = 10^-5;
% vMax = 0.05;

ctrlTM5 = ArmController(TM5(baseTr{2}));

%% Testing EStop armState
% armController.EStop(1)
% Do something (read terminal)
% armController.EStop(0)

%% Establishing Locations
initTr{1} = ctrlDobot.GetJointPose(ctrlDobot.jointCount());
initTr{2} = ctrlTM5.GetJointPose(ctrlTM5.jointCount());

%Basket locations(Closer to Dobot)
basketArray = cell(GetPairCount);
basketArray{1} = baseTr{1} * transl([0,0.2,0.1]) * trotx(0,'deg');
for i = 1:GetPairCount
    plot3(basketArray{i}(1,4), ...
          basketArray{i}(2,4), ...
          basketArray{i}(3,4),'*-r');
end

%Ball locations (closer to TM5-700)
ballArray = cell(GetPairCount);
%Object dectection
ballArray{1} = baseTr{2} * transl([0,0.5,0.1]) * trotx(0,'deg');
for i = 1:GetPairCount
    plot3(ballArray{i}(1,4), ...
          ballArray{i}(2,4), ...
          ballArray{i}(3,4),'*-g');
end

%Ball drop locations
dropArray = cell(GetPairCount);
%Object dectection
dropArray{1} = eye(4) * transl([0,0,0.1]) * trotx(0,'deg');
for i = 1:GetPairCount
    plot3(dropArray{i}(1,4), ...
          dropArray{i}(2,4), ...
          dropArray{i}(3,4),'+-y');
end

disp('Pair Count')
disp(GetPairCount())

%% Temporary Variables for GUI
ManipFlag = 1;
EStopFlag = 0;
qManipDobot = ctrlDobot.qCurrent;
qManipTM5 = ctrlTM5.qCurrent;

%% Main loop
for i = 1:GetPairCount
    for j = 1:stageMax
        if ~ManipFlag
            disp('Stage:')
            disp(j)
            [DobotTr, TM5Tr] = getNextPose(j,i,basketArray,ballArray,dropArray,initTr);
        
            ctrlDobot.GetJointPose(ctrlDobot.jointCount);
            ctrlTM5.GetJointPose(ctrlTM5.jointCount);
    
            if ~(isequal(DobotTr,zeros(4,4))) && ~(isequal(TM5Tr,zeros(4,4)))
                disp('nextLocation')
                qPathDobot = ctrlDobot.genIKPath(DobotTr,'Quin');
                qPathTM5 = ctrlTM5.genIKPath(TM5Tr,'Quin');
            
                %Dobot move qPath
                [success,error] = ctrlDobot.moveToNextPoint(DobotTr,qPathDobot);
                % fprintf('IK Error: %d', error);
                if ~success
                    i = i - 1;
                    pause(GetClockSpeed());
                end
                
                %TM5 move qPath
                [success,error] = ctrlTM5.moveToNextPoint(TM5Tr,qPathTM5);
                % fprintf('IK Error: %d', error);
                if ~success
                    i = i - 1;
                    pause(GetClockSpeed());
                end
            else
                disp('Gripper or wait')
            end
        else
            j = j - 1;
            ctrlDobot.SetQ(qManipDobot);
            ctrlTM5.SetQ(qManipTM5);
        end
    end
end