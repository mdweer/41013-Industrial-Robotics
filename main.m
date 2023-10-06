%% Cleaning Workspace
clear all;
close all;
clc

%% Testing ArmController

baseTr = eye(4);
qInit = [-pi/2,-pi/2,-pi/2,-pi/2,-pi/2,-pi/2];

UR3Controller = ArmController(UR3(baseTr),100,10^-6,qInit);
hold on
%% 

%% RESOLVE MOTION RATE CONTROL instead of Joint Interpolation

disp('L_1');
Joint1 = UR3Controller.GetJointPose(1);
plot3(Joint1(1,4),Joint1(2,4),Joint1(3,4));
disp(Joint1)

disp('L_2');
Joint2 = UR3Controller.GetJointPose(2);
plot3(Joint2(1,4),Joint2(2,4),Joint2(3,4));
disp(Joint2)

disp('L_3');
Joint3 = UR3Controller.GetJointPose(3);
plot3(Joint3(1,4),Joint3(2,4),Joint3(3,4));
disp(Joint3)

disp('L_4');
Joint4 = UR3Controller.GetJointPose(4);
plot3(Joint4(1,4),Joint4(2,4),Joint4(3,4));
disp(Joint4)

disp('L_5');
Joint5 = UR3Controller.GetJointPose(5);
plot3(Joint5(1,4),Joint5(2,4),Joint5(3,4));
disp(Joint5)

disp('L_6');
Joint6 = UR3Controller.GetJointPose(6);
plot3(Joint6(1,4),Joint6(2,4),Joint6(3,4));
disp(Joint6)

UR3.display();