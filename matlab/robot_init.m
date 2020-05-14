clear all
close all

%Togliere il commento se si usa una macchina virtuale e impostare l'ip
%della macchina su cui gira il roscore in ROS_MASTER_URI e l'ip di questa
%macchina in ROS_IP

setenv('ROS_MASTER_URI','http://192.168.101.45:11311')
setenv('ROS_IP','192.168.101.6')


rosinit
% Load robot object from URDF
robot = importrobot('../src/eDo_description/urdf/edo.urdf');
robot.Gravity = [0, 0, -9.81];
robot_2link_planar = importrobot('../src/eDo_description/urdf/edo.urdf');
%% Close ROSCORE
rosshutdown
clear
clc