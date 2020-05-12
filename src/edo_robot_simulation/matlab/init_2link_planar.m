clear all
close all
setenv('ROS_MASTER_URI','http://192.168.101.45:11311')
setenv('ROS_IP','192.168.101.4')
rosinit
% Load robot object from URDF
robot_2link_planar = importrobot('../robots/edo_sim.urdf');

% Parameters (same as in URDF)
L1 = 1; % Arm length [m]
L2 = 1; % Forearm length [m]
L3 = 1;
L4 = 1;
L5 = 1;
L6 = 1;