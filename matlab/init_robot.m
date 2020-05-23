%% Accensione/Collegamento al ROSCORE e caricamento del modello del robot
clear all
close all

%Mettere commento se non si usa una macchina virtuale e impostare l'ip
%della macchina su cui gira il roscore in ROS_MASTER_URI e l'ip di questa
%macchina in ROS_IP

%setenv('ROS_MASTER_URI','http://192.168.101.45:11311')
%setenv('ROS_IP','192.168.101.6')

rosinit

%% Load robot object from URDF
robot=importrobot('../src/eDo_description/urdf/edo.urdf');

default_name_1 = uint8('edo_joint_1');
default_name_2 = uint8('edo_joint_2');
default_name_3 = uint8('edo_joint_3');
default_name_4 = uint8('edo_joint_4');
default_name_5 = uint8('edo_joint_5');
default_name_6 = uint8('edo_joint_6');

name_max_length = uint32(32);

%% Close ROSCORE
clc
rosshutdown
setenv('ROS_MASTER_URI', '');
setenv('ROS_IP','');
clear