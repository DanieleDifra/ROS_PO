clear all
close all
rosinit
% Load robot object from URDF
robot_2link_planar = importrobot('../robots/edo_sim.urdf');

% Parameters (same as in URDF)
dh=[0.337,0,-pi/2; 0,0.21,0; 0,0,-pi/2; 0.268,0,pi/2; 0,0,-pi/2; 0.174,0,0];