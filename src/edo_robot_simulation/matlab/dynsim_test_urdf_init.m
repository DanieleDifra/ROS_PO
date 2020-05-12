clear all
close all

% Create robot object
robot_2link = rigidBodyTree('DataFormat','column','MaxNumBodies',3);

L1 = 1;
L2 = 1;

body = rigidBody('arm_link');
joint = rigidBodyJoint('arm_joint', 'revolute');
setFixedTransform(joint,trvec2tform([0 0 0]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
body.Mass = 2;
body.CenterOfMass = [L1/2 0 0];
body.Inertia = [0 0.67 0.67 0 0 0]; % [Ixx Iyy Izz Iyz Ixz Ixy]
addBody(robot_2link, body, 'base');

body = rigidBody('forearm_link');
joint = rigidBodyJoint('forearm_joint','revolute');
setFixedTransform(joint, trvec2tform([L1,0,0]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
body.Mass = 1;
body.CenterOfMass = [L2/2 0 0];
body.Inertia = [0 0.335 0.335 0 0 0]; % [Ixx Iyy Izz Iyz Ixz Ixy]
addBody(robot_2link, body, 'arm_link');

showdetails(robot_2link)

% Load robot object from URDF
robot_2link_fromURDF = importrobot('../urdf/2link_planar_model.urdf');

showdetails(robot_2link_fromURDF)
