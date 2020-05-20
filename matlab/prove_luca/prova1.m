%% Load robot and show
clear
clc

robot =importrobot('./eDO_description/robots/edo_sim.urdf');
%robot=importrobot('../../src/eDo_description/urdf/edo.urdf');
robot.DataFormat = 'row';

link_6 = 'link_6';
cupHeight = 0.2;
cupRadius = 0.05;
cupPosition = [-0.5, 0.5, cupHeight/2+0.3];
body = rigidBody('cupFrame');
setFixedTransform(body.Joint, trvec2tform(cupPosition))
addBody(robot, body, robot.BaseName);

%axis=show(robot);
%axis.CameraPositionMode='auto';

numWaypoints = 5;
q0 = homeConfiguration(robot);
qWaypoints = repmat(q0, numWaypoints, 1);
gik = generalizedInverseKinematics('RigidBodyTree', robot, ...
    'ConstraintInputs', {'cartesian','position','aiming','orientation','joint'})

heightAboveTable = constraintCartesianBounds(link_6);
heightAboveTable.Bounds = [-inf, inf; ...
                           -inf, inf; ...
                           0.5, inf]
                       
distanceFromCup = constraintPositionTarget('cupFrame');
distanceFromCup.ReferenceBody = link_6;
distanceFromCup.PositionTolerance = 0.005

alignWithCup = constraintAiming('link_6');
alignWithCup.TargetPoint = [0, 0, 100]

limitJointChange = constraintJointBounds(robot)

fixOrientation = constraintOrientationTarget(link_6);
fixOrientation.OrientationTolerance = deg2rad(1)


intermediateDistance = 0.3;

limitJointChange.Weights = ones(size(limitJointChange.Weights));
fixOrientation.Weights = 0;
alignWithCup.Weights = 1;

finalDistanceFromCup = 0.1;
distanceFromCupValues = linspace(intermediateDistance, finalDistanceFromCup, numWaypoints-1);

maxJointChange = deg2rad(60);

for k = 2:numWaypoints
    % Update the target position.
    distanceFromCup.TargetPosition(3) = distanceFromCupValues(k-1);
    % Restrict the joint positions to lie close to their previous values.
    limitJointChange.Bounds = [qWaypoints(k-1,:)' - maxJointChange, ...
                               qWaypoints(k-1,:)' + maxJointChange];
    % Solve for a configuration and add it to the waypoints array.
    [qWaypoints(k,:),solutionInfo] = gik(qWaypoints(k-1,:), ...
                                         heightAboveTable, ...
                                         distanceFromCup, alignWithCup, ...
                                         fixOrientation, limitJointChange);
end

framerate = 15;
r = rateControl(framerate);
tFinal = 10;
tWaypoints = [0,linspace(tFinal/2,tFinal,size(qWaypoints,1)-1)];
numFrames = tFinal*framerate;
qInterp = pchip(tWaypoints,qWaypoints',linspace(0,tFinal,numFrames))';

gripperPosition = zeros(numFrames,3);
for k = 1:numFrames
    gripperPosition(k,:) = tform2trvec(getTransform(robot,qInterp(k,:), ...
                                                    link_6));                                              
end

figure;
show(robot, qWaypoints(1,:), 'PreservePlot', false);
hold on
exampleHelperPlotCupAndTable(cupHeight, cupRadius, cupPosition);
p = plot3(gripperPosition(1,1), gripperPosition(1,2), gripperPosition(1,3));

hold on
for k = 1:size(qInterp,1)
    show(robot, qInterp(k,:), 'PreservePlot', false);
    p.XData(k) = gripperPosition(k,1);
    p.YData(k) = gripperPosition(k,2);
    p.ZData(k) = gripperPosition(k,3);
    waitfor(r);
end
hold off




                   