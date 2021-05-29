clc
clear all
close all


% Load datasets
% % Data = load('/home/yeshi/Desktop/non_collocated_wrench_estimation_dataset/covariance_tuning_dataset/AMTI_Dataset/sub03/tpose/hde/feet/1e-6/withoutSOT.mat');
Data = load('/home/yeshi/Desktop/non_collocated_wrench_estimation_dataset/covariance_tuning_dataset/AMTI_Dataset/sub03/tpose/hde/feet/1e8/matLogFile.mat');

timeExt = Data.data.time'/1e9;
jointPos = Data.data.jointPositions;
jointVel = Data.data.jointVelocities;
jointAcc = Data.data.jointAccs;

% Estimate joint velocities and accelerations
polyOrder = 3;
winSize = 41;

% [jointPosSmoothed, jointVels, jointAccs] = estimateJointVelocitiesAndAccelerations(timeExt, jointPos, polyOrder, winSize);

jointOrder = {'jL5S1_rotx';'jRightHip_rotx';'jLeftHip_rotx';'jLeftHip_roty';'jLeftHip_rotz';'jLeftKnee_rotx';'jLeftKnee_roty';'jLeftKnee_rotz';'jLeftAnkle_rotx';'jLeftAnkle_roty';'jLeftAnkle_rotz';'jLeftBallFoot_rotx';'jLeftBallFoot_roty';'jLeftBallFoot_rotz';'jRightHip_roty';'jRightHip_rotz';'jRightKnee_rotx';'jRightKnee_roty';'jRightKnee_rotz';'jRightAnkle_rotx';'jRightAnkle_roty';'jRightAnkle_rotz';'jRightBallFoot_rotx';'jRightBallFoot_roty';'jRightBallFoot_rotz';'jL5S1_roty';'jL5S1_rotz';'jL4L3_rotx';'jL4L3_roty';'jL4L3_rotz';'jL1T12_rotx';'jL1T12_roty';'jL1T12_rotz';'jT9T8_rotx';'jT9T8_roty';'jT9T8_rotz';'jLeftC7Shoulder_rotx';'jT1C7_rotx';'jRightC7Shoulder_rotx';'jRightC7Shoulder_roty';'jRightC7Shoulder_rotz';'jRightShoulder_rotx';'jRightShoulder_roty';'jRightShoulder_rotz';'jRightElbow_rotx';'jRightElbow_roty';'jRightElbow_rotz';'jRightWrist_rotx';'jRightWrist_roty';'jRightWrist_rotz';'jT1C7_roty';'jT1C7_rotz';'jC1Head_rotx';'jC1Head_roty';'jC1Head_rotz';'jLeftC7Shoulder_roty';'jLeftC7Shoulder_rotz';'jLeftShoulder_rotx';'jLeftShoulder_roty';'jLeftShoulder_rotz';'jLeftElbow_rotx';'jLeftElbow_roty';'jLeftElbow_rotz';'jLeftWrist_rotx';'jLeftWrist_roty';'jLeftWrist_rotz'};


%% Load model and iDynTree related classes
% dynComp = iDynTree.KinDynComputations();

%% Load urdf model
subject = 'humanSubject03';
modelPath = strcat('/home/yeshi/software/robotology-superbuild/src/human-gazebo/', subject, '/');
fileName  = strcat(subject, '_66dof.urdf');

% Main variable of iDyntreeWrappers used for many things including updating
% robot position and getting world to frame transforms
KinDynModel = iDynTreeWrappers.loadReducedModel(jointOrder,'Pelvis',modelPath,fileName,false);

KinDynModel.kinDynComp.setFrameVelocityRepresentation(iDynTree.MIXED_REPRESENTATION);

rightfootIdx = KinDynModel.kinDynComp.model().getLinkIndex('RightFoot');
leftfootIdx = KinDynModel.kinDynComp.model().getLinkIndex('LeftFoot');

righthandIdx = KinDynModel.kinDynComp.model().getLinkIndex('RightHand');
lefthandIdx = KinDynModel.kinDynComp.model().getLinkIndex('LeftHand');

% instantiate and initialize robot state and gravity
gravityTrqs = zeros(size(Data.data.jointTorques));
currentTrqs = zeros(size(Data.data.jointTorques));

gravity = iDynTree.Vector3();
gravity.fromMatlab([0, 0, -9.8]);


n_joints = size(jointPos, 1);
world_H_base=[1,0,0,0;0,1,0,0;0,0,1,0;0,0,0,1];

% % meshFilePrefix="";
% % [visualizer,objects]=iDynTreeWrappers.prepareVisualization(KinDynModel,meshFilePrefix,...
% %     'color',[0.1,0.1,0.25],'material','metal','transparency',0.35,'debug',true,'view',[92.9356   5.4635]);
% %      %%'style','wireframe','wireframe_rendering',1);
% % 

for t = 1 : length(timeExt)
    
%   q = iDynTree.VectorDynSize(n_joints);
  q = iDynTree.JointPosDoubleArray(n_joints);
  q.fromMatlab((jointPos(:, t)));

  dq = iDynTree.VectorDynSize(n_joints);
  dq.fromMatlab((jointVel(:, t)));
  
  d2q = iDynTree.VectorDynSize(n_joints);
  d2q.fromMatlab((jointAcc(:, t)));
  
%   baseVel = iDynTree.Vector6();
  baseVel = iDynTree.Twist();
  baseVel.zero();
  
  baseVel.setVal(0, Data.linkData(1).data.velocity(1, t));
  baseVel.setVal(1, Data.linkData(1).data.velocity(2, t));  
  baseVel.setVal(2, Data.linkData(1).data.velocity(3, t));  
  baseVel.setVal(3, Data.linkData(1).data.velocity(4, t));  
  baseVel.setVal(4, Data.linkData(1).data.velocity(5, t));  
  baseVel.setVal(5, Data.linkData(1).data.velocity(6, t));
  
%   iDynTreeWrappers.setRobotState(KinDynModel, world_H_base, q, baseVel, dq,gravity);
  
  w_H_b = iDynTree.Matrix4x4;
  w_H_b.fromMatlab(world_H_base);
  baseTF = iDynTree.Transform();
% %   baseTF.fromHomogeneousTransform(w_H_b);
  
  basePose = Data.linkData(1).data.pose;
  baseRot = iDynTree.Rotation.RPY(basePose(4), basePose(5), basePose(6));
  
  baseTF.setPosition(iDynTree.Position(basePose(1), basePose(2), basePose(3))); 
  baseTF.setRotation(baseRot);
  
  KinDynModel.kinDynComp.setRobotState(baseTF, q, baseVel, dq, gravity);
  
  generalizedTrqs = iDynTree.FreeFloatingGeneralizedTorques(KinDynModel.kinDynComp.model());
  KinDynModel.kinDynComp.generalizedGravityForces(generalizedTrqs);

  gravityTrqs(:, t) = generalizedTrqs.jointTorques.toMatlab;
  
  generalizedbiasTrqs =  iDynTree.FreeFloatingGeneralizedTorques(KinDynModel.kinDynComp.model());
  KinDynModel.kinDynComp.generalizedBiasForces(generalizedbiasTrqs);
  
  biasTrqs(:, t) = generalizedbiasTrqs.jointTorques.toMatlab;
    
  baseAcc = iDynTree.Vector6();
  baseAcc.zero();
  
  baseAcc.setVal(0, Data.linkData(1).data.acceleration(1, t));
  baseAcc.setVal(1, Data.linkData(1).data.acceleration(2, t));  
  baseAcc.setVal(2, Data.linkData(1).data.acceleration(3, t));  
  baseAcc.setVal(3, Data.linkData(1).data.acceleration(4, t));  
  baseAcc.setVal(4, Data.linkData(1).data.acceleration(5, t));  
  baseAcc.setVal(5, Data.linkData(1).data.acceleration(6, t));
  
  
  %% Get link transform
  w_rpy_lefttFoot = Data.linkData(14).data.pose(4:6, t);
  w_rpy_rightFoot = Data.linkData(3).data.pose(4:6, t);
  
  w_R_leftFoot = iDynTree.Rotation.RPY(w_rpy_lefttFoot(1), w_rpy_lefttFoot(2), w_rpy_lefttFoot(3));
  w_R_rightFoot = iDynTree.Rotation.RPY(w_rpy_rightFoot(1), w_rpy_rightFoot(2), w_rpy_rightFoot(3));
  
  linkNetWrench = iDynTree.LinkWrenches(KinDynModel.kinDynComp.model());
  linkNetWrench.zero();
  
  leftContactWrench = linkNetWrench(leftfootIdx);
  rightContactWrench = linkNetWrench(rightfootIdx); 
  
  leftContactWrench.setVal(0, Data.data.task2_wrenchMeasurementsInLinkFrame(1, t));
  leftContactWrench.setVal(1, Data.data.task2_wrenchMeasurementsInLinkFrame(2, t));
  leftContactWrench.setVal(2, Data.data.task2_wrenchMeasurementsInLinkFrame(3, t));
  leftContactWrench.setVal(3, Data.data.task2_wrenchMeasurementsInLinkFrame(4, t));
  leftContactWrench.setVal(4, Data.data.task2_wrenchMeasurementsInLinkFrame(5, t));
  leftContactWrench.setVal(5, Data.data.task2_wrenchMeasurementsInLinkFrame(6, t));
  
  rightContactWrench.setVal(0, Data.data.task2_wrenchMeasurementsInLinkFrame(7, t));
  rightContactWrench.setVal(1, Data.data.task2_wrenchMeasurementsInLinkFrame(8, t));
  rightContactWrench.setVal(2, Data.data.task2_wrenchMeasurementsInLinkFrame(9, t));
  rightContactWrench.setVal(3, Data.data.task2_wrenchMeasurementsInLinkFrame(10, t));
  rightContactWrench.setVal(4, Data.data.task2_wrenchMeasurementsInLinkFrame(11, t));
  rightContactWrench.setVal(5, Data.data.task2_wrenchMeasurementsInLinkFrame(12, t));
  
  leftContactWrench = w_R_leftFoot * leftContactWrench;
  rightContactWrench = w_R_rightFoot * rightContactWrench;
  
  baseForceAndJointTorques = iDynTree.FreeFloatingGeneralizedTorques(KinDynModel.kinDynComp.model());
  KinDynModel.kinDynComp.inverseDynamics(baseAcc, d2q, linkNetWrench, baseForceAndJointTorques);
  computedTrqs(:, t) = baseForceAndJointTorques.jointTorques.toMatlab;
  
  inertialTrqs(:, t) = computedTrqs(:, t) - gravityTrqs(:, t) - biasTrqs(:, t);
  
% %   iDynTreeWrappers.updateVisualization(KinDynModel,visualizer);
% %   pause(0.01);
% %   
end

%%
for i=1:n_joints
    plot(computedTrqs(i, :))
    hold on;
    plot(gravityTrqs(i, :));
    hold on;
    plot(Data.data.jointTorques(i,:), '--');
    legend('Computed Torques', 'Gravity Torques', 'HDE Estimated Toruqes');
    title(jointOrder(i));
    pause
    clf
end
