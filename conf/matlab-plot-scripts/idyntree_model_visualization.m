close all;
clear all;
clc;

modelPath = '/home/yeshi/software/robotology-superbuild/robotology/human-gazebo/humanSubject08/';
fileName  = 'humanSubject08_66dof.urdf';

jointOrder = {
    'jLeftHip_rotx'; 
    'jL5S1_rotx';
    'jRightHip_rotx';
    'jRightHip_roty';
    'jRightHip_rotz';
    'jRightKnee_roty';
    'jRightKnee_rotz';
    'jRightAnkle_rotx';
    'jRightAnkle_roty';
    'jRightAnkle_rotz';
    'jRightBallFoot_roty';
    'jL5S1_roty';
    'jL4L3_rotx';
    'jL4L3_roty';
    'jL1T12_rotx';
    'jL1T12_roty';
    'jT9T8_rotx';
    'jT9T8_roty';
    'jT9T8_rotz';
    'jT1C7_rotx';
    'jRightC7Shoulder_rotx';
    'jLeftC7Shoulder_rotx';
    'jLeftShoulder_rotx';
    'jLeftShoulder_roty';
    'jLeftShoulder_rotz';
    'jLeftElbow_roty';
    'jLeftElbow_rotz';
    'jLeftWrist_rotx';
    'jLeftWrist_rotz';
    'jRightShoulder_rotx';
    'jRightShoulder_roty';
    'jRightShoulder_rotz';
    'jRightElbow_roty';
    'jRightElbow_rotz';
    'jRightWrist_rotx';
    'jRightWrist_rotz';
    'jT1C7_roty';
    'jT1C7_rotz';
    'jC1Head_rotx';
    'jC1Head_roty';
    'jLeftHip_roty';
    'jLeftHip_rotz';
    'jLeftKnee_roty';
    'jLeftKnee_rotz';
    'jLeftAnkle_rotx';
    'jLeftAnkle_roty';
    'jLeftAnkle_rotz';
    'jLeftBallFoot_roty';
    };
    
% if installed with robotology superbuild you can directly use
icubModelsInstallPrefix = getenv('ROBOTOLOGY_SUPERBUILD_INSTALL_PREFIX');

meshFilePrefix = [icubModelsInstallPrefix '/share'];

% Main variable of iDyntreeWrappers used for many things including updating
% robot position and getting world to frame transforms
KinDynModel = iDynTreeWrappers.loadReducedModel(jointOrder,'Pelvis',modelPath,fileName,false);

% create vector of positions
joints_positions=zeros(KinDynModel.NDOF,1);

% add a world to base mainly to avoid overlap of coordinate frame and robot
world_H_base=[1,0,0,0;0,1,0,0;0,0,1,0.6;0,0,0,1];

% Set initial position of the robot
iDynTreeWrappers.setRobotState(KinDynModel,world_H_base,joints_positions,zeros(6,1),zeros(size(joints_positions)),[0,0,-9.81]);

% Prepare figure, handles and variables required for the update, some extra
% options are commented.
[visualizer,objects]=iDynTreeWrappers.prepareVisualization(KinDynModel,meshFilePrefix,...
    'color',[0,0,1],'material','metal','transparency',1,'debug',true,'view',[-92.9356   22.4635],...
    'groundOn',true,'groundColor',[0.5 0.5 0.5], 'groundTransparency',0.5,'groundFrame','Pelvis');%,... % optional inputs
     %'style','wireframe','wireframe_rendering',0.1);

% pause to let you see the result with the options that are enabled
pause(1);

%% Load data

%% Load data
load('/home/yeshi/software/robotology-superbuild/robotology/human-dynamics-estimation/conf/xml/testData/matLogFile.mat');


%% Set data world_H_base
world_H_base_pos = iDynTree.Position(data.basePose(1,1), data.basePose(2,1), data.basePose(3,1));
quat = iDynTree.Vector4;

quat.setVal(0, data.basePose(4,1));
quat.setVal(1, data.basePose(5,1));
quat.setVal(2, data.basePose(6,1));
quat.setVal(3, data.basePose(7,1));

world_H_base_rot = iDynTree.Rotation.RotationFromQuaternion(quat);

data_world_H_base = zeros(4,4);

data_world_H_base(1:3, 4)   = world_H_base_pos.toMatlab;
data_world_H_base(1:3, 1:3) = world_H_base_rot.toMatlab;

%% Set data joint positions
data_joints_positions = data.jointPositions(:,1);

%% Update kinematics
iDynTreeWrappers.setRobotState(KinDynModel,data_world_H_base,data_joints_positions,zeros(6,1),zeros(size(joints_positions)),[0,0,-9.81]);

tic
 iDynTreeWrappers.updateVisualization(KinDynModel,visualizer);
stringVector_time=toc
axis tight