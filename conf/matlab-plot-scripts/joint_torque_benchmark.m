% % close all;
% % clear all;
% % clc;

gravity = [0,0,9.81];

load = 2; %%Kgs

%% Load urdf model
subject = 'humanSubject03';
modelPath = strcat('/home/yeshi/software/robotology-superbuild/src/human-gazebo/', subject, '/');
fileName  = strcat(subject, '_66dof.urdf');


jointOrder = {'jL5S1_rotx';'jRightHip_rotx';'jLeftHip_rotx';'jLeftHip_roty';'jLeftHip_rotz';'jLeftKnee_rotx';'jLeftKnee_roty';'jLeftKnee_rotz';'jLeftAnkle_rotx';'jLeftAnkle_roty';'jLeftAnkle_rotz';'jLeftBallFoot_rotx';'jLeftBallFoot_roty';'jLeftBallFoot_rotz';'jRightHip_roty';'jRightHip_rotz';'jRightKnee_rotx';'jRightKnee_roty';'jRightKnee_rotz';'jRightAnkle_rotx';'jRightAnkle_roty';'jRightAnkle_rotz';'jRightBallFoot_rotx';'jRightBallFoot_roty';'jRightBallFoot_rotz';'jL5S1_roty';'jL5S1_rotz';'jL4L3_rotx';'jL4L3_roty';'jL4L3_rotz';'jL1T12_rotx';'jL1T12_roty';'jL1T12_rotz';'jT9T8_rotx';'jT9T8_roty';'jT9T8_rotz';'jLeftC7Shoulder_rotx';'jT1C7_rotx';'jRightC7Shoulder_rotx';'jRightC7Shoulder_roty';'jRightC7Shoulder_rotz';'jRightShoulder_rotx';'jRightShoulder_roty';'jRightShoulder_rotz';'jRightElbow_rotx';'jRightElbow_roty';'jRightElbow_rotz';'jRightWrist_rotx';'jRightWrist_roty';'jRightWrist_rotz';'jT1C7_roty';'jT1C7_rotz';'jC1Head_rotx';'jC1Head_roty';'jC1Head_rotz';'jLeftC7Shoulder_roty';'jLeftC7Shoulder_rotz';'jLeftShoulder_rotx';'jLeftShoulder_roty';'jLeftShoulder_rotz';'jLeftElbow_rotx';'jLeftElbow_roty';'jLeftElbow_rotz';'jLeftWrist_rotx';'jLeftWrist_roty';'jLeftWrist_rotz'};


% Main variable of iDyntreeWrappers used for many things including updating
% robot position and getting world to frame transforms
KinDynModel = iDynTreeWrappers.loadReducedModel(jointOrder,'Pelvis',modelPath,fileName,false);


% create vector of positions
joints_positions=zeros(KinDynModel.NDOF,1);

% add a world to base mainly to avoid overlap of coordinate frame and robot
world_H_base=[1,0,0,0;0,1,0,0;0,0,1,0;0,0,0,1];

% Set initial position of the robot
iDynTreeWrappers.setRobotState(KinDynModel,world_H_base,joints_positions,zeros(6,1),zeros(size(joints_positions)),gravity);


%% Compute Shoulder Torque

model = KinDynModel.kinDynComp.getRobotModel;

%% mass, center of mass, 3D inertia matrix
model_inertials = iDynTree.VectorDynSize;
model.getInertialParameters(model_inertials);
model_inertial_params = model_inertials.toMatlab;

upperarm_inertial_params = abs(model_inertial_params(10 * model.getLinkIndex('LeftUpperArm') + 1:...
                                                       10 * model.getLinkIndex('LeftUpperArm') + 10));
                                                   
                                                   
upperarm_inertial_params(3) = upperarm_inertial_params(3)/upperarm_inertial_params(1);
                                                   
                                                   
forearm_inertial_params = abs(model_inertial_params(10 * model.getLinkIndex('LeftForeArm') + 1:...
                                                      10 * model.getLinkIndex('LeftForeArm') + 10));
                                                  
forearm_inertial_params(3) = forearm_inertial_params(3)/forearm_inertial_params(1);
                                                  

elbow_torque =  forearm_inertial_params(1) * gravity(3) * forearm_inertial_params(3) +...
                     load * gravity(3) * (2 * forearm_inertial_params(3))

shoulder_torque = upperarm_inertial_params(1) * gravity(3) * upperarm_inertial_params(3) +...
                        forearm_inertial_params(1) * gravity(3) * (2  * forearm_inertial_params(3) ) +...
                        load * gravity(3) * (2 * (forearm_inertial_params(3) + upperarm_inertial_params(3) ) )