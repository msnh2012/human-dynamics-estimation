close all;
clear all;
clc;

icubModelsInstallPrefix = getenv('ROBOTOLOGY_SUPERBUILD_INSTALL_PREFIX');

gravity = [0,0,9.81];

load = [0, 2, 5]; %%Kgs

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


meshFilePrefix="";
[visualizer,objects]=iDynTreeWrappers.prepareVisualization(KinDynModel,meshFilePrefix,...
    'color',[0.1,0.1,0.25],'material','metal','transparency',0.35,'debug',true,'view',[92.9356   10.4635]);
     %%'style','wireframe','wireframe_rendering',1);
     
%% Visualize marker frames position
%initialize sphere visualization
[X_sphere,Y_sphere,Z_sphere] = sphere;
radius = 0.025;

currentAxisValues=axis;
axisRange=currentAxisValues([2,4,6])-currentAxisValues([1,3,5]);
frameAxisSize = min(axisRange)/4;
linewidthSize=frameAxisSize*50;

model = KinDynModel.kinDynComp.getRobotModel;

%% mass, center of mass, 3D inertia matrix
model_inertials = iDynTree.VectorDynSize;
model.getInertialParameters(model_inertials);
model_inertial_params = model_inertials.toMatlab;

links = [{'LeftUpperArm'}, {'LeftForeArm'}, {'LeftHand'}];

link_inertial_params = [];

for l = 1: size(links, 2)
    
%     link_name = cell2mat(strcat('Right',links(l)));

    link_name = cell2mat(links(l));
    
    
    link_inertial_params(l,:) = model_inertial_params(10 * model.getLinkIndex(link_name) + 1:...
                                                          10 * model.getLinkIndex(link_name) + 10)';
    
    %% Correct link COM value from iDyntree inertal value of mass * com
    link_inertial_params(l,3) = link_inertial_params(l,3)/link_inertial_params(l,1);
    
    
    link_transform_idyn = KinDynModel.kinDynComp.getWorldTransform(link_name);
    link_transform_matlab = link_transform_idyn.asHomogeneousTransform.toMatlab;
    
    % visualize frame center as a sphere
    framePosition = link_transform_matlab(1:3,4);
    surf(X_sphere*radius + framePosition(1) + link_inertial_params(l,2), ...
         Y_sphere*radius + framePosition(2) + link_inertial_params(l,3), ...
         Z_sphere*radius + framePosition(3) + link_inertial_params(l,4), ...
         'FaceColor','w', ...
         'EdgeColor','k')


    iDynTreeWrappers.plotFrame(link_transform_matlab, frameAxisSize, linewidthSize);
                    
    
end

joints = [{'Shoulder'}, {'Elbow'}, {'Wrist'}];

joint_torques = [];

abs(link_inertial_params);

for i = 1:size(load,2)
    
    joint_torques.shoulder(i) = link_inertial_params(1, 1) * gravity(3) * link_inertial_params(1, 3)  +...
                                link_inertial_params(2, 1) * gravity(3) * ( 2 * link_inertial_params(1, 3) + link_inertial_params(2, 3)  ) +...
                                ( link_inertial_params(3, 1) + load(i) ) * gravity(3) * ( 2 * ( link_inertial_params(1, 3) + link_inertial_params(2, 3) ) +  link_inertial_params(3, 3) );
                            
    joint_torques.elbow(i) =    link_inertial_params(2, 1) * gravity(3) *  link_inertial_params(2, 3) +...
                                ( link_inertial_params(3, 1) + load(i) ) * gravity(3) * ( 2 * link_inertial_params(2, 3)  +  link_inertial_params(3, 3) );
                            
    joint_torques.wrist(i) =    ( link_inertial_params(3, 1) + load(i) ) * gravity(3) *  link_inertial_params(3, 3);
    
end                    


joint_torques_table_header = {' 0 Kgs ' ;' 2 Kgs ';' 5 Kgs '};

shoulder = joint_torques.shoulder';
elbow = joint_torques.elbow';
wrist = joint_torques.wrist';

joint_torques_table = table(shoulder, elbow, wrist, 'RowNames', joint_torques_table_header);

% Get the table in string form.
TString = evalc('disp(joint_torques_table)');

% Use TeX Markup for bold formatting and underscores.
TString = strrep(TString,'<strong>','\bf');
TString = strrep(TString,'</strong>','\rm');
TString = strrep(TString,'_','\_');

% Get a fixed-width font.
FixedWidth = get(0,'FixedWidthFontName');

% Output the table using the annotation command.
annotation(gcf,'Textbox','String',TString,'Interpreter','Tex',...
           'FontName',FixedWidth,'Units','Normalized','Position',[0 0 1 1], 'FontSize', 16);