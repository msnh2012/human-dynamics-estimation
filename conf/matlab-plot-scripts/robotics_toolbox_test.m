close all;
clear all;
clc;

%% Load URDF using robotics toolbox

default_model = importrobot('/home/yeshi/software/robotology-superbuild/robotology/human-gazebo/humanSubject08/humanSubject08_66dof.urdf');
xsens_model = importrobot('/home/yeshi/software/robotology-superbuild/robotology/human-gazebo/humanSubject08/humanSubject08_66dof.urdf');

fH = figure('units','normalized','outerposition',[0 0 1 1]);

subplot(1,2,1)
show(default_model);
hold on;
legend('Default Configuration')

% % show(xsens_model);
% % hold on;
% % legend('IK Solution Configuration')

%% Load data
load('/home/yeshi/software/robotology-superbuild/robotology/human-dynamics-estimation/conf/xml/testData/matLogFile.mat');

data.jointPositions(:,1);

xsens_model_config = homeConfiguration(xsens_model);


for d = 1:size(data.jointPositions, 2)
    
    for i = 1:size(xsens_model_config, 2)
        
        configJointName =  xsens_model_config(i).JointName;
        
        dataJointIndex = find(strcmp(stateJointNames, configJointName));
        
        targetJointPosition = data.jointPositions(dataJointIndex, d);
        
        xsens_model_config(i).JointPosition = targetJointPosition;
        
    end
    
    subplot(1,2,2)
    show(xsens_model, xsens_model_config);
    legend('IK Solution Configuration');
    hold off;
    pause(0.01)
    
end

