close all;
clear all;
clc;

N=8;
C = linspecer(N);
L = lines(N);
lineStyles = linspecer(N,'qualitative');

%% Plot parameters
fontSize  = 22;
legendFontSize  = 20;
lineWidth = 4;

gravity = 9.81;
subjectMass = 79.5;
objectMass = 9.55;

mg = subjectMass * gravity;


%% Load urdf model
subject = 'humanSubject03';
modelPath = strcat('/home/yeshi/software/robotology-superbuild/src/human-gazebo/', subject, '/');
fileName  = strcat(subject, '_66dof.urdf');


jointOrder = {'jL5S1_rotx';'jRightHip_rotx';'jLeftHip_rotx';'jLeftHip_roty';'jLeftHip_rotz';'jLeftKnee_rotx';'jLeftKnee_roty';'jLeftKnee_rotz';'jLeftAnkle_rotx';'jLeftAnkle_roty';'jLeftAnkle_rotz';'jLeftBallFoot_rotx';'jLeftBallFoot_roty';'jLeftBallFoot_rotz';'jRightHip_roty';'jRightHip_rotz';'jRightKnee_rotx';'jRightKnee_roty';'jRightKnee_rotz';'jRightAnkle_rotx';'jRightAnkle_roty';'jRightAnkle_rotz';'jRightBallFoot_rotx';'jRightBallFoot_roty';'jRightBallFoot_rotz';'jL5S1_roty';'jL5S1_rotz';'jL4L3_rotx';'jL4L3_roty';'jL4L3_rotz';'jL1T12_rotx';'jL1T12_roty';'jL1T12_rotz';'jT9T8_rotx';'jT9T8_roty';'jT9T8_rotz';'jLeftC7Shoulder_rotx';'jT1C7_rotx';'jRightC7Shoulder_rotx';'jRightC7Shoulder_roty';'jRightC7Shoulder_rotz';'jRightShoulder_rotx';'jRightShoulder_roty';'jRightShoulder_rotz';'jRightElbow_rotx';'jRightElbow_roty';'jRightElbow_rotz';'jRightWrist_rotx';'jRightWrist_roty';'jRightWrist_rotz';'jT1C7_roty';'jT1C7_rotz';'jC1Head_rotx';'jC1Head_roty';'jC1Head_rotz';'jLeftC7Shoulder_roty';'jLeftC7Shoulder_rotz';'jLeftShoulder_rotx';'jLeftShoulder_roty';'jLeftShoulder_rotz';'jLeftElbow_rotx';'jLeftElbow_roty';'jLeftElbow_rotz';'jLeftWrist_rotx';'jLeftWrist_roty';'jLeftWrist_rotz'};

KinDynModel = iDynTreeWrappers.loadReducedModel(jointOrder,'Pelvis',modelPath,fileName,false);

%% Legend or Title Index
wrenchLegendString = ["$f_x [N]$", "$f_y [N]$", "$f_z [N]$","$m_x [Nm]$", "$m_y [Nm]$", "$m_z [Nm]$"];

dataset_dir = '/home/yeshi/Desktop/non_collocated_wrench_estimation_dataset/';
dataset_type = 'experiments_dataset';
dataset_source = 'FTShoes_Dataset';
subject = 'sub02';
dataset = 'static';

dataset_path = strcat(dataset_dir,'/',dataset_type,'/',dataset_source,'/',subject,'/',dataset,'/hde/1e-4'); 

%% Load data
withoutSOT = load(strcat(dataset_path,'/withoutSOT.mat'));
withSOT = load(strcat(dataset_path,'/withSOT.mat'));

Data = withoutSOT;


baseIndex = find(strcmp(Data.linkNames, 'Pelvis')); 

leftFootIndex = find(strcmp(Data.linkNames, 'LeftFoot'));
rightFootIndex = find(strcmp(Data.linkNames, 'RightFoot'));

leftHandIndex = find(strcmp(Data.linkNames, 'LeftHand'));
rightHandIndex = find(strcmp(Data.linkNames, 'RightHand'));


%% Ger measured feet wrench in inertial frame
LeftFootMeasuredWrenchInWorldFrame  = Data.data.task2_wrenchMeasurementsInWorldFrame(1:6,:)';
RightFootMeasuredWrenchInWorldFrame = Data.data.task2_wrenchMeasurementsInWorldFrame(7:12,:)';

y1 = [];
y2 = [];
X = [];
hands_sum = [];
hands_norm = [];

%% Compute hand forces assuming force balance in respect to inertial frame
for i = 1:size(RightFootMeasuredWrenchInWorldFrame, 1)
    
    
    leftFootPosition = Data.linkData(leftFootIndex).data.pose(1:3, i);
    rightFootPosition = Data.linkData(rightFootIndex).data.pose(1:3, i);
    
    leftHandPosition = Data.linkData(leftHandIndex).data.pose(1:3, i);
    rightHandPosition = Data.linkData(rightHandIndex).data.pose(1:3, i);
    
    sk_leftFootPosition = skw(leftFootPosition);
    sk_rightFootPosition = skw(rightFootPosition);
    
    sk_leftHandPosition = skw(leftHandPosition);
    sk_rightHandPosition = skw(rightHandPosition);
    
    %% Computing hand force estimates
    y1(:,i) = [0 0 mg]' - LeftFootMeasuredWrenchInWorldFrame(i, 1:3)' - RightFootMeasuredWrenchInWorldFrame(i, 1:3)';
    y2(:,i) = - sk_leftFootPosition * LeftFootMeasuredWrenchInWorldFrame(i, 1:3)' - sk_rightFootPosition * RightFootMeasuredWrenchInWorldFrame(i, 1:3)';
     
    Y = [y1(:,i); y2(:,i)];
    
    A = [eye(3) eye(3); sk_leftHandPosition sk_rightHandPosition];
    
    X(:,i) = pinv(A, 1e-4) * Y;
    
    hands_sum(:,i) = X(1:3,i) + X(4:6,i);
    hands_norm(i) = norm(hands_sum(:,i));
    
end


%% Plot hand force estimates
fH = figure('units','normalized','outerposition',[0 0 1 1]);
tl = tiledlayout(3,1);

for i = 1:3
    
    ax = nexttile;
    plot(X(i, :), 'LineWidth', lineWidth);
    hold on;
    plot(X(i + 3, :), 'LineWidth', lineWidth);
    hold on;
    plot(X(i, :) + X(i + 3, :), 'LineWidth', lineWidth);
    hold on;
    plot(y1(i, :)', 'LineWidth', lineWidth);
    
    ylabel(wrenchLegendString(i), 'Interpreter', 'latex', 'FontSize', fontSize);
    set (gca, 'FontSize' , fontSize)
    set (gca, 'ColorOrder' , C)
    axis tight
    
% %     lh = legend('Left Hand', 'Right Hand', 'Sum', 'FontSize', legendFontSize,...
% %            'Location', 'Best', 'Orientation','Vertical');

end


lh = legend(ax, 'Left Hand', 'Right Hand', 'Sum', '(mg - feet measurements)' ,'FontSize', legendFontSize,...
           'Location', 'NorthOutside', 'Orientation','Vertical', 'NumColumns', 2);
lh.Layout.Tile = 'North';

txt = title(tl, "Hands Force Estimation in Inertial Frame", 'FontSize', fontSize, 'fontweight','bold');
txt.Interpreter= 'none';






%%Skew symmetric matrix
function sk = skw(u)
    sk = [0      -u(3)   u(2);
          u(3)    0     -u(1);
         -u(2)    u(1)   0  ];
end
