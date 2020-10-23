close all;
clear all;
clc;

%% Plot parameters
fontSize  = 20;
lineWidth = 3;


%% Legend or Title Index
wrenchLegendString = ["$f_x [N]$", "$f_y [N]$", "$f_z [N]$","$m_x [Nm]$", "$m_y [Nm]$", "$m_z [Nm]$"];

%% Load data
load('/home/yeshi/software/robotology-superbuild/robotology/human-dynamics-estimation/conf/xml/testData/matLogFile.mat');

subjectMass = 53.5;
I_g         = [0 0 9.81]';
I_f_g       = (subjectMass) * I_g;

baseLinkName  = 'Pelvis';
baseLinkIndex = find(strcmp(linkNames, baseLinkName)); 

leftFootLinkName  = 'LeftFoot';
leftFootLinkIndex = find(strcmp(linkNames, leftFootLinkName));

rightFootLinkName  = 'RightFoot';
rightFootLinkIndex = find(strcmp(linkNames, rightFootLinkName));

left_foot_f_g  = [];
right_foot_f_g = [];
b_f_g = [];

%% Get left foot wrench measurements expressed in link frame
LeftFootMeasuredWrenchInLinkFrame  = data.task1_wrenchMeasurementsInLinkFrame(1:6,:)';
RightFootMeasuredWrenchInLinkFrame = data.task1_wrenchMeasurementsInLinkFrame(7:12,:)';

LeftFootMeasuredWrenchInLinkFrame_in_base = [];
RightFootMeasuredWrenchInLinkFrame_in_base = [];

for i = 1:size(linkData(baseLinkIndex).data.pose, 2) %% Assuming all the time series length is correct
    
    w_R_b_rpy = linkData(baseLinkIndex).data.pose(4:6, i);
    w_R_b = iDynTree.Rotation.RPY(w_R_b_rpy(1), w_R_b_rpy(2), w_R_b_rpy(3));
    b_R_w = w_R_b.inverse;
    
    % Gravity vector in base
    b_f_g(i,:) = b_R_w.toMatlab * I_f_g;
    
    w_R_left_foot_rpy = linkData(leftFootLinkIndex).data.pose(4:6, i);
    w_R_left_foot = iDynTree.Rotation.RPY(w_R_left_foot_rpy(1), w_R_left_foot_rpy(2), w_R_left_foot_rpy(3));
    
    w_R_right_foot_rpy = linkData(rightFootLinkIndex).data.pose(4:6, i);
    w_R_right_foot = iDynTree.Rotation.RPY(w_R_right_foot_rpy(1), w_R_right_foot_rpy(2), w_R_right_foot_rpy(3));
     
    LeftFootMeasuredWrenchInLinkFrame_in_base(i,:) = b_R_w.toMatlab * w_R_left_foot.toMatlab * LeftFootMeasuredWrenchInLinkFrame(i, 1:3)';
    RightFootMeasuredWrenchInLinkFrame_in_base(i,:) = b_R_w.toMatlab * w_R_right_foot.toMatlab * RightFootMeasuredWrenchInLinkFrame(i, 1:3)';
    
end

%% Plot forces in assumed link and xsens base frames
fH = figure('units','normalized','outerposition',[0 0 1 1]);

for s = 1:3
    
    subplot(3,1,s);
    plot(LeftFootMeasuredWrenchInLinkFrame_in_base(:,s), 'LineWidth', lineWidth);
    hold on;
    plot(RightFootMeasuredWrenchInLinkFrame_in_base(:,s), 'LineWidth', lineWidth);
    hold on;
    plot(LeftFootMeasuredWrenchInLinkFrame_in_base(:,s) + RightFootMeasuredWrenchInLinkFrame_in_base(:,s), 'LineWidth', lineWidth);
    hold on;
    %         ylim([-400 800])
    xlabel('Samples', 'FontSize', fontSize);
    ylabel(wrenchLegendString(s), 'Interpreter', 'latex', 'FontSize', fontSize);
    set (gca, 'FontSize' , fontSize)
    legend('Left Foot', 'Right Foot', 'Left + Right',...
           'FontSize', fontSize, 'Location', 'Best');
end

a = axes;
t = title ("Foot measured forces expressed in Base Frame");
t.FontSize = fontSize;
a.Visible = 'off' ;
t.Visible = 'on' ;
axis(a,'fill')


%% Plot forces in assumed link and xsens base frames
fH = figure('units','normalized','outerposition',[0 0 1 1]);

for s = 1:3
    
    subplot(3,1,s);
    plot(LeftFootMeasuredWrenchInLinkFrame_in_base(:,s) + RightFootMeasuredWrenchInLinkFrame_in_base(:,s), 'LineWidth', lineWidth);
    hold on;
    plot(b_f_g(:,s), 'LineWidth', lineWidth);
    hold on;
    plot(LeftFootMeasuredWrenchInLinkFrame_in_base(:,s) + RightFootMeasuredWrenchInLinkFrame_in_base(:,s) - b_f_g(:,s), 'LineWidth', lineWidth);
    hold on;
    %         ylim([-400 800])
    xlabel('Samples', 'FontSize', fontSize);
    ylabel(wrenchLegendString(s), 'Interpreter', 'latex', 'FontSize', fontSize);
    set (gca, 'FontSize' , fontSize)
    legend('Left + Right ', 'Weight', 'Diff',...
           'FontSize', fontSize, 'Location', 'Best');
end

a = axes;
t = title ("Foot measured forces expressed Vs Weight in Base Frame ");
t.FontSize = fontSize;
a.Visible = 'off' ;
t.Visible = 'on' ;
axis(a,'fill')


%% Norm computation
leftFootForcesNormInLinkFrame = [];
rightFootForcesNormInLinkFrame = [];
feetForcesNormInLinkFrame = [];
feetForcesNormInLinkFrame_in_base = [];

for i=1:size(RightFootMeasuredWrenchInLinkFrame, 1)
    leftFootForcesNormInLinkFrame(i) = norm(LeftFootMeasuredWrenchInLinkFrame(i,1:3));
    rightFootForcesNormInLinkFrame(i) = norm(RightFootMeasuredWrenchInLinkFrame(i,1:3));
    feetForcesNormInLinkFrame(i) = norm(LeftFootMeasuredWrenchInLinkFrame(i,1:3) + RightFootMeasuredWrenchInLinkFrame(i,1:3));
end

for i=1:size(RightFootMeasuredWrenchInLinkFrame_in_base, 1)
    feetForcesNormInLinkFrame_in_base(i) = norm(LeftFootMeasuredWrenchInLinkFrame_in_base(i,1:3) + RightFootMeasuredWrenchInLinkFrame_in_base(i,1:3));
end

fH = figure('units','normalized','outerposition',[0 0 1 1]);

plot(leftFootForcesNormInLinkFrame, 'LineWidth', lineWidth);
hold on;
plot(rightFootForcesNormInLinkFrame, 'LineWidth', lineWidth);
hold on;
plot(feetForcesNormInLinkFrame, 'LineWidth', lineWidth);
hold on;
legend('Left', 'Right', 'Left + Right',...
           'FontSize', fontSize, 'Location', 'Best');
xlabel('Samples', 'FontSize', fontSize);
ylabel('Force Norm [N]', 'Interpreter', 'latex', 'FontSize', fontSize);
a = axes;
t = title ("Foot measured forces norm");
t.FontSize = fontSize;
a.Visible = 'off' ;
t.Visible = 'on' ;
axis(a,'fill')

