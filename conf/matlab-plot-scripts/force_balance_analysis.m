close all;
clear all;
clc;

%% Plot parameters
fontSize  = 20;
lineWidth = 3;

%% Load data
load('/home/yeshi/software/robotology-superbuild/robotology/human-dynamics-estimation/conf/xml/testData/matLogFile.mat');

subjectMass = 55.2;
I_g         = [0 0 9.81]';
I_f_g       = (subjectMass/2) * I_g;

baseLinkName  = 'Pelvis';
baseLinkIndex = find(strcmp(linkNames, baseLinkName)); 

leftFootLinkName  = 'LeftFoot';
leftFootLinkIndex = find(strcmp(linkNames, leftFootLinkName));

rightFootLinkName  = 'RightFoot';
rightFootLinkIndex = find(strcmp(linkNames, rightFootLinkName));

left_foot_f_g  = [];
right_foot_f_g = [];

for i = 1:size(linkData(baseLinkIndex).data.pose, 2) %% Assuming all the time series length is correct
    
    w_R_b_rpy = linkData(baseLinkIndex).data.pose(4:6, i);
    w_R_b = iDynTree.Rotation.RPY(w_R_b_rpy(1), w_R_b_rpy(2), w_R_b_rpy(3));
    
    w_R_left_foot_rpy = linkData(leftFootLinkIndex).data.pose(4:6, i);
    w_R_left_foot = iDynTree.Rotation.RPY(w_R_left_foot_rpy(1), w_R_left_foot_rpy(2), w_R_left_foot_rpy(3));
    
    w_R_right_foot_rpy = linkData(rightFootLinkIndex).data.pose(4:6, i);
    w_R_right_foot = iDynTree.Rotation.RPY(w_R_right_foot_rpy(1), w_R_right_foot_rpy(2), w_R_right_foot_rpy(3));
    
    %% gravity wrench expressed in left foot frame
    left_foot_R_w = w_R_left_foot.inverse;  
    left_foot_f_g(i, :) = left_foot_R_w.toMatlab * I_f_g;
    
    %% gravity wrench expressed in right foot frame
    right_foot_R_w = w_R_right_foot.inverse;  
    right_foot_f_g(i, :) = right_foot_R_w.toMatlab * I_f_g;
    
end

%% Get left foot wrench measurements expressed in link frame
LeftFootMeasuredWrenchInLinkFrame  = data.task1_wrenchMeasurementsInLinkFrame(1:6,:)';
RightFootMeasuredWrenchInLinkFrame = data.task1_wrenchMeasurementsInLinkFrame(7:12,:)';

%% Legend or Title Index
wrenchLegendString = ["$f_x [N]$", "$f_y [N]$", "$f_z [N]$","$m_x [Nm]$", "$m_y [Nm]$", "$m_z [Nm]$"];


%% Plot foot wrench measurements in link frame and half of weight expressed in same link frame

fH = figure('units','normalized','outerposition',[0 0 1 1]);

plotIndex = 1;
for s = 1:2:6
    
    subplot(3,2,s);
    plot(LeftFootMeasuredWrenchInLinkFrame(:,plotIndex), 'LineWidth', lineWidth);
    hold on;
    plot(left_foot_f_g(:,plotIndex), 'LineWidth', lineWidth, 'LineStyle', '--');
    hold on;
    %         ylim([-400 800])
    xlabel('Samples', 'FontSize', fontSize);
    ylabel(wrenchLegendString(plotIndex), 'Interpreter', 'latex', 'FontSize', fontSize);
    set (gca, 'FontSize' , fontSize)
    legend('Left Foot Measured Force in Left Foot', 'Half of Gravity Force in Left Foot',...
           'FontSize', fontSize, 'Location', 'Best');
       
    plotIndex = plotIndex+1;
    
end


plotIndex = 1;
for s = 2:2:6
    
    subplot(3,2,s);
    plot(RightFootMeasuredWrenchInLinkFrame(:,plotIndex), 'LineWidth', lineWidth);
    hold on;
    plot(right_foot_f_g(:,plotIndex), 'LineWidth', lineWidth, 'LineStyle', '--');
    hold on;
    %         ylim([-400 800])
    xlabel('Samples', 'FontSize', fontSize);
    ylabel(wrenchLegendString(plotIndex), 'Interpreter', 'latex', 'FontSize', fontSize);
    set (gca, 'FontSize' , fontSize)
    legend('Right Foot Measured Force in Right Foot', 'Half of Gravity Force in Right Foot',...
           'FontSize', fontSize, 'Location', 'Best');
    
    plotIndex = plotIndex+1;
    
end

a = axes;
t = title ("Foot measured force vs half of gravity force expressed in respective link frame");
t.FontSize = fontSize;
a.Visible = 'off' ;
t.Visible = 'on' ;
axis(a,'fill')


%% Compute norms

LeftFootMeasuredWrenchInLinkFrameNorm = [];
RightFootMeasuredWrenchInLinkFrameNorm = [];

left_foot_f_g_norm  = [];
right_foot_f_g_norm = [];

for i=1:size(LeftFootMeasuredWrenchInLinkFrame, 1)
    LeftFootMeasuredWrenchInLinkFrameNorm(i) = norm(LeftFootMeasuredWrenchInLinkFrame(i,1:3));
    RightFootMeasuredWrenchInLinkFrameNorm(i) = norm(RightFootMeasuredWrenchInLinkFrame(i,1:3));
    left_foot_f_g_norm(i) = norm(left_foot_f_g(i,:));
    right_foot_f_g_norm(i) = norm(right_foot_f_g(i,:));
end

fH = figure('units','normalized','outerposition',[0 0 1 1]);

subplot(2,2,1)
plot(LeftFootMeasuredWrenchInLinkFrameNorm, 'LineWidth', lineWidth);
hold on;
plot(left_foot_f_g_norm, 'LineWidth', lineWidth, 'LineStyle', '--');
hold on;
legend('Norm of Left Foot Measured Force in Left Foot', 'Norm of Half of Gravity Force in Left Foot',...
       'FontSize', fontSize, 'Location', 'Best');
       
subplot(2,2,3)
plot(LeftFootMeasuredWrenchInLinkFrameNorm - left_foot_f_g_norm, 'LineWidth', lineWidth);
hold on;
legend('Difference', 'FontSize', fontSize, 'Location', 'Best');

subplot(2,2,2)
plot(RightFootMeasuredWrenchInLinkFrameNorm, 'LineWidth', lineWidth);
hold on;
plot(right_foot_f_g_norm, 'LineWidth', lineWidth, 'LineStyle', '--');
hold on;
legend('Norm of Right Foot Measured Force in Right Foot', 'Norm of Half of Gravity Force in Right Foot',...
       'FontSize', fontSize, 'Location', 'Best');
       
subplot(2,2,4)
plot(RightFootMeasuredWrenchInLinkFrameNorm - right_foot_f_g_norm, 'LineWidth', lineWidth);
hold on;
legend('Difference', 'FontSize', fontSize, 'Location', 'Best');

a = axes;
t = title ("Norms comparison");
t.FontSize = fontSize;
a.Visible = 'off' ;
t.Visible = 'on' ;
axis(a,'fill')
