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

LeftHandEstimatedWrenchInBaseFrame  = data.task1_wrenchEstimatesInBaseFrame(13:18,:)';
RightHandEstimatedWrenchInBaseFrame = data.task1_wrenchEstimatesInBaseFrame(19:24,:)';

order = 1;
framelen = 21;

sg_LeftHandEstimatedWrenchInBaseFrame = sgolayfilt(LeftHandEstimatedWrenchInBaseFrame, order, framelen);
sg_RightHandEstimatedWrenchInBaseFrame = sgolayfilt(RightHandEstimatedWrenchInBaseFrame, order, framelen);


for s = 1:3
    
    subplot(3,1,s);
    plot(LeftHandEstimatedWrenchInBaseFrame(:,s), 'LineWidth', lineWidth);
    hold on;
    plot(sg_LeftHandEstimatedWrenchInBaseFrame(:,s), 'LineWidth', lineWidth);
    hold on;
    %         ylim([-400 800])
    xlabel('Samples', 'FontSize', fontSize);
    ylabel(wrenchLegendString(s), 'Interpreter', 'latex', 'FontSize', fontSize);
    set (gca, 'FontSize' , fontSize)
    legend('Original Estimation', 'Savitzky-Golay Filtering',...
           'FontSize', fontSize, 'Location', 'Best');
end
