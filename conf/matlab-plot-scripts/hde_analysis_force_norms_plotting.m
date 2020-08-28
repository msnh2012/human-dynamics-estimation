close all;
clear all;
clc;

%% Plot parameters
fontSize = 20;
lineWidth = 2;

parulacolors = colormap(parula);
pinkcolors   = colormap(pink);
wintercolors = colormap(winter);
bonecolors   = colormap(bone);
autumncolors = colormap(autumn);
summercolors = colormap(summer);

%% Legend or Title Index
wrenchLegendString = ["$f_x [N]$", "$f_y [N]$", "$f_z [N]$","$m_x [Nm]$", "$m_y [Nm]$", "$m_z [Nm]$"];
wrenchEstimatesLegendString = ["$\hat{f}_x [N]$", "$\hat{f}_y [N]$", "$\hat{f}_z [N]$","$\hat{m}_x [Nm]$", "$\hat{m}_y [Nm]$", "$\hat{m}_z [Nm]$"];
wrenchSourceName = ["Left Foot Wrench", "Right Foot Wrench", "Left Hand Wrench", "Right Hand Wrench"];
momentumLegendString = ["$^{B}\underline{\dot{h}}_{L_x}$", "$^{B}\underline{\dot{h}}_{L_y}$", "$^{B}\underline{\dot{h}}_{L_z}$",...
                        "$^{B}\underline{\dot{h}}_{\omega_x}$", "$^{B}\underline{\dot{h}}_{\omega_y}$", "$^{B}\underline{\dot{h}}_{\omega_z}$"];
                    
%% Load data
load('/home/yeshi/software/robotology-superbuild/robotology/human-dynamics-estimation/conf/xml/testData/matLogFile.mat');

%% Measurement wrenches in link frame
LeftFootMeasuredWrenchInLinkFrame  = data.task1_wrenchMeasurementsInLinkFrame(1:6,:)';
RightFootMeasuredWrenchInLinkFrame = data.task1_wrenchMeasurementsInLinkFrame(7:12,:)';
LeftHandMeasuredWrenchInLinkFrame  = data.task1_wrenchMeasurementsInLinkFrame(13:18,:)';
RightHandMeasuredWrenchInLinkFrame = data.task1_wrenchMeasurementsInLinkFrame(19:24,:)';

normLeftFootMeasuredWrenchInLinkFrame = [];
normRightFootMeasuredWrenchInLinkFrame = [];

for i=1:size(LeftFootMeasuredWrenchInLinkFrame, 1)
    normLeftFootMeasuredWrenchInLinkFrame(i) = norm(LeftFootMeasuredWrenchInLinkFrame(i,1:3));
end

for i=1:size(RightFootMeasuredWrenchInLinkFrame, 1)
    normRightFootMeasuredWrenchInLinkFrame(i) = norm(RightFootMeasuredWrenchInLinkFrame(i,1:3));
end

%% Measurement wrenches in base frame
LeftFootMeasuredWrenchInBaseFrame  = data.task1_wrenchMeasurementsInBaseFrame(1:6,:)';
RightFootMeasuredWrenchInBaseFrame = data.task1_wrenchMeasurementsInBaseFrame(7:12,:)';
LeftHandMeasuredWrenchInBaseFrame  = data.task1_wrenchMeasurementsInBaseFrame(13:18,:)';
RightHandMeasuredWrenchInBaseFrame = data.task1_wrenchMeasurementsInBaseFrame(19:24,:)';

normLeftFootMeasuredWrenchInBaseFrame = [];
normRightFootMeasuredWrenchInBaseFrame = [];

for i=1:size(LeftFootMeasuredWrenchInBaseFrame, 1)
    normLeftFootMeasuredWrenchInBaseFrame(i) = norm(LeftFootMeasuredWrenchInBaseFrame(i,1:3));
end

for i=1:size(RightFootMeasuredWrenchInBaseFrame, 1)
    normRightFootMeasuredWrenchInBaseFrame(i) = norm(RightFootMeasuredWrenchInBaseFrame(i,1:3));
end

sumFeetWrenchMeasurementsInBaseFrame = LeftFootMeasuredWrenchInBaseFrame + RightFootMeasuredWrenchInBaseFrame;

feetForceMeasurementsNormInBaseFrame = [];

for i=1:size(sumFeetWrenchMeasurementsInBaseFrame, 1)
    feetForceMeasurementsNormInBaseFrame(i) = norm(sumFeetWrenchMeasurementsInBaseFrame(i,1:3));
end

%% Measurement wrenches in world frame
LeftFootMeasuredWrenchInWorldFrame  = data.task1_wrenchMeasurementsInWorldFrame(1:6,:)';
RightFootMeasuredWrenchInWorldFrame = data.task1_wrenchMeasurementsInWorldFrame(7:12,:)';
LeftHandMeasuredWrenchInWorldFrame  = data.task1_wrenchMeasurementsInWorldFrame(13:18,:)';
RightHandMeasuredWrenchInWorldFrame = data.task1_wrenchMeasurementsInWorldFrame(19:24,:)';

normLeftFootMeasuredWrenchInWorldFrame = [];
normRightFootMeasuredWrenchInWorldFrame = [];

for i=1:size(LeftFootMeasuredWrenchInWorldFrame, 1)
    normLeftFootMeasuredWrenchInWorldFrame(i) = norm(LeftFootMeasuredWrenchInWorldFrame(i,1:3));
end

for i=1:size(RightFootMeasuredWrenchInBaseFrame, 1)
    normRightFootMeasuredWrenchInWorldFrame(i) = norm(RightFootMeasuredWrenchInWorldFrame(i,1:3));
end

sumFeetWrenchMeasurementsInWorldFrame = LeftFootMeasuredWrenchInWorldFrame + RightFootMeasuredWrenchInWorldFrame;

feetForceMeasurementsNormInWorldFrame = [];

for i=1:size(sumFeetWrenchMeasurementsInWorldFrame, 1)
    feetForceMeasurementsNormInWorldFrame(i) = norm(sumFeetWrenchMeasurementsInWorldFrame(i,1:3));
end

%% Measurement wrenches in centroidal frame
LeftFootMeasuredWrenchInCentroidalFrame  = data.task1_wrenchMeasurementsInCentroidalFrame(1:6,:)';
RightFootMeasuredWrenchInCentroidalFrame = data.task1_wrenchMeasurementsInCentroidalFrame(7:12,:)';
LeftHandMeasuredWrenchInCentroidalFrame  = data.task1_wrenchMeasurementsInCentroidalFrame(13:18,:)';
RightHandMeasuredWrenchInCentroidalFrame = data.task1_wrenchMeasurementsInCentroidalFrame(19:24,:)';

normLeftFootMeasuredWrenchInCentroidalFrame = [];
normRightFootMeasuredWrenchInCentroidalFrame = [];

for i=1:size(LeftFootMeasuredWrenchInCentroidalFrame, 1)
    normLeftFootMeasuredWrenchInCentroidalFrame(i) = norm(LeftFootMeasuredWrenchInCentroidalFrame(i,1:3));
end

for i=1:size(RightFootMeasuredWrenchInCentroidalFrame, 1)
    normRightFootMeasuredWrenchInCentroidalFrame(i) = norm(RightFootMeasuredWrenchInCentroidalFrame(i,1:3));
end

sumFeetWrenchMeasurementsInCentroidalFrame = LeftFootMeasuredWrenchInCentroidalFrame + RightFootMeasuredWrenchInCentroidalFrame;

feetForceMeasurementsNormInCentroidalFrame = [];

for i=1:size(sumFeetWrenchMeasurementsInCentroidalFrame, 1)
    feetForceMeasurementsNormInCentroidalFrame(i) = norm(sumFeetWrenchMeasurementsInCentroidalFrame(i,1:3));
end



%% Plot feet forces norm in link and base frame
fH = figure('units','normalized','outerposition',[0 0 1 1]);

subplot(2,1,1)
plot(normLeftFootMeasuredWrenchInLinkFrame, 'LineWidth', lineWidth, 'Color', autumncolors(80,:));
hold on;
plot(normLeftFootMeasuredWrenchInBaseFrame, '--', 'LineWidth', lineWidth, 'Color', autumncolors(120,:));
hold on;
plot(normLeftFootMeasuredWrenchInWorldFrame, '-.', 'LineWidth', lineWidth, 'Color', autumncolors(160,:));
hold on;
plot(normLeftFootMeasuredWrenchInCentroidalFrame, '-.', 'LineWidth', lineWidth, 'Color', autumncolors(200,:));
hold on;
legend('Left Foot Link Frame', 'Left Foot Base Frame', 'Left Foot World Frame', 'Left Foot Centroidal Frame', 'FontSize', fontSize, 'Location', 'Best');
    
subplot(2,1,2)
plot(normRightFootMeasuredWrenchInLinkFrame, 'LineWidth', lineWidth, 'Color', wintercolors(80,:));
hold on;
plot(normRightFootMeasuredWrenchInBaseFrame, '--', 'LineWidth', lineWidth, 'Color', wintercolors(120,:));
hold on;
plot(normRightFootMeasuredWrenchInWorldFrame, '-.', 'LineWidth', lineWidth, 'Color', wintercolors(160,:));
hold on;
plot(normRightFootMeasuredWrenchInCentroidalFrame, '-.', 'LineWidth', lineWidth, 'Color', wintercolors(200,:));
hold on;
legend('Right Foot Link Frame', 'Right Foot Base Frame', 'Right Foot World Frame', 'Right Foot Centroidal Frame', 'FontSize', fontSize, 'Location', 'Best');

a = axes;
t = title ("Feet forces norm in link, base, world and centroidal frame");
t.FontSize = fontSize;
a.Visible = 'off' ;
t.Visible = 'on' ;

%% Plot feet forces norms in common frame
fH = figure('units','normalized','outerposition',[0 0 1 1]);

subplot(3,1,1)
plot(normLeftFootMeasuredWrenchInBaseFrame + normRightFootMeasuredWrenchInBaseFrame, 'LineWidth', lineWidth, 'Color', pinkcolors(120,:));
hold on;
plot(feetForceMeasurementsNormInBaseFrame, '--', 'LineWidth', lineWidth, 'Color', bonecolors(120,:));
hold on;
legend('Sum of force Norms Base Frame', 'Norm of sum of forces norm Base Frame', 'FontSize', fontSize, 'Location', 'Best');

subplot(3,1,2)
plot(normLeftFootMeasuredWrenchInWorldFrame + normRightFootMeasuredWrenchInWorldFrame, 'LineWidth', lineWidth, 'Color', pinkcolors(120,:));
hold on;
plot(feetForceMeasurementsNormInWorldFrame, '--', 'LineWidth', lineWidth, 'Color', bonecolors(120,:));
hold on;
legend('Sum of force Norms World Frame', 'Norm of sum of forces norm World Frame', 'FontSize', fontSize, 'Location', 'Best');

subplot(3,1,3)
plot(normLeftFootMeasuredWrenchInCentroidalFrame + normRightFootMeasuredWrenchInCentroidalFrame, 'LineWidth', lineWidth, 'Color', pinkcolors(120,:));
hold on;
plot(feetForceMeasurementsNormInCentroidalFrame, '--', 'LineWidth', lineWidth, 'Color', bonecolors(120,:));
hold on;
legend('Sum of force Norms Centroidal Frame', 'Norm of sum of forces norm Centroidal Frame', 'FontSize', fontSize, 'Location', 'Best');

a = axes;
t = title ("Force Norms comparison base, world and centroidal frame");
t.FontSize = fontSize;
a.Visible = 'off' ;
t.Visible = 'on' ;
