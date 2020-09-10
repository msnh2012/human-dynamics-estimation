close all;
clear all;
clc;

%% Plot parameters
fontSize  = 20;
lineWidth = 3;


%% Load data
load('/home/yeshi/software/robotology-superbuild/robotology/human-dynamics-estimation/conf/xml/testData/matLogFile.mat');

velLegendString = ["$v_x$", "$v_y$", "$v_z$","${\omega}_x$", "${\omega}_y$", "${\omega}_z$"];


baseLinkName  = 'Pelvis';
baseLinkIndex = find(strcmp(linkNames, baseLinkName)); 

HandLinkName  = 'RightHand';
HandLinkIndex = find(strcmp(linkNames, HandLinkName)); 

linkMeasurementsInworld = [];
linkPosition = [];
linkPositionSmoothed = [];
linkPositionDerivative = [];
linkPositionDerivativeInWorld = [];
time = data.time/10e9; %Nanoseconds to seconds conversion

%% Get measurements in world from mixed representation
for i = 1:size(linkData(baseLinkIndex).data.pose, 2) %% Assuming all the time series length is correct
    
    w_o_l = linkData(HandLinkIndex).data.pose(1:3, i);
    linkPosition(:,i) = w_o_l;
    w_H_l_w = iDynTree.Transform(iDynTree.Rotation.Identity, iDynTree.Position(w_o_l(1), w_o_l(2), w_o_l(3)));
    w_X_l_w = w_H_l_w.asAdjointTransform;
    
    linkMeasurementsInworld(:,i) = w_X_l_w.toMatlab * linkData(HandLinkIndex).data.velocity(:,i);

end

linkPositionSmoothed(1,:) = sgolayfilt(linkPosition(1, :)', 3, 11);
linkPositionSmoothed(2,:) = sgolayfilt(linkPosition(2, :)', 3, 11);
linkPositionSmoothed(3,:) = sgolayfilt(linkPosition(3, :)', 3, 11);


for i=1:3
    linkPositionDerivative(i,:) = diff(linkPositionSmoothed(i,:))/0.02;
    
end

for i = 2:size(linkData(baseLinkIndex).data.pose, 2)-1 %% Assuming all the time series length is correct
    
    w_o_l = linkData(HandLinkIndex).data.pose(1:3, i);
    linkPosition(:,i) = w_o_l;
    w_H_l_w = iDynTree.Transform(iDynTree.Rotation.Identity, iDynTree.Position(w_o_l(1), w_o_l(2), w_o_l(3)));
    w_X_l_w = w_H_l_w.asAdjointTransform;
    
    w_X_l_w_mat = w_X_l_w.toMatlab;
    
    linkPositionDerivativeInWorld(:,i) = linkPositionDerivative(:,i) + w_X_l_w_mat(1:3,4:6) * linkData(HandLinkIndex).data.velocity(4:6,i);

end


%% Plot velocity measurements
fH = figure('units','normalized','outerposition',[0 0 1 1]);

for i=1:3
    
    subplot(3,1,i)
    plot(linkData(HandLinkIndex).data.velocity(i,:)', 'LineWidth', lineWidth)
    hold on;
    plot(linkPositionDerivative(i,:)', 'LineWidth', lineWidth, 'LineStyle', '-.')
    hold on;
    plot(linkPositionDerivativeInWorld(i,:)',  'LineWidth', lineWidth, 'LineStyle', '--')
    hold on;
    plot(linkMeasurementsInworld(i,:)',  'LineWidth', lineWidth, 'LineStyle', '--')
    hold on;
    xlabel('Samples', 'FontSize', fontSize);
    ylabel(velLegendString(i), 'Interpreter', 'latex', 'FontSize', fontSize);
    set (gca, 'FontSize' , fontSize)
    legend('Xsens Measured Velocity', 'Xsens Position Derivative ${}^{A}\dot{O}_{L}$',...
           '${}^{A}\dot{O}_{L} + {}^{A}{O}_{L} \times {}^{A}{\omega}_{A, L}$', 'Xsens Measured Velocity Expressed in World ${}^{A}{V}_{A,L}$',...
           'FontSize', fontSize, 'Location', 'Best', 'Interpreter', 'latex');
    
end

a = axes;
t = title ("Hand link velocity measurement");
t.FontSize = fontSize;
a.Visible = 'off' ;
t.Visible = 'on' ;
axis(a,'fill')



