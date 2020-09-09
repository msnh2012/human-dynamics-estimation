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

%% Get measurements in world from mixed representation
for i = 1:size(linkData(baseLinkIndex).data.pose, 2) %% Assuming all the time series length is correct
    
    w_o_l = linkData(HandLinkIndex).data.pose(1:3, i);
    w_H_l_w = iDynTree.Transform(iDynTree.Rotation.Identity, iDynTree.Position(w_o_l(1), w_o_l(2), w_o_l(2)));
    w_X_l_w = w_H_l_w.asAdjointTransform;
    
    linkMeasurementsInworld(:,i) = w_X_l_w.toMatlab * linkData(HandLinkIndex).data.velocity(:,i);

end

%% Plot velocity measurements
fH = figure('units','normalized','outerposition',[0 0 1 1]);

for i=1:6
    
    subplot(2,3,i)
    plot(linkData(HandLinkIndex).data.velocity(i,:)', 'LineWidth', lineWidth)
    hold on;
    plot(linkMeasurementsInworld(i,:)', 'LineWidth', lineWidth)
    hold on;
    xlabel('Samples', 'FontSize', fontSize);
    ylabel(velLegendString(i), 'Interpreter', 'latex', 'FontSize', fontSize);
    set (gca, 'FontSize' , fontSize)
    legend('$ {}^{L[A]}{V}_{A,L}$', '${}^{A}{V}_{A,L}$',...
           'FontSize', fontSize, 'Location', 'Best', 'Interpreter', 'latex');
    
end

a = axes;
t = title ("Hand link velocity measurement");
t.FontSize = fontSize;
a.Visible = 'off' ;
t.Visible = 'on' ;
axis(a,'fill')