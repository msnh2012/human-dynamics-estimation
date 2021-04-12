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
lineWidth = 3;

gravity = 9.81;
objectMass = 5;

%% Legend or Title Index
wrenchLegendString = ["$f_x [N]$", "$f_y [N]$", "$f_z [N]$","$m_x [Nm]$", "$m_y [Nm]$", "$m_z [Nm]$"];

dataset_dir = '/home/yeshi/Desktop/non_collocated_wrench_estimation_dataset/';
dataset_type = 'covariance_tuning_dataset';
dataset_source = 'AMTI_Dataset';
subject = 'sub03';
dataset = 'tpose_right_5kgs/hde';

subjectMass = 0;

if subject == 'sub02'

    subjectMass = 79.4;

elseif subject == 'sub03'
    
    subjectMass = 75.4;

end

dataset_path = strcat(dataset_dir,'/',dataset_type,'/',dataset_source,'/',subject,'/',dataset,'/'); 

%% Load data for the hands force/torque measurement covariance from 1e-4 to 1e4
dataset_dir_names = ["1e-4", "1e0", "1e4"];

experiments_dataset = [];

feet_wrench_measurements = [];

hands_wrench_measurements = [];
hands_wrench_estimates = [];

startIndex = 180;

for i = 1:size(dataset_dir_names, 2)
    
    experiments_dataset(i).withoutSOT = load(strcat(dataset_path,  dataset_dir_names(i),'/withoutSOT.mat'));
    experiments_dataset(i).withSOT = load(strcat(dataset_path,  dataset_dir_names(i), '/withSOT.mat'));
    
    %% Get feet measurements in world
    feet_wrench_measurements(i).withoutSOT.leftfoot = experiments_dataset(i).withoutSOT.data.task2_wrenchMeasurementsInWorldFrame(1:6, startIndex : end )'; 
    feet_wrench_measurements(i).withoutSOT.rightfoot = experiments_dataset(i).withoutSOT.data.task2_wrenchMeasurementsInWorldFrame(7:12, startIndex : end )';
    feet_wrench_measurements(i).withoutSOT.sum = feet_wrench_measurements(i).withoutSOT.leftfoot + feet_wrench_measurements(i).withoutSOT.rightfoot;
    
    for j = 1:size(feet_wrench_measurements(i).withoutSOT.sum, 1)
        feet_wrench_measurements(i).withoutSOT.force_norm(j) = norm(feet_wrench_measurements(i).withoutSOT.sum(j,1:3));
    end
    
    
    %% Get hands measurements vs estimates in world
    hands_wrench_measurements(i).withoutSOT.lefthand = experiments_dataset(i).withoutSOT.data.task2_wrenchMeasurementsInWorldFrame(13:18, startIndex : end )'; 
    hands_wrench_measurements(i).withoutSOT.righthand = experiments_dataset(i).withoutSOT.data.task2_wrenchMeasurementsInWorldFrame(19:24, startIndex : end )';
    hands_wrench_measurements(i).withoutSOT.sum = hands_wrench_measurements(i).withoutSOT.lefthand + hands_wrench_measurements(i).withoutSOT.righthand;
    
    for j = 1:size(hands_wrench_measurements(i).withoutSOT.sum, 1)
        hands_wrench_measurements(i).withoutSOT.force_norm(j) = norm(hands_wrench_measurements(i).withoutSOT.sum(j,1:3));
    end
    
    hands_wrench_measurements(i).withSOT.lefthand = experiments_dataset(i).withSOT.data.task2_wrenchMeasurementsInWorldFrame(13:18, startIndex : end )'; 
    hands_wrench_measurements(i).withSOT.righthand = experiments_dataset(i).withSOT.data.task2_wrenchMeasurementsInWorldFrame(19:24, startIndex : end )';
    hands_wrench_measurements(i).withSOT.sum = hands_wrench_measurements(i).withSOT.lefthand + hands_wrench_measurements(i).withSOT.righthand;
    
    for j = 1:size(hands_wrench_measurements(i).withSOT.sum, 1)
        hands_wrench_measurements(i).withSOT.force_norm(j) = norm(hands_wrench_measurements(i).withSOT.sum(j,1:3));
    end
    
    hands_wrench_estimates(i).withoutSOT.lefthand = experiments_dataset(i).withoutSOT.data.task2_wrenchEstimatesInWorldFrame(13:18, startIndex : end )'; 
    hands_wrench_estimates(i).withoutSOT.righthand = experiments_dataset(i).withoutSOT.data.task2_wrenchEstimatesInWorldFrame(19:24, startIndex : end )';
    hands_wrench_estimates(i).withoutSOT.sum = hands_wrench_estimates(i).withoutSOT.lefthand + hands_wrench_estimates(i).withoutSOT.righthand;
    
    for j = 1:size(hands_wrench_estimates(i).withoutSOT.sum, 1)
        hands_wrench_estimates(i).withoutSOT.force_norm(j) = norm(hands_wrench_estimates(i).withoutSOT.sum(j,1:3));
    end
    
    hands_wrench_estimates(i).withSOT.lefthand = experiments_dataset(i).withSOT.data.task2_wrenchEstimatesInWorldFrame(13:18, startIndex : end )'; 
    hands_wrench_estimates(i).withSOT.righthand = experiments_dataset(i).withSOT.data.task2_wrenchEstimatesInWorldFrame(19:24, startIndex : end )';
    hands_wrench_estimates(i).withSOT.sum = hands_wrench_estimates(i).withSOT.lefthand + hands_wrench_estimates(i).withSOT.righthand;
    
    for j = 1:size(hands_wrench_estimates(i).withSOT.sum, 1)
        hands_wrench_estimates(i).withSOT.force_norm(j) = norm(hands_wrench_estimates(i).withSOT.sum(j,1:3));
    end
   
end


%% Plot hands wrench estimation withoutSOT

fH = figure('units','normalized','outerposition',[0 0 1 1]);
tl = tiledlayout(4,3);


colorSize = 10;
PlotsColorMap = [summer(colorSize); winter(colorSize); autumn(colorSize)];
LineStyles = ["-.", "-.", "-.", "-.x", "-o", ":", "-h"];

ax = [];


for s = 1:3
    
    ax(s) = nexttile;
    for i = 1:size(dataset_dir_names, 2)
    
        plot(hands_wrench_estimates(i).withoutSOT.lefthand(:, s), LineStyles(i),...
            'LineWidth', lineWidth,...
            'Color', L(i,:));
        hold on;
    
    end
    
    axis tight
    ylabel(wrenchLegendString(s), 'Interpreter', 'latex', 'FontSize', fontSize);
    set (gca, 'FontSize' , fontSize)
    
    if s == 6
        xlabel('Samples', 'FontSize', fontSize);
    else
        set(gca, 'XTickLabel', [])
    end
    
    if s == 1
        title("Left Hand", 'FontSize', fontSize, 'fontweight','bold');
    end
    
    nexttile;
    for i = 1:size(dataset_dir_names, 2)
    
        plot(hands_wrench_estimates(i).withoutSOT.righthand(:, s), LineStyles(i),...
            'LineWidth', lineWidth,...
            'Color', L(i,:));
        hold on;
    
    end
    
    axis tight
    set (gca, 'FontSize' , fontSize)
  
    if s == 6
        xlabel('Samples', 'FontSize', fontSize);
    else
        set(gca, 'XTickLabel', [])
    end
    
    
    if s == 1
        title("Right Hand", 'FontSize', fontSize, 'fontweight','bold');
    end
    
    nexttile;
    for i = 1:size(dataset_dir_names, 2)
    
        plot(hands_wrench_estimates(i).withoutSOT.sum(:, s), LineStyles(i),...
            'LineWidth', lineWidth,...
            'Color', L(i,:));
        hold on;
    
    end
    
    axis tight
    set (gca, 'FontSize' , fontSize)
    
    if s == 6
        xlabel('Samples', 'FontSize', fontSize);
    else
        set(gca, 'XTickLabel', [])
    end
    
    
    if s == 1
        title("Hands", 'FontSize', fontSize, 'fontweight','bold');
    end
end

%% Plot force norm
nexttile([1 3])
for i = 1:size(dataset_dir_names, 2)
    
    plot(hands_wrench_estimates(i).withoutSOT.force_norm, LineStyles(i),...
        'LineWidth', lineWidth,...
        'Color', L(i,:));
    hold on;
    
end

title("Hands Estimated Force Norm without NCWE", 'FontSize', fontSize, 'fontweight','bold');

xlabel('Samples', 'FontSize', fontSize);
ylabel('Force Norm $ [N]$', 'Interpreter', 'latex', 'FontSize', fontSize);
set (gca, 'FontSize' , fontSize)
axis tight


txt = title(tl, "Hands Force Estimation without NCWE", 'FontSize', fontSize, 'fontweight','bold');
txt.Interpreter= 'none'; 

lh = legend(ax(1), dataset_dir_names, 'FontSize', legendFontSize,...
           'Location', 'NorthOutside', 'Orientation','Vertical',...
           'NumColumns', size(dataset_dir_names, 2));
lh.Layout.Tile = 'North';
title(lh,'Measurement Covariance')

%% Save figure
save2pdf(strcat(dataset_path + "hands_force_estimation_without_ncwe.pdf"), fH, 600);




% Plot hands wrench estimation withSOT
fH = figure('units','normalized','outerposition',[0 0 1 1]);
tl = tiledlayout(4,3);


colorSize = 10;
PlotsColorMap = [summer(colorSize); winter(colorSize); autumn(colorSize)];

ax = [];


for s = 1:3
    
    ax(s) = nexttile;
    for i = 1:size(dataset_dir_names, 2)
    
        plot(hands_wrench_estimates(i).withSOT.lefthand(:, s), LineStyles(i),...
            'LineWidth', lineWidth,...
            'Color', C(i,:));
        hold on;
    
    end
    
    axis tight
    ylabel(wrenchLegendString(s), 'Interpreter', 'latex', 'FontSize', fontSize);
    set (gca, 'FontSize' , fontSize)
    
    if s == 6
        xlabel('Samples', 'FontSize', fontSize);
    else
        set(gca, 'XTickLabel', [])
    end
    
    if s == 1
        title("Left Hand", 'FontSize', fontSize, 'fontweight','bold');
    end
    
    nexttile;
    for i = 1:size(dataset_dir_names, 2)
    
        plot(hands_wrench_estimates(i).withSOT.righthand(:, s), LineStyles(i),...
            'LineWidth', lineWidth,...
            'Color', C(i,:));
        hold on;
    
    end
    
    axis tight
    set (gca, 'FontSize' , fontSize)
  
    if s == 6
        xlabel('Samples', 'FontSize', fontSize);
    else
        set(gca, 'XTickLabel', [])
    end
    
    
    if s == 1
        title("Right Hand", 'FontSize', fontSize, 'fontweight','bold');
    end
    
    nexttile;
    for i = 1:size(dataset_dir_names, 2)
    
        plot(hands_wrench_estimates(i).withSOT.sum(:, s), LineStyles(i),...
            'LineWidth', lineWidth,...
            'Color', C(i,:));
        hold on;
    
    end
    
    axis tight
    set (gca, 'FontSize' , fontSize)
    
    if s == 6
        xlabel('Samples', 'FontSize', fontSize);
    else
        set(gca, 'XTickLabel', [])
    end
    
    
    if s == 1
        title("Hands", 'FontSize', fontSize, 'fontweight','bold');
    end
end

%% Plot force norm
nexttile([1 3])
for i = 1:size(dataset_dir_names, 2)
    
    plot(hands_wrench_estimates(i).withSOT.force_norm, LineStyles(i),...
        'LineWidth', lineWidth,...
        'Color', C(i,:));
    hold on;
    
end

title("Hands Estimated Force Norm with NCWE", 'FontSize', fontSize, 'fontweight','bold');

xlabel('Samples', 'FontSize', fontSize);
ylabel('Force Norm $ [N]$', 'Interpreter', 'latex', 'FontSize', fontSize);
set (gca, 'FontSize' , fontSize)
axis tight


txt = title(tl, "Hands Force Estimation with NCWE", 'FontSize', fontSize, 'fontweight','bold');
txt.Interpreter= 'none'; 

lh = legend(ax(1), dataset_dir_names, 'FontSize', legendFontSize,...
           'Location', 'NorthOutside', 'Orientation','Vertical',...
           'NumColumns', size(dataset_dir_names, 2));
lh.Layout.Tile = 'North';
title(lh,'Measurement Covariance')

%% Save figure
save2pdf(strcat(dataset_path + "hands_force_estimation_with_ncwe.pdf"), fH, 600);


%% Plot hands estimated force norm and object mass estimation
% % % fH = figure('units','normalized','outerposition',[0 0 1 1]);
% % % tl = tiledlayout(3,1);
% % % 
% % % ax1 = nexttile;
% % % for i = 1:size(dataset_dir_names, 2)
% % %     
% % %     plot(hands_wrench_estimates(i).withoutSOT.force_norm, '-',...
% % %         'LineWidth', lineWidth,...
% % %         'Color', C(i,:));
% % %     hold on;
% % %     
% % % end
% % % 
% % % for i = 1:size(dataset_dir_names, 2)
% % %     
% % %     plot(hands_wrench_estimates(i).withSOT.force_norm, '-.',...
% % %         'LineWidth', lineWidth,...
% % %         'Color', C(i,:));
% % %     hold on;
% % %     
% % % end
% % % 
% % % title("Hands Estimated Force Norm with and without NCWE", 'FontSize', fontSize, 'fontweight','bold');
% % % ylabel('Force Norm $ [N]$', 'Interpreter', 'latex', 'FontSize', fontSize);
% % % set (gca, 'FontSize' , fontSize)
% % % axis tight
% % % ylim([-5 inf])
% % % 
% % % lh = legend(ax1, dataset_dir_names, 'FontSize', legendFontSize,...
% % %            'Location', 'NorthOutside', 'Orientation','Vertical',...
% % %            'NumColumns', size(dataset_dir_names, 2));
% % % lh.Layout.Tile = 'North';
% % % title(lh,'Measurement Covariance without NCWE')
% % % 
% % % 
% % % 
% % % ax2 = nexttile;
% % % yline(objectMass, '--', '9.55 Kg', 'LineWidth', lineWidth, 'FontSize', fontSize);
% % % hold on;
% % % 
% % % for i = 1:size(dataset_dir_names, 2)
% % %     
% % %     plot(hands_wrench_estimates(i).withoutSOT.force_norm/abs(gravity), '-',...
% % %         'LineWidth', lineWidth,...
% % %         'Color', C(i,:));
% % %     hold on;
% % %     
% % % end
% % % 
% % % for i = 1:size(dataset_dir_names, 2)
% % %     
% % %     plot(hands_wrench_estimates(i).withSOT.force_norm/abs(gravity), '-.',...
% % %         'LineWidth', lineWidth,...
% % %         'Color', C(i,:));
% % %     hold on;
% % %     
% % % end
% % % 
% % % title("Estimated Object mass with and without NCWE", 'FontSize', fontSize, 'fontweight','bold');
% % % xlabel('Samples', 'FontSize', fontSize);
% % % ylabel('Object Mass $ [Kg]$', 'Interpreter', 'latex', 'FontSize', fontSize);
% % % set (gca, 'FontSize' , fontSize)
% % % axis tight
% % % ylim([-20 inf])
% % % 
% % % lh = legend(ax2, ['','', '', '', dataset_dir_names], 'FontSize', legendFontSize,...
% % %            'Location', 'NorthOutside', 'Orientation','Vertical',...
% % %            'NumColumns', size(dataset_dir_names, 2));
% % % lh.Layout.Tile = 'North';
% % % title(lh,'Measurement Covariance with NCWE')
% % % 
% % % 
% % % nexttile
% % % yline(objectMass, '--', '5 Kgs', 'LineWidth', lineWidth, 'FontSize', fontSize);
% % % hold on;
% % % 
% % % for i = 1:size(dataset_dir_names, 2)
% % %     
% % %     plot(hands_wrench_estimates(i).withSOT.force_norm/abs(gravity), '-.',...
% % %         'LineWidth', lineWidth,...
% % %         'Color', C(i,:));
% % %     hold on;
% % %     
% % % end
% % % 
% % % title("Estimated Object mass with NCWE", 'FontSize', fontSize, 'fontweight','bold');
% % % xlabel('Samples', 'FontSize', fontSize);
% % % ylabel('Object Mass $ [Kg]$', 'Interpreter', 'latex', 'FontSize', fontSize);
% % % set (gca, 'FontSize' , fontSize)
% % % axis tight
% % % ylim([-5 inf])
% % % 
% % % %% Save figure
% % % save2pdf(strcat(dataset_path + "hands_wrench_estimation_object_mass_estimation.pdf"), fH, 600);
% % % 
% % % 

%% Plot estimated object mass vs measured object mass through feet measurements
fH = figure('units','normalized','outerposition',[0 0 1 1]);
tl = tiledlayout(1,1);

ax = nexttile;

yline(objectMass, '--', '5 Kgs', 'LineWidth', lineWidth, 'FontSize', fontSize);
hold on;
plot(feet_wrench_measurements(1).withoutSOT.force_norm/abs(gravity) - subjectMass, 'LineWidth', lineWidth, 'Color', C(6,:));
hold on;


for i = 1:size(dataset_dir_names, 2)
    
    plot(hands_wrench_estimates(i).withSOT.force_norm/abs(gravity), '-.',...
        'LineWidth', lineWidth,...
        'Color', C(i,:));
    hold on;
    
end

xlabel('Samples', 'FontSize', fontSize);
ylabel("Mass [$kg$]", 'Interpreter', 'latex', 'FontSize', fontSize);
set (gca, 'FontSize' , fontSize)
set (gca, 'ColorOrder' , C)
axis tight

lh = legend(ax, ['Ground Truth (5 Kgs)', 'Using Feet Measurements', dataset_dir_names], 'FontSize', legendFontSize,...
           'Location', 'NorthOutside', 'Orientation','Vertical',...
           'NumColumns', 5);
lh.Layout.Tile = 'North';

txt = title(tl,"Object Mass Value", 'FontSize', fontSize, 'fontweight','bold');
txt.Interpreter= 'none'; 

%% Save figure
save2pdf(strcat(dataset_path + "object_mass_estimation.pdf"), fH, 600);




%% Joint Torque Analysis

joint_names = experiments_dataset(1).withoutSOT.dynamicsJointNames;

joint_torque_estimates = [];

for i = 1:size(dataset_dir_names, 2)
    
    
    for j = 1:size(joint_names, 1)
        joint_torque_estimates(i).withoutSOT.(cell2mat(joint_names(j))).joint_torques = experiments_dataset(i).withoutSOT.data.jointTorques(j, startIndex : end)';
        joint_torque_estimates(i).withSOT.(cell2mat(joint_names(j))).joint_torques = experiments_dataset(i).withSOT.data.jointTorques(j, startIndex : end)';
    end
    
end


fH = figure('units','normalized','outerposition',[0 0 1 1]);
tl = tiledlayout(4,2);

joint_name = "Shoulder";
joint_suffix = ["_rotx", "_roty", "_rotz"];


ax1 = [];
ax2 = [];

for l = 1:size(joint_suffix, 2)
    
    ax1(l) = nexttile;
    
    plot(joint_torque_estimates(1).withoutSOT.(strcat("jLeft",joint_name, joint_suffix(l))).joint_torques, '-',...
             'LineWidth', lineWidth,...
             'Color', L(1,:));
    hold on;
         
    for i = 1:size(dataset_dir_names, 2)
    
        plot(joint_torque_estimates(i).withSOT.(strcat("jLeft",joint_name, joint_suffix(l))).joint_torques, '-.',...
             'LineWidth', lineWidth,...
             'Color', C(i,:));
        hold on;
    
    end
    
    txt = title(strcat("jLeft",joint_name, joint_suffix(l)), 'FontSize', fontSize, 'fontweight','bold');
    txt.Interpreter= 'none'; 
    
    ylabel("$\tau$ [$Nm$]", 'Interpreter', 'latex', 'FontSize', fontSize);
    set (gca, 'FontSize' , fontSize)
    set (gca, 'ColorOrder' , C)
    axis tight
    
    ax2(l) = nexttile;
    
    plot(joint_torque_estimates(1).withoutSOT.(strcat("jRight",joint_name, joint_suffix(l))).joint_torques, '-',...
        'LineWidth', lineWidth,...
        'Color', L(1,:));
    hold on;
    

    for i = 1:size(dataset_dir_names, 2)
    
        plot(joint_torque_estimates(i).withSOT.(strcat("jRight",joint_name, joint_suffix(l))).joint_torques, '-.',...
             'LineWidth', lineWidth,...
             'Color', C(i,:));
        hold on;
    
    end
    
    txt = title(strcat("jRight",joint_name, joint_suffix(l)), 'FontSize', fontSize, 'fontweight','bold');
    txt.Interpreter= 'none'; 

    set (gca, 'FontSize' , fontSize)
    set (gca, 'ColorOrder' , C)
    axis tight
    
    
end



for i = 1:size(dataset_dir_names, 2)
    
    dataSize = size( joint_torque_estimates(i).withSOT.(strcat("jLeft",joint_name, joint_suffix(1))).joint_torques, 1);
    
    for e = 1:dataSize
        
        joint_torque_estimates(i).withSOT.(strcat("jLeft", joint_name)).effort(e) = norm([joint_torque_estimates(i).withSOT.(strcat("jLeft",joint_name, joint_suffix(1))).joint_torques(e),...
                                                                                          joint_torque_estimates(i).withSOT.(strcat("jLeft",joint_name, joint_suffix(2))).joint_torques(e),...
                                                                                          joint_torque_estimates(i).withSOT.(strcat("jLeft",joint_name, joint_suffix(3))).joint_torques(e)]);
        
        joint_torque_estimates(i).withSOT.(strcat("jRight", joint_name)).effort(e) = norm([joint_torque_estimates(i).withSOT.(strcat("jRight",joint_name, joint_suffix(1))).joint_torques(e),...
                                                                                           joint_torque_estimates(i).withSOT.(strcat("jRight",joint_name, joint_suffix(2))).joint_torques(e),...
                                                                                           joint_torque_estimates(i).withSOT.(strcat("jRight",joint_name, joint_suffix(3))).joint_torques(e)]);
        
    end
    
    dataSize = size( joint_torque_estimates(i).withoutSOT.(strcat("jLeft",joint_name, joint_suffix(1))).joint_torques, 1);
    
    for e = 1:dataSize
        
        joint_torque_estimates(i).withoutSOT.(strcat("jLeft", joint_name)).effort(e) = norm([joint_torque_estimates(i).withoutSOT.(strcat("jLeft",joint_name, joint_suffix(1))).joint_torques(e),...
                                                                                             joint_torque_estimates(i).withoutSOT.(strcat("jLeft",joint_name, joint_suffix(2))).joint_torques(e),...
                                                                                             joint_torque_estimates(i).withoutSOT.(strcat("jLeft",joint_name, joint_suffix(3))).joint_torques(e)]);
        
        joint_torque_estimates(i).withoutSOT.(strcat("jRight", joint_name)).effort(e) = norm([joint_torque_estimates(i).withoutSOT.(strcat("jRight",joint_name, joint_suffix(1))).joint_torques(e),...
                                                                                              joint_torque_estimates(i).withoutSOT.(strcat("jRight",joint_name, joint_suffix(2))).joint_torques(e),...
                                                                                              joint_torque_estimates(i).withoutSOT.(strcat("jRight",joint_name, joint_suffix(3))).joint_torques(e)]);
        
    end
end


nexttile

plot(joint_torque_estimates(1).withoutSOT.(strcat("jLeft", joint_name)).effort, '-',...
    'LineWidth', lineWidth,...
    'Color', L(1,:));
hold on;

for i = 1:size(dataset_dir_names, 2)
    plot(joint_torque_estimates(i).withSOT.(strcat("jLeft", joint_name)).effort, '-.',...
        'LineWidth', lineWidth,...
        'Color', C(i,:));
    hold on;
end

title(strcat("jLeft",joint_name, " Effort"), 'FontSize', fontSize, 'fontweight','bold');

xlabel('Samples', 'FontSize', fontSize);
ylabel("$\tau$ [$Nm$]", 'Interpreter', 'latex', 'FontSize', fontSize);
set (gca, 'FontSize' , fontSize)
set (gca, 'ColorOrder' , C)
axis tight


nexttile
plot(joint_torque_estimates(1).withoutSOT.(strcat("jRight", joint_name)).effort, '-',...
    'LineWidth', lineWidth,...
    'Color', L(1,:));
hold on;

for i = 1:size(dataset_dir_names, 2)
    plot(joint_torque_estimates(i).withSOT.(strcat("jRight", joint_name)).effort, '-.',...
        'LineWidth', lineWidth,...
        'Color', C(i,:));
    hold on;
end

title(strcat("jRight",joint_name, " Effort"), 'FontSize', fontSize, 'fontweight','bold');

xlabel('Samples', 'FontSize', fontSize);
set (gca, 'FontSize' , fontSize)
set (gca, 'ColorOrder' , C)
axis tight


% % lh = legend(ax2(1), dataset_dir_names, 'FontSize', legendFontSize,...
% %            'Location', 'NorthOutside', 'Orientation','Vertical',...
% %            'NumColumns', size(dataset_dir_names, 2));
% % lh.Layout.Tile = 'North';
% % title(lh,'Hands Measurement Covariance with NCWE')

lh = legend(ax1(1), dataset_dir_names(1),...
           'FontSize', legendFontSize,...
           'Location', 'NorthOutside', 'Orientation','Vertical',...
           'NumColumns', 5);
lh.Layout.Tile = 'North';
title(lh,'Hands Measurement Covariance without NCWE')

lh = legend(ax2(1), ['', dataset_dir_names(1), dataset_dir_names(2), dataset_dir_names(3)], 'FontSize', legendFontSize,...
           'Location', 'NorthOutside', 'Orientation','Vertical',...
           'NumColumns', size(dataset_dir_names, 2));
lh.Layout.Tile = 'North';
title(lh,'Hands Measurement Covariance with NCWE')


txt = title(tl, strcat(joint_name, " Joint Torque Estimates"), 'FontSize', fontSize, 'fontweight','bold');
txt.Interpreter= 'none'; 




%%% Plot joint torques for other than limbs
fH = figure('units','normalized','outerposition',[0 0 1 1]);

% joint_name_list = ["jC1Head", "jL4L3", "jL5S1", "jT9T8", "jL1T12","jT1C7"];
joint_name_list = ["jLeftShoulder", "jRightShoulder", "jLeftElbow", "jRightElbow", "jLeftWrist", "jRightWrist"];

tl = tiledlayout(4,size(joint_name_list, 2));

joint_suffix = ["_rotx", "_roty", "_rotz"];


ax = [];

for l = 1:size(joint_suffix, 2)
    
    for j = 1:size(joint_name_list,2)
        
        ax(j) = nexttile;
        
        plot(joint_torque_estimates(1).withoutSOT.(strcat(joint_name_list(j), joint_suffix(l))).joint_torques, '-',...
            'LineWidth', lineWidth,...
            'Color', L(1,:));
        hold on;
        
        for i = 1:size(dataset_dir_names, 2)
                        
            plot(joint_torque_estimates(i).withSOT.(strcat(joint_name_list(j), joint_suffix(l))).joint_torques, '-.',...
                'LineWidth', lineWidth,...
                'Color', C(i,:));
            hold on;
            
        end
        
        txt = title(strcat(joint_name_list(j), joint_suffix(l)), 'FontSize', fontSize, 'fontweight','bold');
        txt.Interpreter= 'none';
        
        
        if j == 1
            ylabel("$\tau$ [$Nm$]", 'Interpreter', 'latex', 'FontSize', fontSize);
        end
        set (gca, 'FontSize' , fontSize)
        set (gca, 'ColorOrder' , C)
        axis tight
        
        
    end
    
end


for j = 1:size(joint_name_list,2)

    for i = 1:size(dataset_dir_names, 2)
        
        dataSize = size( joint_torque_estimates(i).withSOT.(strcat(joint_name_list(j), joint_suffix(1))).joint_torques, 1);
        
        for e = 1:dataSize
            
            joint_torque_estimates(i).withSOT.joint_name_list(j).effort(e) = norm([joint_torque_estimates(i).withSOT.(strcat(joint_name_list(j), joint_suffix(1))).joint_torques(e),...
                                                                                   joint_torque_estimates(i).withSOT.(strcat(joint_name_list(j), joint_suffix(2))).joint_torques(e),...
                                                                                   joint_torque_estimates(i).withSOT.(strcat(joint_name_list(j), joint_suffix(3))).joint_torques(e)]);
            
        end
        
        dataSize = size( joint_torque_estimates(i).withoutSOT.(strcat(joint_name_list(j), joint_suffix(1))).joint_torques, 1);
        
        for e = 1:dataSize
            
            joint_torque_estimates(i).withoutSOT.joint_name_list(j).effort(e) = norm([joint_torque_estimates(i).withoutSOT.(strcat(joint_name_list(j), joint_suffix(1))).joint_torques(e),...
                                                                                      joint_torque_estimates(i).withoutSOT.(strcat(joint_name_list(j), joint_suffix(2))).joint_torques(e),...
                                                                                      joint_torque_estimates(i).withoutSOT.(strcat(joint_name_list(j), joint_suffix(3))).joint_torques(e)]);
            
        end
         
    end
    
end

joint_torques_benchmark = [0; 45.57; 0; 20.76; 0; 5.15];

axyline = [];

for j = 1:size(joint_name_list,2)
    
    nexttile
    
    if joint_torques_benchmark(j) ~= 0
        axyline = yline(joint_torques_benchmark(j), '-', num2str(joint_torques_benchmark(j)), 'LineWidth',...
          lineWidth, 'FontSize', fontSize, 'Color', 'k', 'LabelVerticalAlignment', 'bottom', 'FontSize', fontSize);
      hold on;
    end
    
    plot(joint_torque_estimates(1).withoutSOT.joint_name_list(j).effort, '-',...
            'LineWidth', lineWidth,...
            'Color', L(1,:));
        hold on;
        
        
    for i = 1:size(dataset_dir_names, 2)
        
        plot(joint_torque_estimates(i).withSOT.joint_name_list(j).effort, '-.',...
            'LineWidth', lineWidth,...
            'Color', C(i,:));
        hold on;
        
        xlabel('Samples', 'FontSize', fontSize);
        if j == 1
            ylabel("$\tau$ [$Nm$]", 'Interpreter', 'latex', 'FontSize', fontSize);
        end
        set (gca, 'FontSize' , fontSize)
        set (gca, 'ColorOrder' , C)
        axis tight
        
        title(strcat(joint_name_list(j), " Effort"), 'FontSize', fontSize, 'fontweight','bold');
        
    end
    
end

% % lh = legend(ax(1), dataset_dir_names, 'FontSize', legendFontSize,...
% %            'Location', 'NorthOutside', 'Orientation','Vertical',...
% %            'NumColumns', size(dataset_dir_names, 2));
% % lh.Layout.Tile = 'North';
% % title(lh,'Hands Measurement Covariance with NCWE')

lh = legend(ax(1), dataset_dir_names(1),...
           'FontSize', legendFontSize,...
           'Location', 'NorthOutside', 'Orientation','Vertical',...
           'NumColumns', 5);
lh.Layout.Tile = 'North';
title(lh,'Hands Measurement Covariance without NCWE', 'FontSize', fontSize)

lh = legend(ax(2), ['', dataset_dir_names(1), dataset_dir_names(2), dataset_dir_names(3)], 'FontSize', legendFontSize,...
           'Location', 'NorthOutside', 'Orientation','Vertical',...
           'NumColumns', size(dataset_dir_names, 2));
lh.Layout.Tile = 'North';
title(lh,'Hands Measurement Covariance with NCWE', 'FontSize', fontSize)

lh = legend(axyline, 'Benchmark Joint Effort', 'FontSize', legendFontSize,...
           'Location', 'NorthOutside', 'Orientation','Vertical');
lh.Layout.Tile = 'North';

txt = title(tl, "Joint Torque Estimates", 'FontSize', fontSize, 'fontweight','bold');
txt.Interpreter= 'none'; 

%% Save figure
save2pdf(strcat(dataset_path + "joint_torque_estimates_tpose_5kgs.pdf"), fH, 600);