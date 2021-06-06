close all;
clear all;
clc;

N = 5;
C = linspecer(N);
L = lines(N);
lineStyles = linspecer(N,'qualitative');

tposeColor = [0.3333    0.4196    0.1843];

%% Plot parameters
fontSize  = 22;
legendFontSize  = 20;
lineWidth = 4;

gravity = 9.81;
objectMass = 5;

datasettype = 'with';

%% Legend or Title Index
wrenchLegendString = ["$f_x [N]$", "$f_y [N]$", "$f_z [N]$","$m_x [Nm]$", "$m_y [Nm]$", "$m_z [Nm]$"];

dataset_dir = '/home/yeshi/Desktop/non_collocated_wrench_estimation_dataset/';

dataset_type = 'covariance_tuning_dataset';
dataset_source = 'AMTI_Dataset';
subject = 'sub03';



dataset = strcat('tpose_right_5kgs/hde_', datasettype, '_sot');

subjectMass = 0;

if subject == 'sub02'
    
    subjectMass = 79.4;
    
elseif subject == 'sub03'
    
    subjectMass = 75.4;
    
end

dataset_path = strcat(dataset_dir,'/',dataset_type,'/',dataset_source,'/',subject,'/',dataset,'/');

dataset_dir_names = ["1e-4", "1e-3", "1e-2", "1e-1", "1e0"];
% % dataset_dir_names = ["1e0", "1e1", "1e2", "1e4"];


time_data = [];
plot_time = [];

for i = 1:size(dataset_dir_names, 2)
    
    covariance_tuning_dataset(i).Data = load(strcat(dataset_path, '/',  dataset_dir_names(i), '/ ', datasettype, 'SOT.mat'));
    
    %% Get time
    time_data(i).time = covariance_tuning_dataset(i).Data.data.time'/1e9;
    
end


startIndex = 500;

%% Get the shortest sample dataset time
plot_time = time_data(1).time(startIndex:end) - time_data(1).time(startIndex);

endIndex = startIndex + numel(plot_time) - 1;


jointPosition  = covariance_tuning_dataset(1).Data.data.jointPositions(42, startIndex : endIndex);
tposeRange = jointPosition < 0.5 & jointPosition > 0.25;
tposeStartIndex = find(tposeRange, 1, 'first');
tposeEndIndex   = find(tposeRange, 1, 'last');


experiments_dataset = [];

feet_wrench_measurements = [];

hands_wrench_measurements = [];
hands_wrench_estimates = [];


for i = 1:size(dataset_dir_names, 2)
    
    %% Get feet measurements in world
    feet_wrench_measurements(i).Data.leftfoot = covariance_tuning_dataset(i).Data.data.task2_wrenchMeasurementsInWorldFrame(1:6, startIndex : endIndex )';
    feet_wrench_measurements(i).Data.rightfoot = covariance_tuning_dataset(i).Data.data.task2_wrenchMeasurementsInWorldFrame(7:12, startIndex : endIndex )';
    feet_wrench_measurements(i).Data.sum = feet_wrench_measurements(i).Data.leftfoot + feet_wrench_measurements(i).Data.rightfoot;
    
    for j = 1:size(feet_wrench_measurements(i).Data.sum, 1)
        feet_wrench_measurements(i).Data.force_norm(j) = norm(feet_wrench_measurements(i).Data.sum(j,1:3));
    end
    
    
    %% Get hands measurements vs estimates in world
    hands_wrench_measurements(i).Data.lefthand = covariance_tuning_dataset(i).Data.data.task2_wrenchMeasurementsInWorldFrame(13:18, startIndex : endIndex )';
    hands_wrench_measurements(i).Data.righthand = covariance_tuning_dataset(i).Data.data.task2_wrenchMeasurementsInWorldFrame(19:24, startIndex : endIndex )';
    hands_wrench_measurements(i).Data.sum = hands_wrench_measurements(i).Data.lefthand + hands_wrench_measurements(i).Data.righthand;
    
    for j = 1:size(hands_wrench_measurements(i).Data.sum, 1)
        hands_wrench_measurements(i).Data.force_norm(j) = norm(hands_wrench_measurements(i).Data.sum(j,1:3));
    end
    
    
    hands_wrench_estimates(i).Data.lefthand = covariance_tuning_dataset(i).Data.data.task2_wrenchEstimatesInWorldFrame(13:18, startIndex : endIndex )';
    hands_wrench_estimates(i).Data.righthand = covariance_tuning_dataset(i).Data.data.task2_wrenchEstimatesInWorldFrame(19:24, startIndex : endIndex )';
    hands_wrench_estimates(i).Data.sum = hands_wrench_estimates(i).Data.lefthand + hands_wrench_estimates(i).Data.righthand;
    
    for j = 1:size(hands_wrench_estimates(i).Data.sum, 1)
        hands_wrench_estimates(i).Data.force_norm(j) = norm(hands_wrench_estimates(i).Data.sum(j,1:3));
    end
    
end


%% Plot hands wrench estimation withoutSOT
fH = figure('units','normalized','outerposition',[0 0 1 1]);
tl = tiledlayout(6,3);


LineStyles = ["-", "-", "-", "-", "-", "-", "-"];

ax = [];


for s = 1:6
    
    ax(s) = nexttile;
    for i = 1:size(dataset_dir_names, 2)
        
        plot(plot_time, hands_wrench_estimates(i).Data.lefthand(:, s), LineStyles(i),...
            'LineWidth', lineWidth,...
            'Color', C(i,:));
        hold on;
        
    end
    
    xline(plot_time(tposeStartIndex), '-.','Tpose Start', 'LineWidth', 3, 'FontSize', 14,...
        'LabelVerticalAlignment', 'bottom', 'Color', tposeColor);
    
    xline(plot_time(tposeEndIndex), '-.','Tpose End', 'LineWidth', 3, 'FontSize', 14,...
        'LabelVerticalAlignment', 'bottom',  'Color', tposeColor);
    
    axis tight
    ylabel(wrenchLegendString(s), 'Interpreter', 'latex', 'FontSize', fontSize);
    set (gca, 'FontSize' , fontSize)
    
    if s == 6
        xlabel('Time [S]', 'FontSize', fontSize, 'Interpreter', 'latex');
    else
        set(gca, 'XTickLabel', [])
    end
    
    if s == 1
        title("Left Hand with Covariance 1e-6", 'FontSize', fontSize, 'fontweight','bold');
    end
    
    nexttile;
    for i = 1:size(dataset_dir_names, 2)
        
        plot(plot_time, hands_wrench_estimates(i).Data.righthand(:, s), LineStyles(i),...
            'LineWidth', lineWidth,...
            'Color', C(i,:));
        hold on;
        
    end
    
    xline(plot_time(tposeStartIndex), '-.','Tpose Start', 'LineWidth', 3, 'FontSize', 14,...
        'LabelVerticalAlignment', 'bottom', 'Color', tposeColor);
    
    xline(plot_time(tposeEndIndex), '-.','Tpose End', 'LineWidth', 3, 'FontSize', 14,...
        'LabelVerticalAlignment', 'bottom',  'Color', tposeColor);
    
    axis tight
    set (gca, 'FontSize' , fontSize)
    
    if s == 6
        xlabel('Time [S]', 'FontSize', fontSize, 'Interpreter', 'latex');
    else
        set(gca, 'XTickLabel', [])
    end
    
    
    if s == 1
        title("Right Hand", 'FontSize', fontSize, 'fontweight','bold');
    end
    
    ax1 = nexttile;
    for i = 1:size(dataset_dir_names, 2)
        
        plot(plot_time, hands_wrench_estimates(i).Data.sum(:, s), LineStyles(i),...
            'LineWidth', lineWidth,...
            'Color', C(i,:));
        hold on;
        
    end
    
    xline(plot_time(tposeStartIndex), '-.','Tpose Start', 'LineWidth', 3, 'FontSize', 14,...
        'LabelVerticalAlignment', 'middle', 'Color', tposeColor);
    
    xline(plot_time(tposeEndIndex), '-.','Tpose End', 'LineWidth', 3, 'FontSize', 14,...
        'LabelVerticalAlignment', 'middle',  'Color', tposeColor);
    
    axis tight
    set (gca, 'FontSize' , fontSize)
    
    if s == 6
        xlabel('Time [S]', 'FontSize', fontSize, 'Interpreter', 'latex');
    else
        set(gca, 'XTickLabel', [])
    end
    
    
    if s == 1
        title("Hands", 'FontSize', fontSize, 'fontweight','bold');
    end
end


lh = legend(ax1, dataset_dir_names, 'FontSize', legendFontSize,...
    'Location', 'NorthOutside', 'Orientation','Vertical',...
    'NumColumns', 5);
lh.Layout.Tile = 'North';
title(lh,'Right Hand Measurement Covariance')

txt = title(tl, strcat("Hands Wrench Estimation ", datasettype, " NCWE"), 'FontSize', fontSize, 'fontweight','bold');
txt.Interpreter= 'none';

%% Save figure
save2pdf(strcat(dataset_path + "hands_force_estimation_", datasettype, "_ncwe.pdf"), fH, 600);


%% Plot estimated object mass
fH = figure('units','normalized','outerposition',[0 0 1 1]);
tl = tiledlayout(1,1);

ax = nexttile;

yline(objectMass, '--', '5 Kgs' ,'LineWidth', lineWidth, 'FontSize', fontSize,...
    'LabelVerticalAlignment', 'bottom');
hold on;

for i = 1:size(dataset_dir_names, 2)
    
    plot(plot_time, hands_wrench_estimates(i).Data.force_norm/abs(gravity), '-.',...
        'LineWidth', 2 * lineWidth,...
        'Color', C(i,:));
    hold on;
        
end

xline(plot_time(tposeStartIndex), '-.','Tpose Start', 'LineWidth', 5, 'FontSize', 20,...
    'LabelVerticalAlignment', 'middle', 'Color', tposeColor);

xline(plot_time(tposeEndIndex), '-.','Tpose End', 'LineWidth', 5, 'FontSize', 20,...
    'LabelVerticalAlignment', 'middle',  'Color', tposeColor);

xlabel('Time [S]', 'FontSize', 2 * fontSize, 'Interpreter', 'latex');
ylabel("Mass [$kg$]", 'Interpreter', 'latex', 'FontSize', 2 * fontSize);


lh = legend(ax, ['Ground Truth (5 Kg)' dataset_dir_names], 'FontSize', 2 * legendFontSize,...
    'Location', 'NorthOutside', 'Orientation','Vertical',...
    'NumColumns', 10);
lh.Layout.Tile = 'North';

txt = title(tl, strcat("Estimated Object Mass Value ", datasettype, " NCWE"), 'FontSize', 2 * fontSize, 'fontweight','bold');
txt.Interpreter= 'none';

axis tight
set (gca, 'FontSize' , 2 * fontSize)    

%% Save figure
save2pdf(strcat(dataset_path + "object_mass_estimation_", datasettype, "_ncwe.pdf"), fH, 600);
