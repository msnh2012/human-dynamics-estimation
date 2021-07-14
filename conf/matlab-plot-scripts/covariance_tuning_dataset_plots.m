close all;
clear all;
clc;

N=8;
% C = linspecer(N);
C = lines(N);
lineStyles = linspecer(N,'qualitative');

tposeColor = [0.3333    0.4196    0.1843];

%% Plot parameters
fontSize  = 30;
legendFontSize  = 30;
lineWidth = 4;

gravity = 9.81;
subjectMass = 75.4;

%% Legend or Title Index
wrenchLegendString = ["$f_x [N]$", "$f_y [N]$", "$f_z [N]$","$m_x [Nm]$", "$m_y [Nm]$", "$m_z [Nm]$"];

dataset_dir = '/home/yeshi/Desktop/non_collocated_wrench_estimation_dataset/';
dataset_type = 'covariance_tuning_dataset';
dataset_source = 'AMTI_Dataset';
subject = 'sub03';
dataset = 'tpose/hde/feet';

dataset_path = strcat(dataset_dir,'/',dataset_type,'/',dataset_source,'/',subject,'/',dataset, '/');

%% Load data for the feet force/torque measurement covariance from 1e-6 to 1e6

% % dataset_dir_names = ["1e-6", "1e-4", "1e-2", "1e0", "1e+2", "1e4", "1e6"];
dataset_dir_names = ["1e-6", "1e-3", "1e-2"];

covariance_tuning_dataset = [];

feet_wrench_measurements = [];
feet_wrench_estimates = [];

feet_wrench_error = [];

time_data = [];
plot_time = [];

for i = 1:size(dataset_dir_names, 2)
    
    covariance_tuning_dataset(i).withoutSOT = load(strcat(dataset_path, '/',  dataset_dir_names(i),'/withoutSOT.mat'));
    
    %% Get time
    time_data(i).time = covariance_tuning_dataset(i).withoutSOT.data.time'/1e9;
    
end


startIndex = 100;

%% Get the shortest sample dataset time
plot_time = time_data(3).time(startIndex:end) - time_data(3).time(startIndex);

endIndex = startIndex + numel(plot_time) - 1;

jointPosition  = covariance_tuning_dataset(3).withoutSOT.data.jointPositions(42, startIndex : endIndex);
tposeRange = jointPosition < 0.15 & jointPosition > 0.12;
tposeStartIndex = find(tposeRange, 1, 'first');
tposeEndIndex   = find(tposeRange, 1, 'last');


for i = 1:size(dataset_dir_names, 2)
    
    
    
    %% Get feet measurements vs estimates in world
    feet_wrench_measurements(i).withoutSOT.leftfoot = covariance_tuning_dataset(i).withoutSOT.data.task2_wrenchMeasurementsInWorldFrame(1:6, startIndex: endIndex)';
    feet_wrench_measurements(i).withoutSOT.rightfoot = covariance_tuning_dataset(i).withoutSOT.data.task2_wrenchMeasurementsInWorldFrame(7:12, startIndex: endIndex)';
    feet_wrench_measurements(i).withoutSOT.sum = feet_wrench_measurements(i).withoutSOT.leftfoot + feet_wrench_measurements(i).withoutSOT.rightfoot;
    
    for j = 1:size(feet_wrench_measurements(i).withoutSOT.sum, 1)
        feet_wrench_measurements(i).withoutSOT.force_norm(j) = norm(feet_wrench_measurements(i).withoutSOT.sum(j,1:3));
    end
    
    feet_wrench_estimates(i).withoutSOT.leftfoot = covariance_tuning_dataset(i).withoutSOT.data.task2_wrenchEstimatesInWorldFrame(1:6, startIndex: endIndex)';
    feet_wrench_estimates(i).withoutSOT.rightfoot = covariance_tuning_dataset(i).withoutSOT.data.task2_wrenchEstimatesInWorldFrame(7:12, startIndex: endIndex)';
    feet_wrench_estimates(i).withoutSOT.sum = feet_wrench_estimates(i).withoutSOT.leftfoot + feet_wrench_estimates(i).withoutSOT.rightfoot;
    
    
    for j = 1:size(feet_wrench_estimates(i).withoutSOT.sum, 1)
        feet_wrench_estimates(i).withoutSOT.force_norm(j) = norm(feet_wrench_estimates(i).withoutSOT.sum(j,1:3));
    end
    
    feet_wrench_error(i).withoutSOT.sum_error = abs(feet_wrench_measurements(i).withoutSOT.sum - feet_wrench_estimates(i).withoutSOT.sum) ;
    feet_wrench_error(i).withoutSOT.force_norm_error = abs(feet_wrench_measurements(i).withoutSOT.force_norm - feet_wrench_estimates(i).withoutSOT.force_norm);
    
    feet_wrench_error(i).withoutSOT.leftfoot_error = abs(feet_wrench_measurements(i).withoutSOT.leftfoot - feet_wrench_estimates(i).withoutSOT.leftfoot) ;
    feet_wrench_error(i).withoutSOT.rightfoot_error = abs(feet_wrench_measurements(i).withoutSOT.rightfoot - feet_wrench_estimates(i).withoutSOT.rightfoot) ;
    
end


%% Plot Sum of Feet Wrench Error
% % fH = figure('units','normalized','outerposition',[0 0 1 1]);
% % tl = tiledlayout(2,3);
% %
colorSize = 10;
% % PlotsColorMap = [summer(colorSize); winter(colorSize); autumn(colorSize)];
% %
LineStyles = ["-", "-", "-", "-.x", "-o", ":", "-h"];
% %
% % ax = [];
% %
% %
% % for s = 1:6
% %
% %     ax(s) = nexttile;
% %
% %     for i = 1:size(dataset_dir_names, 2)
% %         plot(feet_wrench_error(i).withoutSOT.sum_error(:, s), LineStyles(i),...
% %              'LineWidth', lineWidth,...
% %              'Color',  C(i,:));
% %         hold on;
% %     end
% %
% %
% %     xlabel('Samples', 'FontSize', fontSize);
% %     ylabel(wrenchLegendString(s), 'Interpreter', 'latex', 'FontSize', fontSize);
% %     set (gca, 'FontSize' , fontSize)
% %     axis tight
% %     ylim([-3 inf])
% %
% % end
% %
% %
% % lh = legend(ax(1), dataset_dir_names, 'FontSize', legendFontSize,...
% %            'Location', 'NorthOutside', 'Orientation','Vertical',...
% %            'NumColumns', size(dataset_dir_names, 2));
% % lh.Layout.Tile = 'North';
% % title(lh,'Measurement Covariance')
% %
% % txt = title(tl, "Sum of Feet Wrench Error", 'FontSize', fontSize, 'fontweight','bold');
% % txt.Interpreter= 'none';


% % %% Plot norm of feet force measurements vs estimates
% % fH = figure('units','normalized','outerposition',[0 0 1 1]);
% %
% %
% % for i = 1:size(dataset_dir_names, 2)
% %     plot(feet_wrench_error(i).withoutSOT.force_norm_error, LineStyles(i),...
% %          'LineWidth', lineWidth,...
% %          'Color',  PlotsColorMap((i -1 ) * colorSize + 1,:))
% %     hold on;
% % end
% %
% % xlabel('Samples', 'FontSize', fontSize);
% % ylabel('Force Norm $ [N]$', 'Interpreter', 'latex', 'FontSize', fontSize);
% % set (gca, 'FontSize' , fontSize)
% % axis tight
% %
% % title("Error of Sum of Forces Norm", 'FontSize', fontSize, 'fontweight','bold');
% % txt.Interpreter= 'none';
% %


%% Plot feet wrench error

fH = figure('units','normalized','outerposition',[0 0 1 1]);
tl = tiledlayout(6,3);


PlotsColorMap = [summer(colorSize); winter(colorSize); autumn(colorSize)];

ax = [];


for s = 1:6
    
    ax(s) = nexttile;
    for i = 1:size(dataset_dir_names, 2)
        
        plot(plot_time, feet_wrench_error(i).withoutSOT.leftfoot_error(:, s), LineStyles(i),...
            'LineWidth', lineWidth,...
            'Color',  C(i,:));
        hold on;
        
    end
    
    xline(plot_time(tposeStartIndex), '-.','Tpose Start', 'LineWidth', 3, 'FontSize', 14,...
        'LabelVerticalAlignment', 'middle', 'Color', tposeColor);
    
    xline(plot_time(tposeEndIndex), '-.','Tpose End', 'LineWidth', 3, 'FontSize', 14,...
        'LabelVerticalAlignment', 'middle',  'Color', tposeColor);
    
    axis tight
    ylim([-3 inf])
    ylabel(wrenchLegendString(s), 'Interpreter', 'latex', 'FontSize', fontSize);
    set (gca, 'FontSize' , fontSize)
    
    if s == 6
        xlabel('Time [S]', 'FontSize', fontSize, 'Interpreter', 'latex');
    else
        set(gca, 'XTickLabel', [])
    end
    
    if s == 1
        title("Left Foot", 'FontSize', fontSize, 'fontweight','bold');
    end
    
    nexttile;
    for i = 1:size(dataset_dir_names, 2)
        
        plot(plot_time, feet_wrench_error(i).withoutSOT.rightfoot_error(:, s), LineStyles(i),...
            'LineWidth', lineWidth,...
            'Color',  C(i,:));
        hold on;
        
    end
    
    xline(plot_time(tposeStartIndex), '-.','Tpose Start', 'LineWidth', 3, 'FontSize', 14,...
        'LabelVerticalAlignment', 'middle', 'Color', tposeColor);
    
    xline(plot_time(tposeEndIndex), '-.','Tpose End', 'LineWidth', 3, 'FontSize', 14,...
        'LabelVerticalAlignment', 'middle',  'Color', tposeColor);
    
    axis tight
    ylim([-3 inf])
    set (gca, 'FontSize' , fontSize)
    
    if s == 6
        xlabel('Time [S]', 'FontSize', fontSize, 'Interpreter', 'latex');
    else
        set(gca, 'XTickLabel', [])
    end
    
    
    if s == 1
        title("Right Foot", 'FontSize', fontSize, 'fontweight','bold');
    end
    
    nexttile;
    for i = 1:size(dataset_dir_names, 2)
        
        plot(plot_time, feet_wrench_error(i).withoutSOT.sum_error(:, s), LineStyles(i),...
            'LineWidth', lineWidth,...
            'Color',  C(i,:));
        hold on;
        
    end
    
    xline(plot_time(tposeStartIndex), '-.','Tpose Start', 'LineWidth', 3, 'FontSize', 14,...
        'LabelVerticalAlignment', 'middle', 'Color', tposeColor);
    
    xline(plot_time(tposeEndIndex), '-.','Tpose End', 'LineWidth', 3, 'FontSize', 14,...
        'LabelVerticalAlignment', 'middle',  'Color', tposeColor);
    
    axis tight
    ylim([-3 inf])
    set (gca, 'FontSize' , fontSize)
    
    if s == 6
        xlabel('Time [S]', 'FontSize', fontSize, 'Interpreter', 'latex');
    else
        set(gca, 'XTickLabel', [])
    end
    
    
    if s == 1
        title("Feet", 'FontSize', fontSize, 'fontweight','bold');
    end
end



txt = title(tl, "Feet Wrench Error", 'FontSize', fontSize, 'fontweight','bold');
txt.Interpreter= 'none';

lh = legend(ax(1), dataset_dir_names, 'FontSize', legendFontSize,...
    'Location', 'NorthOutside', 'Orientation','Vertical',...
    'NumColumns', size(dataset_dir_names, 2));
lh.Layout.Tile = 'North';
title(lh,'Measurement Covariance')


%% Save figure
save2pdf((strcat(dataset_path + "feet_wrench_error.pdf")), fH, 600);


%% Plot wrench force measurements vs estimates for 1e-4
fH = figure('units','normalized','outerposition',[0 0 1 1]);
tl = tiledlayout(6,3);


PlotsColorMap = [autumn(colorSize); summer(colorSize)];

measurementColorIndex = 1;
estimationColorIndex = 15;

covarianceDataIndex = 1;

ax = [];

for s = 1:6
    
    ax(s) = nexttile;
    plot(plot_time, feet_wrench_measurements(covarianceDataIndex).withoutSOT.leftfoot(:, s),'-.',...
        'LineWidth', lineWidth, 'Color', PlotsColorMap(measurementColorIndex,:));
    hold on;
    plot(plot_time, feet_wrench_estimates(covarianceDataIndex).withoutSOT.leftfoot(:, s), '-.',...
        'LineWidth', lineWidth, 'Color', PlotsColorMap(estimationColorIndex,:));
    hold on;
    
    axis tight
    ylabel(wrenchLegendString(s), 'Interpreter', 'latex', 'FontSize', fontSize);
    set (gca, 'FontSize' , fontSize)
    
    if s == 6
        xlabel('Time [S]', 'FontSize', fontSize, 'Interpreter', 'latex');
    else
        set(gca, 'XTickLabel', [])
    end
    
    if s == 1
        title("Left Foot", 'FontSize', fontSize, 'fontweight','bold');
    end
    
    nexttile;
    plot(plot_time, feet_wrench_measurements(covarianceDataIndex).withoutSOT.rightfoot(:, s),'-.',...
        'LineWidth', lineWidth, 'Color', PlotsColorMap(measurementColorIndex,:));
    hold on;
    plot(plot_time, feet_wrench_estimates(covarianceDataIndex).withoutSOT.rightfoot(:, s), '-.',...
        'LineWidth', lineWidth, 'Color', PlotsColorMap(estimationColorIndex,:));
    hold on;
    
    axis tight
    set (gca, 'FontSize' , fontSize)
    
    if s == 6
        xlabel('Time [S]', 'FontSize', fontSize, 'Interpreter', 'latex');
    else
        set(gca, 'XTickLabel', [])
    end
    
    
    if s == 1
        title("Right Foot", 'FontSize', fontSize, 'fontweight','bold');
    end
    
    nexttile;
    plot(plot_time, feet_wrench_measurements(covarianceDataIndex).withoutSOT.sum(:, s),'-.',...
        'LineWidth', lineWidth, 'Color', PlotsColorMap(measurementColorIndex,:));
    hold on;
    plot(plot_time, feet_wrench_estimates(covarianceDataIndex).withoutSOT.sum(:, s), '-.',...
        'LineWidth', lineWidth, 'Color', PlotsColorMap(estimationColorIndex,:));
    hold on;
    
    axis tight
    set (gca, 'FontSize' , fontSize)
    
    if s == 6
        xlabel('Time [S]', 'FontSize', fontSize, 'Interpreter', 'latex');
    else
        set(gca, 'XTickLabel', [])
    end
    
    
    if s == 1
        title("Feet", 'FontSize', fontSize, 'fontweight','bold');
    end
end

txt = title(tl, strcat("Feet Wrench Measurements and Estimates for Covariance ", dataset_dir_names(covarianceDataIndex)),...
    'FontSize', fontSize, 'fontweight','bold');
txt.Interpreter= 'none';

lh = legend(ax(1), 'Measurements', 'Estimates', 'FontSize', legendFontSize,...
    'Location', 'NorthOutside', 'Orientation','Vertical',...
    'NumColumns', 2);
lh.Layout.Tile = 'North';

%% Save figure
save2pdf((strcat(dataset_path + "feet_wrench_tracking.pdf")), fH, 600);
