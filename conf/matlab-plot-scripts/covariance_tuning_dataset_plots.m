close all;
clear all;
clc;


%% Plot parameters
fontSize  = 25;
legendFontSize  = 20;
lineWidth = 2.5;

gravity = 9.81;
subjectMass = 75.4;

%% Legend or Title Index
wrenchLegendString = ["$f_x [N]$", "$f_y [N]$", "$f_z [N]$","$m_x [Nm]$", "$m_y [Nm]$", "$m_z [Nm]$"];

dataset_dir = '/home/yeshi/Desktop/non_collocated_wrench_estimation_dataset/';
dataset_type = 'covariance_tuning_dataset';
dataset_source = 'FTShoes_Dataset';
subject = 'sub03';
dataset = 'npose/hde';

dataset_path = strcat(dataset_dir,'/',dataset_type,'/',dataset_source,'/',subject,'/',dataset); 

%% Load data for the feet force/torque measurement covariance from 1e-6 to 1e6

% % dataset_dir_names = ["1e-6", "1e-4", "1e-2", "1e0", "1e+2", "1e4", "1e6"];
dataset_dir_names = ["1e-4", "1e0", "1e4"];

covariance_tuning_dataset = [];

feet_wrench_measurements = [];
feet_wrench_estimates = [];

feet_wrench_error = [];

for i = 1:size(dataset_dir_names, 2)
    
    covariance_tuning_dataset(i).withoutSOT = load(strcat(dataset_path, '/',  dataset_dir_names(i),'/withoutSOT.mat'));
    covariance_tuning_dataset(i).withSOT = load(strcat(dataset_path, '/',  dataset_dir_names(i), '/withSOT.mat'));
    
    
    %% Get feet measurements vs estimates in world
    feet_wrench_measurements(i).withoutSOT.leftfoot = covariance_tuning_dataset(i).withoutSOT.data.task2_wrenchMeasurementsInWorldFrame(1:6, : )'; 
    feet_wrench_measurements(i).withoutSOT.rightfoot = covariance_tuning_dataset(i).withoutSOT.data.task2_wrenchMeasurementsInWorldFrame(7:12, : )';
    feet_wrench_measurements(i).withoutSOT.sum = feet_wrench_measurements(i).withoutSOT.leftfoot + feet_wrench_measurements(i).withoutSOT.rightfoot;
    
    for j = 1:size(feet_wrench_measurements(i).withoutSOT.sum, 1)
        feet_wrench_measurements(i).withoutSOT.force_norm(j) = norm(feet_wrench_measurements(i).withoutSOT.sum(j,1:3));
    end
    
    feet_wrench_estimates(i).withoutSOT.leftfoot = covariance_tuning_dataset(i).withoutSOT.data.task2_wrenchEstimatesInWorldFrame(1:6, : )'; 
    feet_wrench_estimates(i).withoutSOT.rightfoot = covariance_tuning_dataset(i).withoutSOT.data.task2_wrenchEstimatesInWorldFrame(7:12, : )'; 
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
fH = figure('units','normalized','outerposition',[0 0 1 1]);
tl = tiledlayout(2,3);

colorSize = 10; % size(dataset_dir_names, 2);
% PlotsColorMap = [summer(colorSize);...
%                  winter(colorSize);...
%                  spring(colorSize);...
%                  autumn(colorSize);...
%                  parula(colorSize);...
%                  copper(colorSize);...
%                  bone(colorSize)];

PlotsColorMap = [summer(colorSize); winter(colorSize); autumn(colorSize)];
             
LineStyles = ["-.", "-", "--", "-.x", "-o", ":", "-h"];

ax = [];


for s = 1:6
    
    ax(s) = nexttile;
    
    for i = 1:size(dataset_dir_names, 2)
        plot(feet_wrench_error(i).withoutSOT.sum_error(:, s), LineStyles(i),...
             'LineWidth', lineWidth,...
             'Color',  PlotsColorMap((i -1 ) * colorSize + 1,:));
        hold on;
    end
    
    xlabel('Samples', 'FontSize', fontSize);
    ylabel(wrenchLegendString(s), 'Interpreter', 'latex', 'FontSize', fontSize);
    set (gca, 'FontSize' , fontSize)
    axis tight
    
end


lh = legend(ax(1), dataset_dir_names, 'FontSize', legendFontSize,...
           'Location', 'NorthOutside', 'Orientation','Horizontal',...
           'NumColumns', size(dataset_dir_names, 2));
% lh.Layout.Tile = 'North';     

txt = title(tl, "Sum of Feet Wrench Error", 'FontSize', fontSize, 'fontweight','bold');
txt.Interpreter= 'none'; 


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
    
        plot(feet_wrench_error(i).withoutSOT.leftfoot_error(:, s), LineStyles(i),...
            'LineWidth', lineWidth,...
            'Color',  PlotsColorMap((i -1 ) * colorSize + 1,:));
        hold on;
    
    end
    
    ylabel(wrenchLegendString(s), 'Interpreter', 'latex', 'FontSize', fontSize);
    set (gca, 'FontSize' , fontSize)
    
    if s == 6
        xlabel('Samples', 'FontSize', fontSize);
    else
        set(gca, 'XTickLabel', [])
    end
    
    if s == 1
        title("Left Foot", 'FontSize', fontSize, 'fontweight','bold');
    end
    
    nexttile;
    for i = 1:size(dataset_dir_names, 2)
    
        plot(feet_wrench_error(i).withoutSOT.rightfoot_error(:, s), LineStyles(i),...
            'LineWidth', lineWidth,...
            'Color',  PlotsColorMap((i -1 ) * colorSize + 1,:));
        hold on;
    
    end
    
    set (gca, 'FontSize' , fontSize)
  
    if s == 6
        xlabel('Samples', 'FontSize', fontSize);
    else
        set(gca, 'XTickLabel', [])
    end
    
    
    if s == 1
        title("Right Foot", 'FontSize', fontSize, 'fontweight','bold');
    end
    
    nexttile;
    for i = 1:size(dataset_dir_names, 2)
    
        plot(feet_wrench_error(i).withoutSOT.sum_error(:, s), LineStyles(i),...
            'LineWidth', lineWidth,...
            'Color',  PlotsColorMap((i -1 ) * colorSize + 1,:));
        hold on;
    
    end
    
    set (gca, 'FontSize' , fontSize)
    
    if s == 6
        xlabel('Samples', 'FontSize', fontSize);
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
           'Location', 'NorthOutside', 'Orientation','Horizontal',...
           'NumColumns', size(dataset_dir_names, 2));

%% Save figure
save2pdf(strcat(dataset_path + "feet_wrench_error.pdf"), fH, 600);