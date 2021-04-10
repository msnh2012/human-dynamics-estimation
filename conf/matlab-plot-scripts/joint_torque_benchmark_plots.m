close all;
clear all;
clc;

N=8;
C = linspecer(N);
lineStyles = linspecer(N,'qualitative');

%% Plot parameters
fontSize  = 25;
legendFontSize  = 30;
lineWidth = 3;

gravity = 9.81;
objectMass = 9.55;

%% Legend or Title Index
wrenchLegendString = ["$f_x [N]$", "$f_y [N]$", "$f_z [N]$","$m_x [Nm]$", "$m_y [Nm]$", "$m_z [Nm]$"];

dataset_dir = '/home/yeshi/Desktop/non_collocated_wrench_estimation_dataset/';
dataset_type = 'covariance_tuning_dataset';
dataset_source = 'AMTI_Dataset';
subject = 'sub03';
dataset = 'tpose/hde/feet';

dataset_path = strcat(dataset_dir,'/',dataset_type,'/',dataset_source,'/',subject,'/',dataset, '/');

Data =  load(strcat(dataset_path,'1e-4/withoutSOT.mat'));

joint_names = Data.dynamicsJointNames;

joint_torque_estimates = [];

startIndex = 50;

for j = 1:size(joint_names, 1)
    joint_torque_estimates.(cell2mat(joint_names(j))).joint_torques = Data.data.jointTorques(j, startIndex:end)';
end


fH = figure('units','normalized','outerposition',[0 0 1 1]);

joint_names_list = ["Shoulder", "Elbow", "Wrist"];
joint_suffix = ["_rotx", "_roty", "_rotz"];

tl = tiledlayout(4, 2 * size(joint_names_list, 2));


for l = 1:size(joint_suffix, 2)
    
    for j = 1:size(joint_names_list, 2)
        
        
        nexttile;
        
        plot(joint_torque_estimates.(strcat("jLeft",joint_names_list(j), joint_suffix(l))).joint_torques, '-',...
            'LineWidth', lineWidth,...
            'Color', C(l,:));
        hold on;
        
        txt = title(strcat("jLeft",joint_names_list(j), joint_suffix(l)), 'FontSize', fontSize, 'fontweight','bold');
        txt.Interpreter= 'none';
        
        if j == 1
            ylabel("$\tau$ [$Nm$]", 'Interpreter', 'latex', 'FontSize', fontSize);
        end
        set (gca, 'FontSize' , fontSize)
        set (gca, 'ColorOrder' , C)
        axis tight
        
        nexttile;
        
        plot(joint_torque_estimates.(strcat("jRight",joint_names_list(j), joint_suffix(l))).joint_torques, '-',...
            'LineWidth', lineWidth,...
            'Color', C(l,:));
        hold on;
        
        
        txt = title(strcat("jRight",joint_names_list(j), joint_suffix(l)), 'FontSize', fontSize, 'fontweight','bold');
        txt.Interpreter= 'none';
        
        
        set (gca, 'FontSize' , fontSize)
        set (gca, 'ColorOrder' , C)
        axis tight
        
        
    end
    
end

joint_torques_benchmark = [12.94; 3.46; 0.43];

for j = 1:size(joint_names_list, 2)
    
    dataSize = size( joint_torque_estimates.(strcat("jLeft",joint_names_list(j), joint_suffix(1))).joint_torques, 1);
    
    for e = 1:dataSize
        
        joint_torque_estimates.(strcat("jLeft", joint_names_list(j))).effort(e) = norm([joint_torque_estimates.(strcat("jLeft",joint_names_list(j), joint_suffix(1))).joint_torques(e),...
            joint_torque_estimates.(strcat("jLeft",joint_names_list(j), joint_suffix(2))).joint_torques(e),...
            joint_torque_estimates.(strcat("jLeft",joint_names_list(j), joint_suffix(3))).joint_torques(e)]);
        
        joint_torque_estimates.(strcat("jRight", joint_names_list(j))).effort(e) = norm([joint_torque_estimates.(strcat("jRight",joint_names_list(j), joint_suffix(1))).joint_torques(e),...
            joint_torque_estimates.(strcat("jRight",joint_names_list(j), joint_suffix(2))).joint_torques(e),...
            joint_torque_estimates.(strcat("jRight",joint_names_list(j), joint_suffix(3))).joint_torques(e)]);
        
    end
    
    nexttile
    
    ax = yline(joint_torques_benchmark(j), '-', num2str(joint_torques_benchmark(j)), 'LineWidth',...
          lineWidth, 'FontSize', fontSize, 'Color', 'k', 'LabelVerticalAlignment', 'bottom', 'FontSize', fontSize);
    hold on;
    plot(joint_torque_estimates.(strcat("jLeft", joint_names_list(j))).effort, '-',...
        'LineWidth', lineWidth,...
        'Color', C(6,:));
    hold on;
    
    
    title(strcat("jLeft",joint_names_list(j), " Effort"), 'FontSize', fontSize, 'fontweight','bold');
    
    xlabel('Samples', 'FontSize', fontSize);
    if j == 1
        ylabel("$\tau$ [$Nm$]", 'Interpreter', 'latex', 'FontSize', fontSize);
    end
    set (gca, 'FontSize' , fontSize)
    set (gca, 'ColorOrder' , C)
    axis tight
    
    
    nexttile
    yline(joint_torques_benchmark(j), '-', num2str(joint_torques_benchmark(j)), 'LineWidth',...
          lineWidth, 'FontSize', fontSize, 'Color', 'k', 'LabelVerticalAlignment', 'bottom', 'FontSize', fontSize);
    hold on;
    plot(joint_torque_estimates.(strcat("jRight", joint_names_list(j))).effort, '-',...
        'LineWidth', lineWidth,...
        'Color', C(6,:));
    hold on;
    
    title(strcat("jRight",joint_names_list(j), " Effort"), 'FontSize', fontSize, 'fontweight','bold');
    
    xlabel('Samples', 'FontSize', fontSize);
    set (gca, 'FontSize' , fontSize)
    set (gca, 'ColorOrder' , C)
    axis tight
    
end

lh = legend(ax, 'Benchmark Joint Effort', 'FontSize', legendFontSize,...
           'Location', 'NorthOutside', 'Orientation','Vertical');
lh.Layout.Tile = 'North';

txt = title(tl, strcat("Joint Torque Estimates"), 'FontSize', fontSize, 'fontweight','bold');
txt.Interpreter= 'none';


%% Save figure
save2pdf(strcat(dataset_path + "joint_torque_estimates.pdf"), fH, 300);
