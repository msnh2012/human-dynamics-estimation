close all;
clear all;
clc;

N=8;
C = linspecer(N);
lineStyles = linspecer(N,'qualitative');

%% Plot parameters
fontSize  = 20;
legendFontSize  = 20;
lineWidth = 3;

gravity = 9.81;
objectMass = 9.55;

%% Legend or Title Index
wrenchLegendString = ["$f_x [N]$", "$f_y [N]$", "$f_z [N]$","$m_x [Nm]$", "$m_y [Nm]$", "$m_z [Nm]$"];

dataset_dir = '/home/yeshi/Desktop/non_collocated_wrench_estimation_dataset/';
dataset_type = 'covariance_tuning_dataset';
dataset_source = 'AMTI_Dataset';
subject = 'sub03';
dataset = 'tpose/hde';

dataset_path = strcat(dataset_dir,'/',dataset_type,'/',dataset_source,'/',subject,'/',dataset, '/'); 

Data =  load(strcat(dataset_path,'1e-4/matLogFile.mat'));



joint_names = Data.dynamicsJointNames;

joint_torque_estimates = [];

for j = 1:size(joint_names, 1)
    joint_torque_estimates.(cell2mat(joint_names(j))).joint_torques = Data.data.jointTorques(j, :)';
end


fH = figure('units','normalized','outerposition',[0 0 1 1]);
tl = tiledlayout(4,2);

joint_name = "Wrist";
joint_suffix = ["_rotx", "_roty", "_rotz"];

for l = 1:size(joint_suffix, 2)
    
    nexttile;
    
    plot(joint_torque_estimates.(strcat("jLeft",joint_name, joint_suffix(l))).joint_torques, '-',...
             'LineWidth', lineWidth,...
             'Color', C(l,:));
    hold on;
    
    txt = title(strcat("jLeft",joint_name, joint_suffix(l)), 'FontSize', fontSize, 'fontweight','bold');
    txt.Interpreter= 'none'; 
    
    ylabel("$\tau$ [$Nm$]", 'Interpreter', 'latex', 'FontSize', fontSize);
    set (gca, 'FontSize' , fontSize)
    set (gca, 'ColorOrder' , C)
    axis tight
    
    nexttile;
    
    plot(joint_torque_estimates.(strcat("jRight",joint_name, joint_suffix(l))).joint_torques, '-',...
        'LineWidth', lineWidth,...
        'Color', C(l,:));
    hold on;
    

    txt = title(strcat("jRight",joint_name, joint_suffix(l)), 'FontSize', fontSize, 'fontweight','bold');
    txt.Interpreter= 'none'; 
    
    
    ylabel("$\tau$ [$Nm$]", 'Interpreter', 'latex', 'FontSize', fontSize);
    set (gca, 'FontSize' , fontSize)
    set (gca, 'ColorOrder' , C)
    axis tight
    
    
end



dataSize = size( joint_torque_estimates.(strcat("jLeft",joint_name, joint_suffix(1))).joint_torques, 1);

for e = 1:dataSize
    
    joint_torque_estimates.(strcat("jLeft", joint_name)).effort(e) = norm([joint_torque_estimates.(strcat("jLeft",joint_name, joint_suffix(1))).joint_torques(e),...
        joint_torque_estimates.(strcat("jLeft",joint_name, joint_suffix(2))).joint_torques(e),...
        joint_torque_estimates.(strcat("jLeft",joint_name, joint_suffix(3))).joint_torques(e)]);
    
    joint_torque_estimates.(strcat("jRight", joint_name)).effort(e) = norm([joint_torque_estimates.(strcat("jRight",joint_name, joint_suffix(1))).joint_torques(e),...
        joint_torque_estimates.(strcat("jRight",joint_name, joint_suffix(2))).joint_torques(e),...
        joint_torque_estimates.(strcat("jRight",joint_name, joint_suffix(3))).joint_torques(e)]);
    
end
    
 

nexttile

plot(joint_torque_estimates.(strcat("jLeft", joint_name)).effort, '-',...
    'LineWidth', lineWidth,...
    'Color', C(6,:));
hold on;


title(strcat("jLeft",joint_name, " Effort"), 'FontSize', fontSize, 'fontweight','bold');

xlabel('Samples', 'FontSize', fontSize);
ylabel("$\tau$ [$Nm$]", 'Interpreter', 'latex', 'FontSize', fontSize);
set (gca, 'FontSize' , fontSize)
set (gca, 'ColorOrder' , C)
axis tight


nexttile
plot(joint_torque_estimates.(strcat("jRight", joint_name)).effort, '-',...
    'LineWidth', lineWidth,...
    'Color', C(6,:));
hold on;

title(strcat("jRight",joint_name, " Effort"), 'FontSize', fontSize, 'fontweight','bold');

xlabel('Samples', 'FontSize', fontSize);
ylabel("$\tau$ [$Nm$]", 'Interpreter', 'latex', 'FontSize', fontSize);
set (gca, 'FontSize' , fontSize)
set (gca, 'ColorOrder' , C)
axis tight

txt = title(tl, strcat(joint_name, " Joint Torque Estimates"), 'FontSize', fontSize, 'fontweight','bold');
txt.Interpreter= 'none'; 
