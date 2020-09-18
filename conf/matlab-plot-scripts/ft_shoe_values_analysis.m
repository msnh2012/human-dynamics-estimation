close all;
clear all;
clc;

%% Plot parameters
fontSize  = 20;
lineWidth = 3;

%% Legend or Title Index
wrenchLegendString = ["$f_x [N]$", "$f_y [N]$", "$f_z [N]$","$m_x [Nm]$", "$m_y [Nm]$", "$m_z [Nm]$"];


dataPath     = "/home/yeshi/dumper/ft_without_insitu_5kg/";
ftShoe       = "ftShoe_left";
ftShoe_front = "_front";
ftShoe_rear  = "_rear";
analog       = "/analog_o";

ftShoe_data        = load(dataPath + ftShoe + analog + "/data.log"); 
ftShoe_front_data  = load(dataPath + ftShoe + ftShoe_front + analog + "/data.log"); 
ftShoe_rear_data   = load(dataPath + ftShoe + ftShoe_rear + analog + "/data.log");

fH = figure('units','normalized','outerposition',[0 0 1 1]);

for s = 1:6
    subplot(2,3,s)
    plot(ftShoe_front_data(:,s+2), 'LineWidth', lineWidth);
    hold on;
    plot(ftShoe_rear_data(:,s+2), 'LineWidth', lineWidth);
    hold on;
    plot(ftShoe_data(:,s+2), 'LineWidth', lineWidth, 'LineStyle', '--');
    hold on;
    xlabel('Samples', 'FontSize', fontSize);
    ylabel(wrenchLegendString(s), 'Interpreter', 'latex', 'FontSize', fontSize);
    set (gca, 'FontSize' , fontSize)
    legend('Front', 'Rear', 'Shoe',...
           'FontSize', fontSize, 'Location', 'Best');
end


a = axes;
t = title ( replace(ftShoe,"_", "  ") + " Values", 'Interpreter', 'latex');
t.FontSize = fontSize;
a.Visible = 'off' ;
t.Visible = 'on' ;
axis(a,'fill')


ftShoe_data_norm       = [];
ftShoe_front_data_norm = [];
ftShoe_rear_data_norm  = [];

for i=1:size(ftShoe_data, 1)
    ftShoe_data_norm(i) = norm(ftShoe_data(i,3:5));
end

for i=1:size(ftShoe_front_data, 1)
    ftShoe_front_data_norm(i) = norm(ftShoe_front_data(i,3:5));
end

for i=1:size(ftShoe_rear_data, 1)
    ftShoe_rear_data_norm(i) = norm(ftShoe_rear_data(i,3:5));
end

fH = figure('units','normalized','outerposition',[0 0 1 1]);
plot(ftShoe_front_data_norm, 'LineWidth', lineWidth);
hold on;
plot(ftShoe_rear_data_norm, 'LineWidth', lineWidth);
hold on;
plot(ftShoe_data_norm, 'LineWidth', lineWidth, 'LineStyle', '--');
hold on;
legend('Front', 'Rear', 'Shoe',...
           'FontSize', fontSize, 'Location', 'Best');
       

a = axes;
t = title ( replace(ftShoe,"_", "  ") + " Force Norms", 'Interpreter', 'latex');
t.FontSize = fontSize;
a.Visible = 'off' ;
t.Visible = 'on' ;
axis(a,'fill')

