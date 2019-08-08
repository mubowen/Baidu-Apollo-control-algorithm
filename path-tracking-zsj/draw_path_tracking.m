function [path_figure, steer_figure] = draw_path_tracking(...
    path_tracking_alg, roadmap_name, trajref, veh_pose, steer_state,...
    veh_params, simulation_time, simulation_stop_time)
% 路径跟踪效果可视化：车辆跟踪效果、期望前轮偏角?

% 输出:
% path_figure           : 路径跟踪效果的figure
% steer_figure          : 期望前轮偏角的figure

% 输入:
% path_tracking_alg     : 路径跟踪算法
% roadmap_name          : 路网名称
% trajref               : 期望路径
% veh_pose              : 车辆当前位姿
% steer_state           : 前轮偏角当前状态?
% veh_params            : 车辆参数
% simulation_time       :当前仿真时间
% simulation_stop_time  : 仿真停止时间

% 1. 画出期望路径及车辆初始位姿状态?
screen_size = get(groot, 'Screensize'); %获取电脑显示屏的宽度和高度
screen_width = screen_size(3);          %屏幕宽度
screen_height = screen_size(4);         %屏幕高度

path_figure = figure('name', 'Path Tracking', 'position',...
    [1, screen_height/2, screen_width/2, screen_height/2]);
hold on;
grid minor;
axis equal;
path_figure_title_name = set_title_name(path_tracking_alg);
title(path_figure_title_name, 'fontsize', 15);  %设置title名称
path_figure_ylimit = set_y_limits(roadmap_name);
ylim(path_figure_ylimit);               %y轴范围
xlabel('X(m)', 'fontsize', 15);         %x轴名称
ylabel('Y(m)', 'fontsize', 15);         %y轴名称

plot(trajref(:, 1), trajref(:, 2), 'r.', 'markersize', 20); %期望轨迹可视化
draw_traj_curvature(trajref);   %期望轨迹每点的曲率可视化
draw_veh_pose(veh_pose, veh_params);    %车辆初始位姿可视化
%legend({'trajref', 'vehicle pose'}, 'fontsize', 12);        %图例

% 2. 画出前轮偏角
steer_figure = figure('name', 'Path Tracking', 'position',...
    [screen_width/2, screen_height/2, screen_width/2, screen_height/2]);
hold on;
grid minor;
steer_figure_title_name = set_title_name(path_tracking_alg);
title(steer_figure_title_name, 'fontsize', 15);
steer_figure_xlimit = simulation_stop_time;
steer_figure_ylimit = veh_params.max_steer_angle / pi * 180;
axis([0, steer_figure_xlimit, -steer_figure_ylimit, steer_figure_ylimit]);
xlabel('time(s)','fontsize', 15);
ylabel('steer command(deg)', 'fontsize', 15);

plot(simulation_time, steer_state, 'b.', 'markersize', 20);
%legend({'steer command'}, 'fontsize', 12);


