function [trajref_params, simulation_stop_y, simulation_stop_time] =...
    set_trajref_params(roadmap_name, veh_params)
% 设置期望路径参数及仿真停止条件?

% 输出:
% trajref_params        : 期望路径参数
% simulation_stop_y     : 当车辆y坐标值大于60时，仿真停止
% simulation_stop_time  : 仿真总时间, s

% 输入
% roadmap_name          : 路网名称

trajref_params.dist_interval = 0.2;     %trajref两点距离间隔, m

switch (roadmap_name)
    % 根据所选定的路网指定相应的期望路径参数
    case 'small_circle'
        trajref_params.traj1_dist = 30; %traj1的长度, m
        trajref_params.r2 = 5;          %traj2的半径, m
        trajref_params.traj3_dist = 15; %traj3的长度, m
        trajref_params.r4 = 5;          %traj4的半径, m
        trajref_params.r5 = 5;          %traj5的半径, m
        trajref_params.traj6_dist = 30; %traj6的长度?, m
        
        simulation_stop_y = 30;     %当车辆y坐标值大于30时，仿真停止
        simulation_stop_time = 90 / veh_params.velocity; %仿真总时间, s
        
    case 'big_circle'
        trajref_params.traj1_dist = 60;
        trajref_params.r2 = 10;
        trajref_params.traj3_dist = 30;
        trajref_params.r4 = 10;
        trajref_params.r5 = 10;
        trajref_params.traj6_dist = 30;
        
        simulation_stop_y = 60;
        simulation_stop_time = 180 / veh_params.velocity;
    case 'wave_test'
        trajref_params.traj1_dist = 230;
        
        simulation_stop_y = 60;
        simulation_stop_time = 180 / veh_params.velocity;
    otherwise
        disp('The roadmap does not exit!');
end

