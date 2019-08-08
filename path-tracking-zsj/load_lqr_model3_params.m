function lqr_model3_params = load_lqr_model3_params(trajref_params,...
    veh_params)
% 设置LQR_model3的参数?

% 输入:
% trajref_params : trajref的参数
% veh_params     : veh的参数

lqr_model3_params.delta_t = trajref_params.dist_interval /...
    veh_params.velocity;              %LQR的时间步长, s

lqr_model3_params.ref_index = 0;      %跟踪参考点
lqr_model3_params.horizon = 15;       %滚动优化窗口的大小

lqr_model3_params.Q = [10,  0,   0;
                        0, 5,   0;
                        0,  0, 100];  %状态误差权重矩阵 inital 10 10 100
                                                        %10 5 100 best
                    
lqr_model3_params.R = 10;             %控制量权重矩阵

lqr_model3_params.Q0 = [1, 0, 0;
                        0, 1, 0;
                        0, 0, 1];     %状态误差终端矩阵

