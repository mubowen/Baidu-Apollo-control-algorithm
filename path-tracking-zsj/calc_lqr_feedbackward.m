function steer_feedbackward = calc_lqr_feedbackward(trajref,...
    delta_x, lqr_params, start_index, veh_params, steer_feedforward)
% 计算lqr的前轮偏角反馈控制量  //姜老师书69页

% 输出:
% steer_feedbackward: 前轮偏角反馈控制量, rad

% 输入:
% trajref           : 期望路径[X, Y, Theta, Radius]
% delta_x           : 期望位姿与车辆当前位姿的偏差[dx, dy, dtheta]
% lqr_params        : LQR的参数
% start_index       : 滚动优化在trajref的初始index
% veh_params        : 车辆参数
% steer_feedforward : 前轮偏角前馈控制量, rad

% 1. 设置LQR的参数
delta_t = lqr_params.delta_t;   %LQR的时间步长, s
horizon = lqr_params.horizon;   %滚动优化窗口大小
Q = lqr_params.Q;               %状态误差权重矩阵
R = lqr_params.R;               %控制量权重矩阵?
Q0 = lqr_params.Q0;             %状态误差终端矩阵

% 2. 设置车辆参数
vel = veh_params.velocity;      %车速, m/s
wheel_base = veh_params.wheel_base;     %轴距, m

% 3. 将航向偏差限制到[-pi, pi]
delta_x(3) = angle_normalization(delta_x(3));

% 4. 设置Pk，Vk在终端状态时刻的值
Pk = Q0;
Vk = [0; 0; 0];    %终端状态 

% 5. 滚动优化
end_index = start_index + horizon;  %设置滚动优化在trajref上的起始index
for i = end_index : -1  : (start_index + 1)
    Pk_1 = Pk;
    Vk_1 = Vk;
    
    ref_theta = trajref(i-1, 3);        %参考点的航向角
    ref_delta = calc_steer_feedforward(...
        trajref(i-1, 4), wheel_base);   %参考点的前轮偏角
    
    A = [1 0 -vel * sin(ref_theta) * delta_t;
         0 1  vel * cos(ref_theta) * delta_t;
         0 0  1];
    B = [0;
         0;
         vel * delta_t / (wheel_base * cos(ref_delta)^2)];
    
    tmp = B' * Pk_1 * B + R;
    K = B' * Pk_1 * A / tmp;
    Ku = R / tmp;
    Kv = B' / tmp;
    
    Pk = A' * Pk_1 * (A - B * K) + Q;
    u_feedforward = ref_delta;
    Vk = (A - B * K) * Vk_1 - K' * R * u_feedforward;
end

% 6. 求得lqr的前轮偏角反馈控制量
steer_feedbackward = -K * delta_x - Ku * steer_feedforward - Kv * Vk_1;

