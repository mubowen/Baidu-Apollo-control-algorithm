function steer_feedforward = calc_steer_feedforward(radius, wheel_base)
% 根据曲率半径计算对应点前轮偏角，作为前馈控制量

% 输出:
% steer_feedforward : 前馈前轮偏角, rad

% 输入:
% radius            : 点的半径值, m
% wheel_base        : 轴距, m

steer_feedforward = atan(wheel_base / radius);
