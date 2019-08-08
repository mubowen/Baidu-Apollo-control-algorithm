function lqr_apollo_params = load_lqr_apollo_params(trajref_params,...
    veh_params)
% 设置LQR_Apollo的参数

% 输入:
% trajref_params : trajref的参数
% veh_params     : veh的参数

 %set up vehicle parameters
 cf = 155494.663;        %前轮侧偏刚度(cornering stiffness)  
 cr = 155494.663;        %后轮侧偏刚度
 
 mass_fl = 520;           %前悬长度
 mass_fr = 520;
 mass_rl = 520;
 mass_rr = 520;
 eps = 0.01;
 cutoff_freq = 10;
 mean_filter_window_size = 10;

 max_iteration = 150;
 max_throttle_minimum_action = 0.0;
 max_brake_minimum_action = 0.0;
 max_lateral_acceleration = 5.0;
 standstill_acceleration = -3.0;
 unconstraint_control_diff_limit = 5.0;

 mass_front =  mass_fl + mass_fr;
 mass_rear =  mass_rl + mass_rr;
 mass = mass_front + mass_rear;
 
 lf = veh_params.wheel_base * (1.0 - mass_front / mass); %前悬长度
 lr = veh_params.wheel_base * (1.0 - mass_rear / mass);  %后悬长度
 iz = lf * lf * mass_front + lr * lr * mass_rear;  %车辆绕z轴转动的转动惯量
  
 wheel_max_degree=veh_params.max_steer_angle;
 
 % set up matrix dimension
 basic_state_size = 4;
 control_size = 1;
 
 % matrix a
 matrix_a=zeros(basic_state_size,basic_state_size);
 matrix_a_coeff=zeros(basic_state_size,basic_state_size);
 I=eye(basic_state_size,basic_state_size);
 
 matrix_a(1,2) = 1.0;
 matrix_a(2,3) = (cf + cr ) / mass;
 matrix_a(3,4) = 1.0;
 matrix_a(4,3) = (lf * cf - lr * cr) / iz;

 matrix_a_coeff(2,2) = -(cf + cr) / mass;
 matrix_a_coeff(2,4) = (lr * cr - lf * cf) / mass;
 matrix_a_coeff(3,4) = 1.0;
 matrix_a_coeff(4,2) = (lr * cr - lf * cf) / iz;
 matrix_a_coeff(4,4) = -1.0 * (lf * lf * cf + lr * lr * cr)/iz;
 
 matrix_a(2,2)=matrix_a_coeff(2,2);
 matrix_a(2,4)=matrix_a_coeff(2,3);
 matrix_a(4,2)=matrix_a_coeff(4,2);
 matrix_a(4,4)=matrix_a_coeff(4,4);
 

 %matrix b
 matrix_b=zeros(basic_state_size,control_size);
 matrix_b(2,1) = cf/mass;
 matrix_b(4,1) = lf * cf /iz;

 
 % save the Matirx and paramters
 lqr_apollo_params.A=matrix_a;
 lqr_apollo_params.B=matrix_b;
 lqr_apollo_params.I=I;
 lqr_apollo_params.ts = 0.01; % time period
 lqr_apollo_params.lr=lr;
 lqr_apollo_params.lf=lf;
 lqr_apollo_params.cf=cf;
 lqr_apollo_params.cr=cr;
 lqr_apollo_params.iz=iz;
 lqr_apollo_params.mass=mass;

lqr_apollo_params.delta_t = trajref_params.dist_interval /...
    veh_params.velocity;              %LQR的时间步长, s

lqr_apollo_params.ref_index = 0;      %跟踪参考点
lqr_apollo_params.horizon = 15;       %滚动优化窗口的大小

matrix_q=zeros(4,4);                  %设置权重矩阵参数
matrix_q(1,1)=0.05;
matrix_q(3,3)=1;
lqr_apollo_params.Q =matrix_q ;     %状态误差权重矩阵

lqr_apollo_params.R = eye(1,1);      %控制量权重矩阵

lqr_apollo_params.Q0 = eye(4,4);     %状态误差终端矩阵

