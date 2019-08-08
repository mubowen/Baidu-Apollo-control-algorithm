function K = solve_lqr_problem(A, B, Q, R, max_iter)
% 求解LQR的K值

% 输出:
% K         :

% 输入:
% A         : 状态矩阵
% B         : 控制矩阵
% Q         : 状态加权矩阵
% R         : 控制加权矩阵
% min_tol   : 最小允许误差
% max_iter  : 最大迭代次数

% 输出
% K: 控制增益

AT = A';
BT = B';

P = Q; %P的初始值为Q
n = 0; %迭代次数

min_tol = 0.0001;
while(n < max_iter)
    tmp1 = inv(R + BT * P * B);
    P_next = AT * P * A - AT * P * B * tmp1 * BT * P * A + Q;
    
    diff = norm(P_next - P); %迭代误差
    P = P_next;
    
    if (diff < min_tol)
        disp("Error,LQR cannot find the answer")
        break;
    end
    
    n = n + 1;
end

tmp2 = inv(R + BT * P * B);
K = tmp2 * BT * P * A;


