function title_name = set_title_name(path_tracking_alg)
% 设置使用的路径跟踪方法名称

% 输出:
% title_name        : 标题名称

% 输入
% path_tracking_alg : 路径跟踪算法

switch (path_tracking_alg)
    % 根据所选路径跟踪算法指定figure的title
    case 'PP'
        title_name = 'Path Tracking - PP';
        
    case 'CIRCLE_PP'
        title_name = 'Path Tracking - CIRCLE PP';
        
    case 'LQR_model3'
        title_name = 'Path Tracking - LQR model3';
        
    case 'LQR_model4'
        title_name = 'Path Tracking - LQR model4';
    
    case 'MPC_apollo'
        title_name = 'Path Tracking - MPC apollo';

    case 'LQR_apollo'
        title_name = 'Path Tracking - LQR apollo';
        
    otherwise
        title_name = 'Error - No Algorithm';
        disp('There is no this path tracking algorithm!');
end

