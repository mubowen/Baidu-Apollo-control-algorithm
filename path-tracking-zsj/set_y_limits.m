function y_limits = set_y_limits(roadmap_name)
% 设置figure显示的y轴范围

% 输出:
% y_limits     : y轴范围

% 输入
% roadmap_name : 路网名称

switch (roadmap_name)
    % 根据路网指定figure的y轴范围
    case 'small_circle'
        y_limits = [-2, 35];
        
    case 'big_circle'
        y_limits = [-2, 70];
    
    case 'wave_test'
        y_limits = [-2, 70];
    otherwise
        disp('The roadmap does not exit!');
end

