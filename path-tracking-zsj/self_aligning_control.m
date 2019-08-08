function steering_cmd_new=self_aligning_control(steering_cmd, steering_collect)
% set the correction control for the PP algorithm oscillation
right_angle_ratio=0.8;
left_angle_ratio=0.8;
adjust_angle=0.06;
delay_size=length(steering_collect);
curve_judge=false;
i=1;
%judge for the right turn condition
if (steering_cmd>0)
    while(i<=delay_size)
        if (steering_collect(i)-steering_cmd<-0.08)
            curve_judge=true;
        end
        i=i+1;
    end
    if (curve_judge)
    steering_cmd=(steering_cmd-adjust_angle)*right_angle_ratio;
    end
   %judge for the left turn condition
elseif (steering_cmd<0)
    while(i<=delay_size)
        if (steering_collect(i)-steering_cmd>0.08)
            curve_judge=true;
        end
        i=i+1;
    end
    if (curve_judge)
    steering_cmd=(steering_cmd+adjust_angle)*left_angle_ratio;
    end
end
   steering_cmd_new=steering_cmd;

end






























