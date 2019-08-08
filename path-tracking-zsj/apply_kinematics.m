function [vehicle_pose]=apply_kinematics(vehicle_pose,vehicle_m,rad_speed,deltaT)

vehicle_pose=vehicle_pose+[vehicle_m*cos(vehicle_pose(3))*deltaT; ...
    vehicle_m*sin(vehicle_pose(3))*deltaT; rad_speed*deltaT];
end