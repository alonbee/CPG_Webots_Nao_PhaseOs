function obj_val = Webots_PSO_Objfun(optval,dimension)
%% TODO: 需要对关节初始化吗？？

global nao_robot;
global trans_field;
global rot_field;
global INITIAL_TRANS;
global INITIAL_ROT;
global TIME_STEP;
global LHipPitch;
global LHipRoll;
global LKneePitch;
global LAnklePitch;
global LAnkleRoll;
global RHipPitch;
global RHipRoll;
global RKneePitch;
global RAnklePitch;
global RAnkleRoll;

simutime = 20; % 设置每次仿真时长

disp(optval);
[LHP_op,LHR_op,LKP_op,LAP_op,LAR_op,RHP_op,RHR_op,RKP_op,RAP_op,RAR_op,Steps] = CPG_all(optval,simutime);  % 每个关节的输出

time = 0;
distance_T = 0;
dist_everystep = 0;
position_translate{time+1} = INITIAL_TRANS;

%% Robot Initialization 
wb_supervisor_field_set_sf_vec3f(trans_field, INITIAL_TRANS);
wb_supervisor_field_set_sf_rotation(rot_field, INITIAL_ROT);
wb_supervisor_simulation_physics_reset();
pause(2);
disp('End of one Robot Initialization in Obj Function');

while wb_robot_step(TIME_STEP) ~= -1 && time < Steps
    time = time + 1;
    wb_motor_set_position(LHipPitch,LHP_op(time));
    wb_motor_set_position(LHipRoll,LHR_op(time));
    wb_motor_set_position(LKneePitch,LKP_op(time));
    wb_motor_set_position(LAnklePitch,LAP_op(time));
    wb_motor_set_position(LAnkleRoll,LAR_op(time));
    
    wb_motor_set_position(RHipPitch,RHP_op(time));
    wb_motor_set_position(RHipRoll,RHR_op(time));
    wb_motor_set_position(RKneePitch,RKP_op(time));
    wb_motor_set_position(RAnklePitch,RAP_op(time));
    wb_motor_set_position(RAnkleRoll,RAR_op(time));
    
    wb_robot_step(TIME_STEP);
    position_translate{time+1} = wb_supervisor_field_get_sf_vec3f(trans_field);  % supervisor 的节点可能不能用
    dist_everystep = sqrt((position_translate{time+1}(1) - position_translate{time}(1))^2 + (position_translate{time+1}(3) - position_translate{time}(3))^2);
    distance_T = distance_T + dist_everystep;   % 总路程作为object function
end

obj_val = 1 / distance_T;  % 最小值优化

disp('End of one Evaluation');

end
    
    
    
    
    
    
    
    
    