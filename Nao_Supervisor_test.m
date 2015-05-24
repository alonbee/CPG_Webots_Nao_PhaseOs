global nao_robot;
global trans_field;
global rot_field;
global INITIAL_TRANS;
global INITIAL_ROT;

% controller time step
TIME_STEP = 32;

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

global LHipPitch_position;
global LHipRoll_position;
global LKneePitch_position;
global LAnklePitch_position;
global LAnkleRoll_position;
global RHipPitch_position;
global RHipRoll_position;
global RKneePitch_position;
global RAnklePitch_position;
global RAnkleRoll_position;

%% initialization of Webots, use nao_robocup.wbt
nao_robot = wb_supervisor_node_get_from_def('PLAYER'); 
trans_field = wb_supervisor_node_get_field(nao_robot,'translation'); % position
rot_field = wb_supervisor_node_get_field(nao_robot,'rotation');

INITIAL_TRANS = [0 0 0];
INITIAL_ROT = [1 0 0 -1.5708];  % 根据手动调整确定初始姿态

% get device
LHipPitch = wb_robot_get_device('LHipPitch');
LHipRoll = wb_robot_get_device('LHipRoll');
LKneePitch = wb_robot_get_device('LKneePitch');
LAnklePitch = wb_robot_get_device('LAnklePitch');
LAnkleRoll = wb_robot_get_device('LAnkleRoll');
RHipPitch = wb_robot_get_device('RHipPitch');
RHipRoll = wb_robot_get_device('RHipRoll');
RKneePitch = wb_robot_get_device('RKneePitch');
RAnklePitch = wb_robot_get_device('RAnklePitch');
RAnkleRoll = wb_robot_get_device('RAnkleRoll');

LHipPitch_position = wb_motor_get_target_position('LHipPitch');
LHipRoll_position = wb_motor_get_target_position('LHipRoll');
LKneePitch_position = wb_motor_get_target_position('LKneePitch');
LAnklePitch_position = wb_motor_get_target_position('LAnklePitch');
LAnkleRoll_position = wb_motor_get_target_position('LAnkleRoll');
RHipPitch_position = wb_motor_get_target_position('RHipPitch');
RHipRoll_position = wb_motor_get_target_position('RHipRoll');
RKneePitch_position = wb_motor_get_target_position('RKneePitch');
RAnklePitch_position = wb_motor_get_target_position('RAnklePitch');
RAnkleRoll_position = wb_motor_get_target_position('RAnkleRoll');

pause(2);
disp('recover to initial posture');

%% Move Robot to initial position (next individual)
wb_supervisor_field_set_sf_vec3f(trans_field, INITIAL_TRANS);
wb_supervisor_field_set_sf_rotation(rot_field, INITIAL_ROT);
%wb_supervisor_simulation_physics_reset();

steps = 0;
distance_T = 0;
position_translate{steps+1} = INITIAL_TRANS;

while wb_robot_step(TIME_STEP) ~= -1 && steps < 170
    steps = steps + 1;
    time = mod(steps,2*pi);
    wb_motor_set_position(LHipPitch,sin(time));
    wb_motor_set_position(LHipPitch,sin(time));
    wb_motor_set_position(LHipPitch,sin(time));
    wb_motor_set_position(LHipPitch,sin(time));
    wb_motor_set_position(LHipPitch,sin(time));
    wb_motor_set_position(LHipPitch,cos(time));
    wb_motor_set_position(LHipPitch,cos(time));
    wb_motor_set_position(LHipPitch,cos(time));
    wb_motor_set_position(LHipPitch,cos(time));
    wb_motor_set_position(LHipPitch,cos(time));
    position_translate{steps+1} = wb_supervisor_field_get_sf_vec3f(trans_field);  %supervisor 的节点可能不能用
    dist_everystep = sqrt((position_translate{steps+1}(1) - position_translate{steps}(1))^2 + (position_translate{steps+1}(3) - position_translate{steps}(3))^2);
    distance_T = distance_T + dist_everystep;
    wb_robot_step(TIME_STEP);
end
disp('Simulation completed');
wb_supervisor_simulation_reset_physics();






