clear;
clc
format long;

global nao_robot;
global trans_field;
global rot_field;
global INITIAL_TRANS;
global INITIAL_ROT;

% controller time step: 32 ms
global TIME_STEP; 

TIME_STEP =1;

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
% nao_robot = wb_supervisor_node_get_from_def('PLAYER'); 
% trans_field = wb_supervisor_node_get_field(nao_robot,'translation'); % position
% rot_field = wb_supervisor_node_get_field(nao_robot,'rotation');
% 
% INITIAL_TRANS = [0 0 0];
% INITIAL_ROT = [1 0 0 -1.5708];  % 根据手动调整确定初始姿态
% 
% % get device
% LHipPitch = wb_robot_get_device('LHipPitch');
% LHipRoll = wb_robot_get_device('LHipRoll');
% LKneePitch = wb_robot_get_device('LKneePitch');
% LAnklePitch = wb_robot_get_device('LAnklePitch');
% LAnkleRoll = wb_robot_get_device('LAnkleRoll');
% RHipPitch = wb_robot_get_device('RHipPitch');
% RHipRoll = wb_robot_get_device('RHipRoll');
% RKneePitch = wb_robot_get_device('RKneePitch');
% RAnklePitch = wb_robot_get_device('RAnklePitch');
% RAnkleRoll = wb_robot_get_device('RAnkleRoll');
% 
% LHipPitch_position = wb_motor_get_target_position('LHipPitch');
% LHipRoll_position = wb_motor_get_target_position('LHipRoll');
% LKneePitch_position = wb_motor_get_target_position('LKneePitch');
% LAnklePitch_position = wb_motor_get_target_position('LAnklePitch');
% LAnkleRoll_position = wb_motor_get_target_position('LAnkleRoll');
% RHipPitch_position = wb_motor_get_target_position('RHipPitch');
% RHipRoll_position = wb_motor_get_target_position('RHipRoll');
% RKneePitch_position = wb_motor_get_target_position('RKneePitch');
% RAnklePitch_position = wb_motor_get_target_position('RAnklePitch');
% RAnkleRoll_position = wb_motor_get_target_position('RAnkleRoll');
% 
% pause(2);
% disp('recover to initial posture');
% 
% %% Move Robot to initial position (next individual)
% wb_supervisor_field_set_sf_vec3f(trans_field, INITIAL_TRANS);
% wb_supervisor_field_set_sf_rotation(rot_field, INITIAL_ROT);
% wb_supervisor_simulation_physics_reset();
% 
% 
% 
%% PSO Optimization


% PSO 编码表
% swarm_sample = [frequency,offset,hr_x,hr_y,hp_x,hp_y,kp_x1,kp_y1,kp_x2,kp_y2,kp_x3,kp_y3,kp_x4,kp_y4,ap_x1,ap_y1,ap_x2,ap_y2,ap_x3,ap_y3,ap_x4,ap_y4,ar_x1,ar_y1,,,,
            ...phi_LRHP,phi_HPKP,phi_KPAP,phi_HPR,phi_HRAR];
upper_bd = [10 15*3.14/180 0.25 45*3.14/180 0.25 25*3.14/180 1 130*3.14/180 1 130*3.14/180 1 130*3.14/180 1 130*3.14/180 1 45*3.14/180 1 45*3.14/180 1 45*3.14/180 1 45*3.14/180 0.25 25*3.14/180 0 pi/4 pi/4 pi/2 pi/4];    % LRHP 的连接权值一定为负数
lower_bd = [0.2 -40*3.14/180 0 -25*3.14/180 0 -100*3.14/180 0 0 0 0 0 0 0 0 0 -75*3.14/180 0 -75*3.14/180 0 -75*3.14/180 0 -75*3.14/180 0 -45*3.14/180 -2*pi/3 -pi/4 -pi/4 -pi/2 -pi/4 ]; 
swarm_bound = [upper_bd;lower_bd];

N=40;                     %初始化粒子个数
D = length(swarm_bound);     %初始化群体维数
Num_iteration = 20;            %初始化群体最迭代次数
c1 = 1.5;                 %局部学习因子1 cognitive weight
c2 = 2;                   %全局学习因子2   social weight
w = 1.2;                  %惯性权重
eps=10^(-6);            %设置精度（在已知最小值的时候用）
bound = zeros(2,D);
bound = swarm_bound;

x = zeros(N,D);
v = zeros(N,D);
Fitness_table = zeros(N,1);

for i=1:N
    for j=1:D
        x(i,j) =  bound(2,j) + (rand * (bound(1,j) - bound(2,j)));  % 将初始化 x 限制在给定范围内
        v(i,j) = randn;   % 随机初始化速度
    end
end

%------显示群位置----------------------
figure(1)
dim = D / 4;
for j=1:dim     
    if(rem(dim,2)>0)
        subplot((dim+1)/2,2,j)
    else
        subplot(dim/2,2,j)
    end    
    plot(x(:,j),'b*');grid on
    xlabel('粒子')
    ylabel('初始位置')
    tInfo=strcat('第',char(j+48),'维');
    if(j>9)
  tInfo=strcat('第',char(floor(j/10)+48));
char((rem(j,10)+48),'维');
    end    
    title(tInfo)
end

%------初始化种群个体（在此限定速度和位置）------------
x1 = x;
v1 = v;
%------初始化个体最优位置和最优值---
p1 = x1;
pbest1=ones(N,1);
for i=1:N
    pbest1(i) = Webots_PSO_Objfun(x1(i,:),D);   % 算N次适应度函数
    Fitness_table(i) = pbest1(i);
end
%------初始化全局最优位置和最优值(最大值)---------------
g1 = zeros(1,D);    
gbest1 = 0;

[gbest1,gbest1_index] = min(Fitness_table); % 求最小值
g1 = p1(gbest1_index,:);

gb1 = zeros(1,Num_iteration);     %　记录每一代最优值gbest
satu = zeros(N,D,Num_iteration);  %  记录每一代越界情况

%-----浸入主循环，按照公式依次迭代直到满足精度或者迭代次数---
for i=1:Num_iteration
    for j=1:N
        Fitness_table(j) = Webots_PSO_Objfun(x1(j,:),D); % 记下每一代的fitness 
        if (Fitness_table(j) < pbest1(j))
            p1(j,:)=x1(j,:);
            pbest1(j) = Fitness_table(j);
        end
        if(pbest1(j) < gbest1)
            g1=p1(j,:);
            gbest1 = pbest1(j);
        end
        v1(j,:)= w * v1(j,:) + c1*rand*(p1(j,:)- x1(j,:)) + c2*rand*(g1 - x1(j,:));
        x1(j,:)=x1(j,:)+v1(j,:);
        for k = 1:D
         if x1(j,k) > bound(1,k)
             x1(j,k) = bound(1,k);
             satu(k,j,i) = 1;            % 记下每一代越界情况； 上越界:1; 下越界:-1;
         else if x1(j,k) < bound(2,k)
                 x1(j,k) = bound(2,k);
                 satu(k,j,i) = -1; 
             end
         end
        end
    end
    gb1(i)=gbest1;
end

disp('Optimization finished');
bestpar = g1;   % 输出最佳参数值
bestoutput = gbest1;    %　输出最佳适应度值

figure(2);
plot(gb1);   % 输出每一代的最优fitness值
TempStr = sprintf('c1= %g ,c2=%g',c1,c2);
title(TempStr);
xlabel('迭代次数');
ylabel('适应度值');














