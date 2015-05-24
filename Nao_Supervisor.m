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
% INITIAL_ROT = [1 0 0 -1.5708];  % �����ֶ�����ȷ����ʼ��̬
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


% PSO �����
% swarm_sample = [frequency,offset,hr_x,hr_y,hp_x,hp_y,kp_x1,kp_y1,kp_x2,kp_y2,kp_x3,kp_y3,kp_x4,kp_y4,ap_x1,ap_y1,ap_x2,ap_y2,ap_x3,ap_y3,ap_x4,ap_y4,ar_x1,ar_y1,,,,
            ...phi_LRHP,phi_HPKP,phi_KPAP,phi_HPR,phi_HRAR];
upper_bd = [10 15*3.14/180 0.25 45*3.14/180 0.25 25*3.14/180 1 130*3.14/180 1 130*3.14/180 1 130*3.14/180 1 130*3.14/180 1 45*3.14/180 1 45*3.14/180 1 45*3.14/180 1 45*3.14/180 0.25 25*3.14/180 0 pi/4 pi/4 pi/2 pi/4];    % LRHP ������Ȩֵһ��Ϊ����
lower_bd = [0.2 -40*3.14/180 0 -25*3.14/180 0 -100*3.14/180 0 0 0 0 0 0 0 0 0 -75*3.14/180 0 -75*3.14/180 0 -75*3.14/180 0 -75*3.14/180 0 -45*3.14/180 -2*pi/3 -pi/4 -pi/4 -pi/2 -pi/4 ]; 
swarm_bound = [upper_bd;lower_bd];

N=40;                     %��ʼ�����Ӹ���
D = length(swarm_bound);     %��ʼ��Ⱥ��ά��
Num_iteration = 20;            %��ʼ��Ⱥ�����������
c1 = 1.5;                 %�ֲ�ѧϰ����1 cognitive weight
c2 = 2;                   %ȫ��ѧϰ����2   social weight
w = 1.2;                  %����Ȩ��
eps=10^(-6);            %���þ��ȣ�����֪��Сֵ��ʱ���ã�
bound = zeros(2,D);
bound = swarm_bound;

x = zeros(N,D);
v = zeros(N,D);
Fitness_table = zeros(N,1);

for i=1:N
    for j=1:D
        x(i,j) =  bound(2,j) + (rand * (bound(1,j) - bound(2,j)));  % ����ʼ�� x �����ڸ�����Χ��
        v(i,j) = randn;   % �����ʼ���ٶ�
    end
end

%------��ʾȺλ��----------------------
figure(1)
dim = D / 4;
for j=1:dim     
    if(rem(dim,2)>0)
        subplot((dim+1)/2,2,j)
    else
        subplot(dim/2,2,j)
    end    
    plot(x(:,j),'b*');grid on
    xlabel('����')
    ylabel('��ʼλ��')
    tInfo=strcat('��',char(j+48),'ά');
    if(j>9)
  tInfo=strcat('��',char(floor(j/10)+48));
char((rem(j,10)+48),'ά');
    end    
    title(tInfo)
end

%------��ʼ����Ⱥ���壨�ڴ��޶��ٶȺ�λ�ã�------------
x1 = x;
v1 = v;
%------��ʼ����������λ�ú�����ֵ---
p1 = x1;
pbest1=ones(N,1);
for i=1:N
    pbest1(i) = Webots_PSO_Objfun(x1(i,:),D);   % ��N����Ӧ�Ⱥ���
    Fitness_table(i) = pbest1(i);
end
%------��ʼ��ȫ������λ�ú�����ֵ(���ֵ)---------------
g1 = zeros(1,D);    
gbest1 = 0;

[gbest1,gbest1_index] = min(Fitness_table); % ����Сֵ
g1 = p1(gbest1_index,:);

gb1 = zeros(1,Num_iteration);     %����¼ÿһ������ֵgbest
satu = zeros(N,D,Num_iteration);  %  ��¼ÿһ��Խ�����

%-----������ѭ�������չ�ʽ���ε���ֱ�����㾫�Ȼ��ߵ�������---
for i=1:Num_iteration
    for j=1:N
        Fitness_table(j) = Webots_PSO_Objfun(x1(j,:),D); % ����ÿһ����fitness 
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
             satu(k,j,i) = 1;            % ����ÿһ��Խ������� ��Խ��:1; ��Խ��:-1;
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
bestpar = g1;   % �����Ѳ���ֵ
bestoutput = gbest1;    %����������Ӧ��ֵ

figure(2);
plot(gb1);   % ���ÿһ��������fitnessֵ
TempStr = sprintf('c1= %g ,c2=%g',c1,c2);
title(TempStr);
xlabel('��������');
ylabel('��Ӧ��ֵ');














