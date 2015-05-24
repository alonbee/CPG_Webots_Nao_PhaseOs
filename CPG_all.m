function [LHP_op,LHR_op,LKP_op,LAP_op,LAR_op,RHP_op,RHR_op,RKP_op,RAP_op,RAR_op,total_steps] = CPG_all(optpara,time_sim)
% TODO: 完成主要循环的修改
% TODO: 振荡器的输出幅值变换！！！
global TIME_STEP;
 
TIME_STEP =1;
% optpara = rand(29,1);
% time_sim = 20;

frequency = optpara(1);
offset = optpara(2);
hr_x = optpara(3);
hr_y = optpara(4);
hp_x = optpara(5);
hp_y = optpara(6);
kp_x1 = optpara(7);
kp_y1 = optpara(8);
kp_x2 = optpara(9);
kp_y2 = optpara(10);
kp_x3 = optpara(11);
kp_y3 = optpara(12);
kp_x4 = optpara(13);
kp_y4 = optpara(14);
ap_x1 = optpara(15);
ap_y1 = optpara(16);
ap_x2 = optpara(17);
ap_y2 = optpara(18);
ap_x3 = optpara(19);
ap_y3 = optpara(20);
ap_x4 = optpara(21);
ap_y4 = optpara(22);
ar_x = optpara(23);
ar_y = optpara(24);
phi_LRHP = optpara(25);
phi_HPKP = optpara(26);
phi_KPAP = optpara(27);
phi_HPR = optpara(28);
phi_HRAR = optpara(29);     

Time = time_sim;
Os_number = 10; % number of oscillators
functiontype = 0;  % functiontype = 0 : random output;  functiontype = 1 : normal output;
Bias = zeros(Os_number,Os_number); % Bias Matrix : LHP LHR LKP LAP LAR RHP RHR RKP RAP RAR
Bias = [0 phi_HPR phi_HPKP 0 0 phi_LRHP 0 0 0 0;
        phi_HPR 0 0 0 phi_HRAR 0 0 0 0 0;
        phi_HPKP 0 0 phi_KPAP 0 0 0 0 0 0;
        0 0 phi_KPAP 0 0 0 0 0 0 0;
        0 phi_HRAR 0 0 0 0 0 0 0 0;
        phi_LRHP 0 0 0 0 0 phi_HPR phi_HPKP 0 0;
        0 0 0 0 0 phi_HPR 0 0 0 phi_HRAR;
        0 0 0 0 0 phi_HPKP 0 0 phi_KPAP 0;
        0 0 0 0 0 0 0 phi_KPAP 0 0;
        0 0 0 0 0 0 phi_HRAR 0 0 0;
        ]
    
theta_ini = zeros(1,Os_number);
[T,Theta]=ode45('phaseos',[0:TIME_STEP * 10^(-2):Time],theta_ini,[],frequency,Bias);  % Theta(1-10): LHP LHR LKP LAP LAR RHP RHR RKP RAP RAR
total_steps = length(T);

[LHP_norm,LHP_normderive,LHP_theta] = normtrans(Theta(:,1),hp_x,hp_y,1);    % Left Hip Pitch 
[LHR_norm,LHR_normderive,LHR_theta] = normtrans(Theta(:,2),hr_x,hr_y,1);    % Left Hip Roll 
[LAR_norm,LAR_normderive,LAR_theta] = normtrans(Theta(:,5),ar_x,ar_y,1);    % Left Ankle Roll

[LKP_rand,LKP_randderive,LKP_theta] = randtrans(Theta(:,3),kp_x1,kp_x2,kp_x3,kp_x4,kp_y1,kp_y2,kp_y3,kp_y4,4);  % Left Knee Pitch : Random Insertion; LKP_rand 列向量
[LAP_rand,LAP_randderive,LAP_theta] = randtrans(Theta(:,4),ap_x1,ap_x2,ap_x3,ap_x4,ap_y1,ap_y2,ap_y3,ap_y4,4);  % Left Ankle Pitch : Random Insertion 


[RHP_norm,RHP_normderive,RHP_theta] = normtrans(Theta(:,6),hp_x,hp_y,1);    % Right Hip Pitch 
[RHR_norm,RHR_normderive,RHR_theta] = normtrans(Theta(:,7),hr_x,hr_y,1);    % Right Hip Roll 
[RAR_norm,RAR_normderive,RAR_theta] = normtrans(Theta(:,10),ar_x,ar_y,1);    % Right Ankle Roll

[RKP_rand,RKP_randderive,RKP_theta] = randtrans(Theta(:,8),kp_x1,kp_x2,kp_x3,kp_x4,kp_y1,kp_y2,kp_y3,kp_y4,4);  % Right Knee Pitch : Random Insertion; RKP_rand 列向量
[RAP_rand,RAP_randderive,RAP_theta] = randtrans(Theta(:,9),ap_x1,ap_x2,ap_x3,ap_x4,ap_y1,ap_y2,ap_y3,ap_y4,4);  % Right Ankle Pitch : Random Insertion 


output_norm = zeros(total_steps,6);   % LHP LHR LAR; RHP RHR RAR;
output_norm_dot = zeros(total_steps,6);
output_rand = zeros(total_steps,4);   % LKP LAP; RKP RAP;
output_rand_dot = zeros(total_steps,4);
output_norm_ini = [0 0 0 0 0 0];
output_rand_ini = [0 0 0 0];

output_norm(1,:) = output_norm_ini;
output_rand(1,:) = output_rand_ini;

Ki = 0;
gama = 1;
beta = 1;

dfdtheta_norm = zeros(total_steps,6);
dfdtheta_rand = zeros(total_steps,4);

dfdtheta_norm(:,1) = LHP_normderive;
dfdtheta_norm(:,2) = LHR_normderive;
dfdtheta_norm(:,3) = LAR_normderive;
dfdtheta_norm(:,4) = RHP_normderive;
dfdtheta_norm(:,5) = RHR_normderive;
dfdtheta_norm(:,6) = RAR_normderive;

dfdtheta_rand(:,1) = ppval(LKP_randderive,LKP_theta(:,:));
dfdtheta_rand(:,2) = ppval(LAP_randderive,LAP_theta(:,:));
dfdtheta_rand(:,3) = ppval(RKP_randderive,RKP_theta(:,:));
dfdtheta_rand(:,4) = ppval(RAP_randderive,RAP_theta(:,:));

for i = 1:total_steps - 1  
    Thetadot = phaseos([],Theta,[],frequency,Bias);  % Thetadot：（10,1）; 10*1 矩阵！！
    Thetadot_p(i,:) = Thetadot'/ (2*pi);    % Thetadot_p(1-10): LHP LHR LKP LAP LAR RHP RHR RKP RAP RAR

    output_norm_dot(i,1) = gama * (LHP_norm(i,1) - output_norm(i,1)) + beta * dfdtheta_norm (i,1) .* Thetadot_p(i,1)  + Ki; % LHP
    output_norm_dot(i,2) = gama * (LHR_norm(i,1) - output_norm(i,2)) + beta * dfdtheta_norm (i,2) .* Thetadot_p(i,2)  + Ki; % LHR
    output_norm_dot(i,3) = gama * (LAR_norm(i,1) - output_norm(i,3)) + beta * dfdtheta_norm (i,3) .* Thetadot_p(i,5)  + Ki; % LAR
    output_norm_dot(i,4) = gama * (RHP_norm(i,1) - output_norm(i,4)) + beta * dfdtheta_norm (i,4) .* Thetadot_p(i,6)  + Ki; % RHP 
    output_norm_dot(i,5) = gama * (RHR_norm(i,1) - output_norm(i,5)) + beta * dfdtheta_norm (i,5) .* Thetadot_p(i,7)  + Ki; % RHR 
    output_norm_dot(i,6) = gama * (RAR_norm(i,1) - output_norm(i,6)) + beta * dfdtheta_norm (i,6) .* Thetadot_p(i,10)  + Ki; % RAR 

    output_rand_dot(i,1) = gama * (LKP_rand(i,1) - output_rand(i,1)) + beta * dfdtheta_rand (i,1) .* Thetadot_p(i,3)  + Ki; % LKP
    output_rand_dot(i,2) = gama * (LAP_rand(i,1) - output_rand(i,2)) + beta * dfdtheta_rand (i,2) .* Thetadot_p(i,4)  + Ki; % LAP
    output_rand_dot(i,3) = gama * (RKP_rand(i,1) - output_rand(i,3)) + beta * dfdtheta_rand (i,3) .* Thetadot_p(i,8)  + Ki; % RKP
    output_rand_dot(i,4) = gama * (RAP_rand(i,1) - output_rand(i,4)) + beta * dfdtheta_rand (i,4) .* Thetadot_p(i,9)  + Ki; % RAP
    
    output_norm(i+1,:) = output_norm(i,:) + output_norm_dot(i,:) .* (T(i+1) - T(i));
    output_rand(i+1,:) = output_rand(i,:) + output_rand_dot(i,:) .* (T(i+1) - T(i));
    
end
figure(3);
plot(T,output_norm);
figure(4)
plot(T,output_rand);
LHP_op = output_norm(:,1);
LHR_op = output_norm(:,2);
LAR_op = output_norm(:,3); 
RHP_op = output_norm(:,4); 
RHR_op = output_norm(:,5); 
RAR_op = output_norm(:,6); 

LKP_op = output_rand(:,1);
LAP_op = output_rand(:,2);
RKP_op = output_rand(:,3);
RAP_op = output_rand(:,4);





 
    









