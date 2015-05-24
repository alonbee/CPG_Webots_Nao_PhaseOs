function [output,derive,theta_m] = randtrans(theta,x1,x2,x3,x4,y1,y2,y3,y4,N)  
% TODO: pchip不是monotomic 的斜率可能会变化很大 ?
% TODO: line 35  /2pi 改变初始相位差吗？

% output： output of random interpolation 
% f1:     basic wave interpolation in function form
% f2：    basic wave interpolation in discrete points
% f2_der：derivative of f2
% derive： derivative in function form
% theta_m ：Theta transffered in [0 1]
% N： number of data point

[x,index] = sort([x1 x2 x3 x4]); % 对 x 各项排序
y_ini = [y1 y2 y3 y4];
y = [];
samppoint = 1.5;  % 在[-samppoint,samppoint] 内创建随机插值函数

for i=1:N
    y(i) = y_ini(index(i));   % 对xi对应的yi整理
end

x = [x(4)-1,x(3)-1,x,x(1)+1,x(2)+1];
y = [y(4),y(3),y,y(1),y(2)];   % 补充第一个点和最后一个点

t = -samppoint:0.001:samppoint; 
f1 = pchip(x,y);
f2 = pchip(x,y,t);            % Piecewise Cubic Hermite Interpolating Polynomial; 和 monotomnic 有点区别

% f2_der = diff(f2) / 0.001;  % f2_der 单个图像在 -2:2 的导数。 1 * 4000 的大小 

theta_m(:,:) = mod(theta(:,:),2 * pi) / ( 2 * pi );
%theta_m(:,:) = theta(:,:);

output = ppval(f1,theta_m);
derive = fnder(f1);

figure(1);
plot(t,f2);
    