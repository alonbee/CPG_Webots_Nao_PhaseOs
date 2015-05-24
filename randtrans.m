function [output,derive,theta_m] = randtrans(theta,x1,x2,x3,x4,y1,y2,y3,y4,N)  
% TODO: pchip����monotomic ��б�ʿ��ܻ�仯�ܴ� ?
% TODO: line 35  /2pi �ı��ʼ��λ����

% output�� output of random interpolation 
% f1:     basic wave interpolation in function form
% f2��    basic wave interpolation in discrete points
% f2_der��derivative of f2
% derive�� derivative in function form
% theta_m ��Theta transffered in [0 1]
% N�� number of data point

[x,index] = sort([x1 x2 x3 x4]); % �� x ��������
y_ini = [y1 y2 y3 y4];
y = [];
samppoint = 1.5;  % ��[-samppoint,samppoint] �ڴ��������ֵ����

for i=1:N
    y(i) = y_ini(index(i));   % ��xi��Ӧ��yi����
end

x = [x(4)-1,x(3)-1,x,x(1)+1,x(2)+1];
y = [y(4),y(3),y,y(1),y(2)];   % �����һ��������һ����

t = -samppoint:0.001:samppoint; 
f1 = pchip(x,y);
f2 = pchip(x,y,t);            % Piecewise Cubic Hermite Interpolating Polynomial; �� monotomnic �е�����

% f2_der = diff(f2) / 0.001;  % f2_der ����ͼ���� -2:2 �ĵ����� 1 * 4000 �Ĵ�С 

theta_m(:,:) = mod(theta(:,:),2 * pi) / ( 2 * pi );
%theta_m(:,:) = theta(:,:);

output = ppval(f1,theta_m);
derive = fnder(f1);

figure(1);
plot(t,f2);
    