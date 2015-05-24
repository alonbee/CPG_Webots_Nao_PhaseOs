function thetadot = phaseos(t,theta,flag,w,Bias) % ���ж������ʱҪ��flag����
% [t,f]=ode45(@(t,y)fu  n(t,y,a),[t0 tf],[y0 yf]); % �����ⲿ�Ĳ��������� ��PSO�е� w phi ��
%w = 5;
osnum = length(Bias);
sinmatrix = zeros(osnum,osnum);  % ��¼�����������sinֵ
thetadot = zeros(osnum,1);

for i = 1:osnum
    for j = 1:osnum
        sinmatrix(i,j) = sin(theta(j) - theta(i) - Bias(i,j));  %��Bias�Խ���Ϊ0
        thetadot(i) = thetadot(i) + sinmatrix(i,j);
        if j == osnum
            thetadot(i) = thetadot(i) + w;
        end
    end
end
    


% phi12 = 1;
% phi13 = 2;
% phi23 = 2;
% thetadot = zeros(3,1);
% %trans = zeros(:,1);
%     
% thetadot(1) = w + (sin(theta(2) - theta(1) - phi12) +  sin(theta(3)-theta(1) - phi13));
% thetadot(2) = w + (sin(theta(1) - theta(2) - phi12) +  sin(theta(3)-theta(2) - phi23));
% thetadot(3) = w + (sin(theta(1) - theta(3) - phi13) + sin(theta(2)-theta(3) - phi23));
% 
% % figure(1)
% %trans = normtrans(theta,1);
% %randi = randtrans(theta,rand(1),rand(1),rand(1),rand(1),rand(1),rand(1),rand(1),rand(1));
% 
% % plot(t,trans(1),'ro');
% % plot(t,trans(2),'bo');
% % plot(t,trans(3),'go');
% % % plot(t,trans(1));
%  %hold on







