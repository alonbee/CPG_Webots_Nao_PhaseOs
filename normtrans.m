function [normoutput,normoutput_derive,theta_m] = normtrans(theta,p1,amp,os_num)
% function [normoutput,normoutput_derive] = normtrans(theta,amp,bound,os_num)
% TODO: bound 受 amp 的影响 ??? 
% TODO: normoutput 和 振荡器最终输出幅值什么关系？？
% bound 限幅值, 为[0,1]
% if bound > 1 
%     bound  = 1;
% else if bound < 0 
%         bound = 0;
%     end
% end
if p1<0 || p1 > 0.25
    disp('Invalid normal insertion data point p1 = '); 
    disp(p1);
end

theta_m(:,:) = mod(theta(:,:),2 * pi) / ( 2 * pi );

[row,col] = size(theta_m);
normoutput = zeros(row,os_num);
normoutput_derive = zeros(row,os_num);

x1 = p1;
x2 = 0.5 - x1;
x3 = 0.5 + x1;
x4 = 1 - x1;

for i = 1:row
    for j = 1:col
        if theta_m(i,j) > x1 && theta_m(i,j) < x2
            normoutput(i,j) = amp * sin(x1 * 2 * pi);
            normoutput_derive(i,j) = 0;
        else if theta_m(i,j) > x3 && theta_m(i,j) < x4
                normoutput(i,j) = - amp * sin(x1 * 2 * pi); % x1=[0,0.25]; 将x1 * 2 pi 计算sin值
                normoutput_derive(i,j) = 0;
            else
                normoutput(i,j) = amp * sin(theta_m(i,j) * 2 * pi);
                normoutput_derive(i,j) = amp * cos(theta_m(i,j) * 2 *pi);
            end
        end
    end
end
end
            
     
    
