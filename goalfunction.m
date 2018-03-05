function J=goalfunction(x,xgoal,w2) 
% 目标引力势场函数
% 人工势场法进行水下机器人路径规划，考虑体积范围

% 作者：李欣
% 单位：上海海事大学水下机器人与智能系统实验室
% Date: 2008-10-30
% Modified: 2010-1-5, 2014-11-12, 2018-3-5
% Shanghai, China 

%[xx(jj);yy(ii)],xgoal,w2); (x,w1)
%r = 1.5 % 设目标折合半径为1.5
%r = 2.25
d = x - xgoal;
d = sum(d.^2); % 欧氏距离的平方

J = w2*2*(exp(-0.5*d)); % 指数函数
J = J+w2*10*(exp(-0.01*d));
%else
%    J = w2*2*(exp(-0.5*1));
 %   J = J+w2*10*(exp(-0.01*1));
%end
