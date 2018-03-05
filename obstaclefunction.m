function J = obstaclefunction(x,w1,obstacles) 
% 障碍物斥力势场函数
% 人工势场法进行水下机器人路径规划，考虑体积范围

% 作者：李欣
% 单位：上海海事大学水下机器人与智能系统实验室
% Date: 2008-10-30
% Modified: 2010-1-5, 2014-11-12, 2018-3-5
% Shanghai, China 

%sigma = 1.5; % 参数σ
sigma = 1.5;
%r = 2.25; % 障碍物折合半径为1.5

%obstacles
%x m=2 n=20
[m,n] = size(obstacles);
dist = x*ones(1,n)-obstacles; % 2*1与1*n矩阵相乘，相当于等值扩展了
dist = sum(dist.^2); % 把列平方相加
J = min(dist); 
%if J >= 0.2
     J = w1*5.5*exp(-sigma*J); %用指数作为势场函数
%else 
%    J = w1*4.5*exp(-sigma*0.2);
%end