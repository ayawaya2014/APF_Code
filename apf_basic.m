%% function apf_3
% 人工势场法进行水下机器人路径规划，考虑体积范围
% 计算势函数，画出运动轨迹图像

% 作者：李欣
% 单位：上海海事大学水下机器人与智能系统实验室
% Date: 2008-10-30
% Modified: 2010-1-5, 2014-11-12, 2018-3-5
% Shanghai, China

close all; 

% 设置工作区域
xmin = [0; 0];  
xmax = [50;50];

% Maximum number
Nsteps = 600;

%设置机器人的参数%
% 选定方向上机器人运动步长参数
lambda = 0.1;
Ns=30; 
r = 1; 
xs=0*ones(2,Ns); 
Jo(:,1)=0*ones(Ns,1); 
Jg(:,1)=0*ones(Ns,1); 
J(:,1)=0*ones(Ns,1); 
theta(:,1)=0*ones(Ns,1);

for m=2:Ns
    theta(m,1)=theta(m-1,1)+(pi/180)*(360/Ns); 
end 
  
% 设置目标(Goal/Target)位置坐标
P_Goal=[25; 25];
obstacles = [6 20 11 16 18 19 ;6 16 17 14 11.9 19];
Mat = size(obstacles); %障碍物点数
obNum = Mat(1,2);
nt = 20; % Tar运动步数
nr = 20; % Ro的速度，决定能否跟的上
x1 = 1;
y1 = 1;
g = 1;
h = 0;
distrt = 0; % 计算距离，终止条件
distro = 0*ones(2,obNum); % 计算距离，避让临界
t = 0;
na = 0;

% 设置机器人初始位置坐标
P_Ro=[5; 5]; 
w1 = 1; 
w2 = 5; 
P_Ro(:,2:Nsteps) = 0*ones(2,Nsteps-1);

% 画出势场
xx=0:35/100:35; 
yy=xx; 

% 计算障碍物势函数 
 for jj=1:length(xx) 
    for ii=1:length(yy) 
       op(ii,jj)=obstaclefunction([xx(jj);yy(ii)],w1,obstacles); 
    end 
 end 
 
% 计算目标势函数
 for jj=1:length(xx) 
    for ii=1:length(yy) 
        gp(ii,jj)=goalfunction([xx(jj);yy(ii)],P_Goal(:,1),w2); 
    end 
 end 

P_RoA = P_Ro(1,1);
P_RoB = P_Ro(2,1);
P_RoC = P_Ro(1,1);
P_RoD = P_Ro(2,1);

potential = gp - op;

figure;

plot(P_Goal(1,1),P_Goal(2,1),'+','MarkerSize',10);
hold on
plot(P_Goal(1,1),P_Goal(2,1),'o','MarkerSize',22);

xlabel('x'); 
ylabel('y'); 

plot(obstacles(1,:),obstacles(2,:),'o', 'MarkerEdgeColor','r','MarkerSize',10); 
contour(xx,yy,potential,90);

axis([0 35 0 35]); 
hold off


figure,
plot(P_Goal(1,1),P_Goal(2,1),'+','MarkerSize',10);
hold on
plot(P_Goal(1,1),P_Goal(2,1),'o','MarkerSize',22);

xlabel('x'); 
ylabel('y'); 
plot(obstacles(1,:),obstacles(2,:),'o','MarkerSize',22); 
plot(obstacles(1,:),obstacles(2,:),'o', 'MarkerEdgeColor','r','MarkerSize',10); 
 contour(xx,yy,op,20);
 axis([0 35 0 35]); 
hold off

figure, 
plot(P_Goal(1,1),P_Goal(2,1),'+','MarkerSize',10);
hold on
plot(P_Goal(1,1),P_Goal(2,1),'o','MarkerSize',22);

xlabel('x'); 
ylabel('y'); 
plot(obstacles(1,:),obstacles(2,:),'o','MarkerSize',22); 
plot(obstacles(1,:),obstacles(2,:),'o', 'MarkerEdgeColor','r','MarkerSize',10); 
 contour(xx,yy,gp,50);
 axis([0 35 0 35]); 
hold off

% Robot运动路径仿真过程
P_Goal_1 = P_Goal(:,1);

for k=1:Nsteps
    % 设定运动边界
    P_Ro(:,k) = min(P_Ro(:,k),xmax); 
    P_Ro(:,k) = max(P_Ro(:,k),xmin); 
    for m=1:Ns 
        xs(:,m) = [P_Ro(1,k)+r*cos(theta(m,1)); P_Ro(2,k)+r*sin(theta(m,1))];      
        % 计算是否进入障碍物的作用范围
        for t = 1:obNum
        distro(:,t) = xs(:,m) - obstacles(:,t);
        end
        sum1 = sum(distro.^2);
        if min(sum1) < 1.69 % 设定障碍物作用距离的平方值
           Jo(m,1) = 100;
        else
           Jo(m,1) = obstaclefunction(xs(:,m),w1,obstacles); 
        end
        Jg(m,1) = goalfunction(xs(:,m),P_Goal_1,w2); 
        J(m,1)= Jg(m,1) - Jo(m,1);
    end 
    

    [val,num] = max(J); 

    distrt = P_Ro(:,k) - P_Goal_1; 
    if sum(distrt.^2) > 0.5        
        P_Ro(:,k+1) = [P_Ro(1,k)+lambda*cos(theta(num,1)); P_Ro(2,k)+lambda*sin(theta(num,1))]; 
    else
        break;
    end

    P_RoA = P_Ro(1,k+1);
    P_RoB = P_Ro(2,k+1);
    P_RoC = P_Ro(1,1:k+1);
    P_RoD = P_Ro(2,1:k+1);
    Deltalambda=0.1*lambda*(2*rand-1); 
    Deltatheta=2*pi*(2*rand-1); 
    P_Ro(:,k+1)=[P_Ro(1,k+1)+Deltalambda*cos(theta(num,1)+Deltatheta); P_Ro(2,k+1)+Deltalambda*sin(theta(num,1)+Deltatheta)]; 
end

figure;
% 障碍物势场
plot(obstacles(1,:),obstacles(2,:),'o','MarkerSize',22);
hold on;
% 机器人运动路径
plot(P_Ro(1,1:k) ,P_Ro(2,1:k) ,'r-'); 
hold on
plot(P_Ro(1,1),P_Ro(2,1),'s', 'MarkerFaceColor','g','MarkerSize',10);
plot(P_Goal(1,1),P_Goal(2,1),'+','MarkerSize',10);
plot(P_Goal(1,1),P_Goal(2,1),'o','MarkerSize',22);
axis([0 35 0 35]); 
xlabel('x'); 
ylabel('y'); 
title('Robot''s path with obstacles'); 
hold off
