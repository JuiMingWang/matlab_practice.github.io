close all;clc;clear;
%參數設定
green = [0.4660 0.6740 0.1880];
%----球參數
x = 0; y = 0; %球心的初始位置
R = 2; %球的半徑(公尺)
ball_volume = (4/3)*pi*(R^3);%球的體積
m = ball_volume*(2.5e3); %球體質量(密度參考石頭的:2.5e3 kg/m^3)

%----運動參數
a_x = 0; a_y = 0; 
a_total = a_x+a_y; %初始加速度
v_x = 0; v_y = 0; 
v_total = v_x+v_y; %初始速度
angle = 0; angular_v = 0;%初始轉動條件

%----物理參數
t = 0; dt = 0;%初始時間&時間間格
air_density = 1.225;%空氣密度(kg/m^3)
C_D = 0.47;% 球體的阻力係數
A = 0.5*(4*pi*R*R);%球體的參考面積(運動時受的阻力面積)
mu_k_1 = 0.45; %泥土(濕)動摩擦力
mu_k_2 = 0.55; %柏油路(濕)動摩擦力

theta = deg2rad(45); %斜坡角度
L = 50; %斜坡長度(公尺)
H = L*sin(theta); %斜坡高度 = L*sin(theta)
d_1 = L*cos(theta); %斜坡的底 = L*cos(theta)
d_2 = 50; %水平地面長度(公尺)

while y-R >= (-1)*H-R*cos(theta)%球還沒有碰到地面的條件
    tic;
    %合力
    F = -FG(m)*sin(theta) + f_k(mu_k_1, N(theta,FG(m)) ) - f_d(air_density, v_x,v_y, C_D, A);%斜面上受到重力分力-動摩擦力-空氣阻力

    %速度
    a_total = F/m;%合加速度 = 斜面上的合力/質量
    a_x = a_total*cos(theta); a_y = a_total*sin(theta);%加速度的x與y分量
    v_x = v_x + a_x*dt; v_y = v_y + a_y*dt;%速度的x與y分量
    x = x + v_x*dt; y = y + v_y*dt;%更新位置
    %角速度
    angular_v = angular_v + angular_a( f_k(mu_k_1, N(theta,FG(m))), m, R, 90)*dt;%更新角度速度
    angle = angle + angular_v*dt;%更新已轉角度

    %畫球
    plotCircle(x,y, R, angle);
    %斜坡
    line([R*sin(theta)-L*cos(theta), R*sin(theta)], ...
        [-1*R*cos(theta)-L*sin(theta),-1*R*cos(theta)],'LineWidth', 1);
    %地面
    line([-d_1+R*sin(theta),-d_1+R*sin(theta)-d_2], ...
        [-H-R*cos(theta),-H-R*cos(theta)],'LineWidth', 1);
    title('Falling Rock Animation');
    axis auto;
    dt = toc;%計算程式運行時間來作為dt
    hold off
    pause(dt)    
end

dt = 0;
collision_time = 6;%碰撞次數
collistion_count = 1;
%地面上的運動--碰撞
while  collistion_count <= collision_time
    tic;
    %處理y方向運動
    if y-(-R*cos(theta)-L*sin(theta)) <= R & v_y <0 %發生碰撞
    collistion_count = collistion_count+1;
    v = collision(v_y,0,m,1e10);%y方向碰撞,地板質量=很大 速度=0
    v_y = v(1)%取物體1(球)的速度
    else
        v_y = v_y;
    end
    if v_y >= 0%受空氣阻力和重力
        a_y = f_d(air_density, 0,v_y, C_D, A)/m - 9.8;
    else
        a_y = -f_d(air_density, 0,v_y, C_D, A)/m - 9.8;%空氣阻力與v_y方向相反,所以加負號 
    end
    v_y = v_y + a_y*dt;
    y = y + v_y*dt;%y方向位置更新

    %處理x方向運動
    if y-(-R*cos(theta)-L*sin(theta)) <= R & v_y <0 %發生碰撞
        %動摩擦力+空氣阻力 影響x方向加速度
        a_x = (f_k(mu_k_2, FG(m)) - f_d(air_density,v_x,0,C_D,A))/m;
        %動摩擦力影響力矩
        angular_v = angular_v + angular_a(f_k(mu_k_2, FG(m)), m, R,90)*dt;
    else%沒發生碰撞
        a_x = -f_d(air_density,v_x,0,C_D,A)/m;%x方向加速度僅受空氣阻力影響
        angular_v = angular_v;%沒有力矩,所以角速度不變
    end
    v_x = v_x + a_x*dt;
    x = x + v_x*dt;%更新x位置
    angle = angle + angular_v*dt;%更新已轉角度

    %畫圓
    plotCircle(x,y, R, angle);
    %斜坡
    line([R*sin(theta)-L*cos(theta), R*sin(theta)], ...
        [-1*R*cos(theta)-L*sin(theta),-1*R*cos(theta)],'LineWidth', 1);
    %地面
    d_2 = d_2 - v_x*dt;%讓地面一直延伸
    line([-d_1+R*sin(theta),-d_1+R*sin(theta)-d_2], ...
        [-H-R*cos(theta),-H-R*cos(theta)],'LineWidth', 1); 
    title('Falling Rock Animation');
    axis auto;

    dt = toc;
    hold off
    disp([v_x,v_y]);%---------------------
    pause(dt)
end

%力的設定
function f_g = FG(mass)%重力
    f_g = mass * 9.8;
end

function N = N(theta, F_g)%正向力，theta = 分力夾角 (degree)
    N = F_g * cos(theta);
end

function f_d = f_d(density,v_x,v_y,C_D,A)%空氣阻力(流體密度,v_x,v_y,阻力係數,參考面積)
    f_d = (-1)*0.5*density*(v_x^2+v_y^2)*C_D*A;
end

function f_k = f_k(coefficient_k,N)%動摩擦力
    f_k = coefficient_k*N;
end

function angular_a = angular_a(F,m,r,theta)%角加速度(力,質量,力臂,角度(degree))
    I = m * r^2;%轉動慣量
    torque = r*F*sin( deg2rad(theta) );%力矩
    angular_a = torque/I;%角加速度與力矩方向相同，所以直接相除
end

%其他function
function distant = d(p1_x, p1_y, p2_x, p2_y)%點到點的距離
    d = sqrt( (p1_x-p2_x)^2 + (p1_y-p2_y)^2 );
end

function plotCircle(a,b, r, angle)%畫圓
    aplha_1 = linspace(0, pi, 20);

    aplha_2 = linspace(pi, 2*pi, 20);
    %上半圈
    x_1 = a+r*cos(aplha_1+angle);
    y_1 = b+r*sin(aplha_1+angle);
    plot(x_1,y_1,'-','Color','r');
    hold on
    %下半圈
    x_2 = a+r*cos(aplha_2+angle);
    y_2 = b+r*sin(aplha_2+angle);
    plot(x_2,y_2,'-','Color','b');
end

function [v_coll_1, v_coll_2] = collision(v1,v2,m1,m2)%1:撞擊物 2:被撞擊物

    v_coll_1 = ((m1-m2)/(m1+m2)).*v1 + ((2*m2)/(m1+m2) ).*v2;
    v_coll_2 = ((2*m1)/(m1+m2) ).*v1 + ((m2-m1)/(m1+m2)).*v2;  
end


