clear all; clc
X_length = 700;
Y_length = 200;
interval = 2;

x = (0:interval:X_length); y = (0:interval:Y_length);
M = length(x) - 1;%x的分段數
N = length(y) - 1;%y的分段數
v1 = zeros(N+1,M+1);%
%設置邊界條件
v1(1,:) = 30;  %v1(x,0) = 30
v1(:,1) = 20;  %v1(0,y) = 20
v1(N+1,:) = 10;%v1(x,Y_length) = 10
v1(:,M+1) = 5 ;%v1(X_length,y) = 5
%三角形
theta = deg2rad(5);%角度設置
m1 = -tan(theta/2);
L1_y = m1.*(x-250) + 100;

m2 = tan(theta/2);
L2_y = m2.*(x-250) + 100;
%圓形
R = 65;
d = 300;
center_x = 250+d;
center_y = 100;

for i=1:M+1
    for j=1:N+1
        if y(j)<=L1_y(i) & y(j)>=L2_y(i) & x(i)>=75
            v1(j,i)=100;
        elseif p2p_distant(x(i),y(j),center_x,center_y) <= R
            v1(j,i)=0;
        end
    end
end

v2 = v1;
mark = 1; t=0; m=1;
k = 0;%迭代次數
while (m>1e-5)
    m=0;
    k = k+1;
    for i=2:M
        for j=2:N
            %先確認要計算的點是否在邊界條件中
            if y(j)<=L1_y(i) & y(j)>=L2_y(i) & x(i)>=75
                continue               
            elseif p2p_distant(x(i),y(j),center_x,center_y) <= R
                continue  
            else
            %如果不在，再算該點的差分
                v2(j,i)=( v1(j,i+1) + v1(j+1,i) + v1(j-1,i) + v1(j,i-1) )/4;
                t = (v2(j,i)-v1(j,i));
                if t>m
                    m=t;
                end
            end                
        end
    end
v1 = v2;
end
%畫電位分布
surf(v1);
view(2);
colorbar;
%畫電場分布
% [Ex,Ey] = gradient(v1);
% quiver(x,y,-Ex,-Ey);
disp("done");
function p2p = p2p_distant(x1,y1,x2,y2)
    p2p = sqrt( (x1-x2)^2 + (y1-y2)^2 );
end
