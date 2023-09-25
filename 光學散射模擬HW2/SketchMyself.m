hold on
theta = [0:pi/20:2*pi];%頭
r1 = 5;
x1 = r1.*cos(theta);
y1 = r1.*sin(theta);
plot(x1,y1,'bl')
r2 = 1.5;%右眼
x2 = r2.*cos(theta);
y2 = r2.*sin(theta);
plot(x2+2,y2,'bl')

r3 = -1.5%左眼
x3 = r3.*cos(theta);
y3 = r3.*sin(theta);
plot(x2-2,y2,'bl')

line([-1,1],[-4,-4])%y嘴吧
p = nsidedpoly(3,'Center',[0,-2],'SideLength',1.5);%鼻子
plot(p);

for i = [1:25]
    plot(theta,sin(theta)+i*0.2,'k')
    plot(-theta,sin(theta)+i*0.2,'k')
end
hold off