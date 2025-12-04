function A=DHfk_WZW(th1,th2,th3,th4,th5,th6,dz7,fcla)
% close all

global Link

DH_Table;        %构建DH表
radius    = 25;  %25
len       = 60;  %60
joint_col = 0;


%plot3(0,0,0,'ro'); 


 
 Link(2).th=th1*pi/180;
 Link(3).th=th2*pi/180;
 Link(4).th=th3*pi/180;
 Link(5).th=th4*pi/180;
 Link(6).th=th5*pi/180;
 Link(7).th=th6*pi/180;
 Link(8).dz=dz7+30;

p0=[0,0,0]';


for i=1:8
Matrix_DH(i);
end


for i=2:8

      Link(i).A=Link(i-1).A*Link(i).A;
      Link(i).p= Link(i).A(:,4);
      Link(i).n= Link(i).A(:,1);
      Link(i).o= Link(i).A(:,2);
      Link(i).a= Link(i).A(:,3);
      Link(i).R=[Link(i).n(1:3),Link(i).o(1:3),Link(i).a(1:3)];
      Connect3D(Link(i-1).p,Link(i).p,'b',2); hold on;
      plot3(Link(i).p(1),Link(i).p(2),Link(i).p(3),'rx');hold on;
     
      if i<=8
          DrawCylinder(Link(i-1).p, Link(i-1).R * Link(i).az, radius,len, joint_col); hold on;
      end 
     % DrawFrame(Link(i).R, [Link(i).p(1),Link(i).p(2),Link(i).p(3)]' );hold on;%绘制机械臂末端坐标系,可以去掉
end
DrawFrame(Link(8).R, [Link(8).p(1),Link(8).p(2),Link(8).p(3)]' );hold on;    %绘制机械臂末端坐标系
% view(125,52);
% set (gcf,'Position',[650,100,700,600])
%axis([-700,700,-700,700,-400,1000]);
axis([-1000,1000,-1000,1000,-1000,1000]);
xlabel('x');
ylabel('y'); 
zlabel('z');
grid on;
drawnow;

A=Link(8).A;
%判断是否清空坐标系
if(fcla)
    cla;
end
hold on
axis([-1000,1000,-1000,1000,-1000,1000]);
xlabel('x');
ylabel('y');
zlabel('z');

x=[50  50  50   50  50 50   450    450    50 450    450    450 450 50   50 450 450 450 450];
y=[-200 -700 -700 -200 -200 -200   -200   -200   -200 -200   -200   -700 -700 -700 -700 -700 -700 -700 -200];
z=[ 0   0   300  300 0 300 300    0      0 0      300    300 0   0   300 300 0   0   0];
plot3(x,y,z,'b');
hold on;

%设置矩形左下角的顶点坐标
ax = -650;
ay = -800;

%设置矩形长宽
l = 1500;
w = 500;
x = [ax,ax+w,ax+w,ax,ax];
y = [ay,ay,ay+l,ay+l,ay];

%绘图
line(x,y)





