close all;

clear;

global Link
DH_Table;

th=[0,0,0,0,0,0];
grid on;

for th1=-165:55:165 
    for th2= -110:50:110
        for th3=-90:80:70
             for th4=-160:80:160
                for th5=-120:80:120
                    for th6=0:40:360 
                        for dz7=0:50:200
                            Link(2).th=0*pi+th1*pi/180; 
                            %Link(2).th=0;
                            Link(3).th=-0.5*pi+th2*pi/180;
                            Link(4).th=0*pi+th3*pi/180;
                            Link(5).th=th4*pi/180;
                            Link(6).th=pi+th5*pi/180; 
                            Link(7).th=th6*pi/180;
                            Link(8).dz=dz7+30;
                            %fprintf('%d %d %d %d %d %d\n',[th1,th2,th3,th4,th5,dz6]');
                        for i=1:8
                            Matrix_DH(i);
                        end
                        for i=2:8
                            Link(i).A=Link(i-1).A*Link(i).A;
                            Link(i).p= Link(i).A(:,4); 
                        end
                        grid on;  
                        plot3(Link(8).p(1),Link(8).p(2),Link(8).p(3),'r*');hold on;
                        end
                    end
                 end
             end
        end
    end
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
                        
 

    





