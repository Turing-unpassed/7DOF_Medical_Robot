clc;close all;clear;

ToDeg = 180/pi;
ToRad = pi/180;

th1=-6.8921;
th2=30.5374;
th3=-92.3178;
th4=0;
th5=62.3102;
th6=6.8540;
dz7=30;
th=[th1,th2,th3,th4,th5,th6,dz7]';
DHfk_WZW(th1,th2,th3,th4,th5,th6,dz7,0);
view(134,12);

pause;
cla;


x1=[500  500  500   500  500 450   400    350    300 300    300    300 300   ];
y1=[0 -50 -100 -150 -200 -150   -100   -50   0 -50   -100   -150 -200        ];
z1=[ 300   300   300  300 300 300 300    300      300 300      300    300 300   ];
num=1;
%r=500;
for i=1:13
	x(num)=x1(i);
    y(num)=y1(i);
    z(num)=z1(i);
    W=[1,0,0,x(num);
       0,1,0,y(num);
       0,0,1,z(num);
       0,0,0,1];
    solu11=IK_numsolu_noattitude(W,th);%数值解
    th=solu11;
    num=num+1;
    plot3(x,y,z,'rX'); 
    DHfk_WZW(solu11(1),solu11(2),solu11(3),solu11(4),solu11(5),solu11(6),solu11(7),1);
end
plot3(x,y,z,'rX'); 

th1=-1.912997319398594;
th2=-24.024008350267493;
th3=19.695693195119910;
th4=-0.205133585820680;
th5=4.861390415390018;
th6=1.698686554855763;
dz7=30;
th=[th1,th2,th3,th4,th5,th6,dz7]';
DHfk_WZW(th1,th2,th3,th4,th5,th6,dz7,0);
x1=[300  350  400   450  500    450   400    350    300 350  400   450  500    450   400    350    300   ];
y1=[50 70 90 110 130      150   170   190 210 230   250   270 290  310 330 350 370       ];
z1=[ 300   300   300  300     300    300 300    300      300 300      300    300 300 300 300 300 300  ];
for i=1:17
	x(num)=x1(i);
    y(num)=y1(i);
    z(num)=z1(i);
    W=[1,0,0,x(num);
       0,1,0,y(num);
       0,0,1,z(num);
       0,0,0,1];
    solu11=IK_numsolu_noattitude(W,th);%数值解
    th=solu11;
    num=num+1;
    plot3(x,y,z,'rX'); 
    if i == 17
        DHfk_WZW(solu11(1),solu11(2),solu11(3),solu11(4),solu11(5),solu11(6),solu11(7),0);
    else
        DHfk_WZW(solu11(1),solu11(2),solu11(3),solu11(4),solu11(5),solu11(6),solu11(7),1);
    end
end
plot3(x,y,z,'rX'); 
