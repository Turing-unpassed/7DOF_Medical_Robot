
close all;
clear;
clc;

Link = MDH_Table_Build();
Link = Transimation_Matrix_Build(Link);

step=1;

th1=0; 
th2=0;
th3=0;
d4=0;
th5=0;
th6=0;
th7=0;

Q=[th1,th2,th3,d4,th5,th6,th7]';
Forward_kinematics(Link,Q,0);
input("press Enter to continue...",'s');

for th=0:step:360
    Q(1)=th;
    T = Forward_kinematics(Link,Q,1);
end

for th=0:step:360
    Q(2)=th;
    Forward_kinematics(Link,Q,1);
end

for th=0:step:360
    Q(3)=th;
    Forward_kinematics(Link,Q,1);
end

for d=0:step:150
    Q(4)=d;
    Forward_kinematics(Link,Q,1);
end

for th=0:step:360
    Q(5)=th;
    Forward_kinematics(Link,Q,1);
end

for th=0:step:360
    Q(6)=th;
    Forward_kinematics(Link,Q,1);
end

for th=0:step:360
    if(th<360)
        Q(7)=th;
        Forward_kinematics(Link,Q,1);
    else
        Q(7)=th;
        Forward_kinematics(Link,Q,0);
    end
end
