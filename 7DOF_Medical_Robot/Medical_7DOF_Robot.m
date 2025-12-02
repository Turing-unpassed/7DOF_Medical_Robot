close all;
clear;
clc;

ToRad = pi/180;
Link = MDH_Table_Build();
Link = Transimation_Matrix_Build(Link);
ToDegree = 180/pi;
th1=0; 
th2= 239.0519;
th3=77.6612;
d4=291.7000;
th5=180;
th6=316.7132;
th7=180; 

Q=[th1,th2,th3,d4,th5,th6,th7];

% T = Forward_kinematics(Link,Q,0)

T = [1,0,0,151;
     0,1,0,151;
     0,0,1,151;
     0,0,0,1];
Geometric_Inverse_Kinematics(Link,T)


