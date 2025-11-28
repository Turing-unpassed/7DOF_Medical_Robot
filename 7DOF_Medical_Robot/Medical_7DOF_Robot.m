close all;
clear;
clc;

ToRad = pi/180;
Link = MDH_Table_Build();
Link = Transimation_Matrix_Build(Link);
ToDegree = 180/pi;
th1=106.2602; 
th2=106.2602;
th3=0;
d4=0;
th5=0;
th6=253.7398;
th7=253.7398; 

Q=[th1,th2,th3,d4,th5,th6,th7]';
T = Forward_kinematics(Link,Q,0)

q = Geometric_Inverse_Kinematics(Link,T)