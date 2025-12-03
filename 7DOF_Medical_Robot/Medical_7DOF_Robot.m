close all;
clear;
clc;

ToRad = pi/180;
Link = MDH_Table_Build();
Link = Transimation_Matrix_Build(Link);
ToDegree = 180/pi;
th1=45.0000; 
th2=35.3079;
th3=243.3542;
d4=300.0000;
th5=89.6729;
th6=92.5486;
th7=96.0638; 

Q=[th1,th2,th3,d4,th5,th6,th7];

T = Forward_kinematics(Link,Q,0)

T = [0,0,-1,450;
     1,0,0,450;
     0,-1,0,450;
     0,0,0,1];
Geometric_Inverse_Kinematics(Link,T)



