close all;
clear;
clc;

ToRad = pi/180;
Link = MDH_Table_Build();
Link = Transimation_Matrix_Build(Link);
ToDegree = 180/pi;
th1=225.0000; 
th2=138.2415;
th3=3.8072;
d4=0;
th5=270.0000;
th6=127.9514;
th7=135.0000; 

Q=[th1,th2,th3,d4,th5,th6,th7];

% T = Forward_kinematics(Link,Q,0)

WorkSpace_Create(50000);

Singularity_Analyse(Link,50000);





