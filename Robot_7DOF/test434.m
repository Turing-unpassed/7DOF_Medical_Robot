close all;
clear;
% global Link

ToDeg = 180/pi;
ToRad = pi/180;

th1=48.8603;   %53.8603
th2=142.0557;
th3=-48.2355;
th4=-10.2630;
th5=93.5790;
th6=54.3200;
dz7=30.1738;

grid on;

th=[th1,th2,th3,th4,th5,th6,dz7]';

DHfk_WZW(th(1),th(2),th(3),th(4),th(5),th(6),th(7),0);
pause;
num=1;

%%%直角末端路径规划
i=6;
n=81;
% Position=Tra434(0,300,400,200,0,0,0,0);   
[TH1,V1,ACC1]=Tra434(48.0555,68.24301565,142.2849588,149.0022,0,0,0,0);  %末端在高度变化情况下的轨迹   
[TH2,V2,ACC2]=Tra434(156.9762,118.8942494,148.2991359,181.3628,0,0,0,0); 
[TH3,V3,ACC3]=Tra434(-89.9986,3.374457172,-22.58552991,-103.9431,0,0,0,0); 
[TH4,V4,ACC4]=Tra434(-11.5897,-13.58071226,-8.959626297,-0.5090,0,0,0,0); 
[TH5,V5,ACC5]=Tra434(120.1067,62.06805843,48.50731296,91.2616,0,0,0,0); 
[TH6,V6,ACC6]=Tra434(58.6918,66.82447351,142.3781947,155.3653,0,0,0,0); 
[TH7,V7,ACC7]=Tra434(99.9309,34.75148971,34.38413689,30,0,0,0,0); 


for t=1:81
    if TH5(t)<=40
        TH5(t)=40;
    end
    if t>=81
        A=DHfk_WZW(TH1(t),TH2(t),TH3(t),TH4(t),TH5(t),TH6(t),TH7(t),0);
    else
        A=DHfk_WZW(TH1(t),TH2(t),TH3(t),TH4(t),TH5(t),TH6(t),TH7(t),1);%清除
    end
    p=A(1:3,4);
    x(t)=p(1);
    y(t)=p(2);
    z(t)=p(3);
    plot3(x,y,z,'kX');
end

% k=10;b=-410;
% 
% for t=1:i:n%采集数据个数为n/i
%     x(num)=k*t+b;       %将三维平面转换到二维平面  用直线拟合曲线
%     y(num)=-300;
%     z(num)=Position(t);
%     W=[1,0,0,x(num);
%        0,1,0,y(num);
%        0,0,1,z(num);
%        0,0,0,1];
% 
% plot3(x,y,z,'kX');hold on;%x,y,z为数组
% theta2=IK_numsolu_noattitude(W,th);%theta2为角度制
% th=theta2;
% %plot3(0,k*t+b,Posion(t),'k*');hold on;
% if(t>n-i)
%     DHfk_WZW(theta2(1),theta2(2),theta2(3),theta2(4),theta2(5),theta2(6),theta2(7),0);
% else
%     DHfk_WZW(theta2(1),theta2(2),theta2(3),theta2(4),theta2(5),theta2(6),theta2(7),1);%清除
% end
% %plot3(x,y,z,'kX'); 
% q1(num)=theta2(1);
% q2(num)=theta2(2);
% q3(num)=theta2(3);
% q4(num)=theta2(4);
% q5(num)=theta2(5);
% q6(num)=theta2(6);
% q7(num)=theta2(7);
% num=num+1;
% end

figure(1),hold on;