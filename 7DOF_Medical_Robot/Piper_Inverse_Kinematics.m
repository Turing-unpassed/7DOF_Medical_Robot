function Q = Piper_Inverse_Kinematics(link,T07)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
syms th1 th2 th3 
p = T07(:,4);

d4=0;
d = 150+d4;

q1 = symvar(link(2).T);
q2 = symvar(link(3).T);
q3 = symvar(link(4).T);
q4= symvar(link(5).T);

link(2).T = subs(link(2).T,q1,th1);
link(3).T = subs(link(3).T,q2,th2);
link(4).T = subs(link(4).T,q3,th3);
link(5).T = subs(link(5).T,q4,d);

equals = link(1).T*link(2).T*link(3).T*link(4).T*link(5).T*link(6).T(:,4)==p;
% F = link(4).T*link(5).T*link(6).T(:,4);
% G = link(3).T*F;
S = zeros(3,4);
for i = 1:4
    s = vpasolve(equals,[th1,th2,th3],[-180,180;-180,180;-180,180],"Random",true);
    S(1,i) = s.th1;
    S(2,i) = s.th2;
    S(3,i) = s.th3;
end
S
end
