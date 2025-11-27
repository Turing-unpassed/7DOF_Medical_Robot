function Q = Piper_Inverse_Kinematics(link,p)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
link =Transimation_Matrix_Build(link);

q4= symvar(link(5).T);
link(5).T = subs(link(5).T,q4,0);
link(2).T*link(3).T*link(4).T*link(5).T*link(6).T(:,4) == p;
F = link(4).T*link(5).T*link(6).T(:,4);
G = link(3).T*F;

end