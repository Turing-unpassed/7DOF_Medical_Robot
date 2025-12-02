function q = IK_NumSolution(Aref)   %数值解
%UR5 IK
%Input:position and orientation   位姿
%Output:angle

global Link
q = zeros(6,1);    
e = zeros(6,1);
MaxIter = 1000;
countIter = 0;
err = Inf;
  
Rref=Aref(1:3,1:3);  %输入的位姿矩阵 1-3行 1到3列  这里意思是获得期望R
J = zeros(6);        %雅克比矩阵初始化
P = zeros(3,7);      %3行7列的零矩阵

while true
    %Maximum number of iterations
    countIter = countIter + 1;
    if countIter >= MaxIter
        fprintf('ikine: iteration number %d final err %f \n',countIter,err);  %打印最后的迭代次数和误差
        break
    end

	%calulate err
    Acur=DHfk_nodraw(q(1),q(2),q(3),q(4),q(5),q(6));
    Aerr=Aref-Acur;
    e(1:3)=Aerr(1:3,4);%Perr   将Terr中第四列的123行赋给e的123行 表示位置误差
    Rcur=Acur(1:3,1:3);
    Rerr=(Rcur)' * Rref;%'
    th = acos((Rerr(1,1)+Rerr(2,2)+Rerr(3,3)-1.0)/2.0);
    if th==0
    	Werr = zeros(3,1);
    else
    	Werr = [Rerr(3,2)-Rerr(2,3),Rerr(1,3)-Rerr(3,1),Rerr(2,1)-Rerr(1,2)]';%'
        Werr = (th/(2*sin(th)))*Werr;
    end
    e(4:6)=Werr;%Rerr

    %tolerance
    np = norm(e(1:3));  %|perr|
    nw = norm(e(4:6));  %|werr|
    err = np + nw;       %err   
    if  err <= 10             %1e-6
        fprintf('ikine: iteration number %d final err %f \n',countIter,err);
        break
    end

    %calulate correction
    J = Creat_Jacobian(q(1),q(2),q(3),q(4),q(5),q(6));

    dq = pinv(J) * e;%'    pinv伪逆
    dq = 1/2*dq;

    %update
    q = q + dq;
end