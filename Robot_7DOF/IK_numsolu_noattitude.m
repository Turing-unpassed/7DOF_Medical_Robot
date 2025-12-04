function q=IK_numsolu_noattitude(W,q)%数值解函数
%W为4*4 的期望位置和姿态
global Link
lamda=0.5;%范围（0,1） 0.5

e=zeros(1,6);
%q=zeros(1,7);
%q=[74.2171,-29.0177,34.6849,83.3101,29.3103,10.5616,30]';
pref=W(1:3,4);
Rref=W(1:3,1:3);
ilimit=10000;
count=1;

while true
    %DrawFrame(Rref,pref);      %绘制目标点坐标系
    count = count + 1;
    if count >= ilimit
        fprintf('iteration number %d final err %f \n',count,err);
        break
    end

    P=DHfk_nodraw(q(1),q(2),q(3),q(4),q(5),q(6),q(7));
    p=P(1:3,4);
    perr=pref-p;

    R=P(1:3,1:3);%3*3  当前姿态
%     Rerr=R'*Rref;%计算姿态误差Rerr
%     th=acos((Rerr(1,1)+Rerr(2,2)+Rerr(3,3)-1)/2);
    
%     if Rerr==eye(3)
%         werr=[0 0 0]';
%     else 
%         %%姿态误差：角度误差
%         werr=(th/2*sin(th))*[Rerr(3,2)-Rerr(2,3),Rerr(1,3)-Rerr(3,1), Rerr(2,1)-Rerr(1,2)]';
%     end
    
%     if isequal(R, Rref)
%         werr = 0;
%         else
%             R_err = R' * Rref;
%             Err_th = acos((R_err(1,1) + R_err(2,2) + R_err(3,3) - 1)/2);
%         if Err_th == 0
%             werr = [0,0,0]';
%         else
%              werr = (Err_th / (2 * sin(Err_th))) * [  R_err(3,2)-R_err(2,3),  R_err(1,3)-R_err(3,1),  R_err(2,1)-R_err(1,2)  ]';
%         end
%     end

    e(1:3)=perr(1:3);
    %e(4:6)=werr(1:3);
    e(4:6)=[0;0;0];
    norm(e(1:3))
    e(4:6)
    %err=norm(e(1:3))+norm(e(4:6))
    err=norm(e(1:3))+0

    if err<=3
       fprintf(' iteration number %d final err %f \n',count,err);
      break
    end

    J=Creat_Jacobian(q(1),q(2),q(3),q(4),q(5),q(6),q(7));
    deta_q=lamda*pinv(J)*e';
    q=q+deta_q;
    if(q(7)<=30)
        q(7)=30;
    end
    if(q(5)>=90)
        q(5)=90;
    end
%     if(q(5)<=45)
%         q(5)=45;
%     end
%     if(q(2)<=160)
%         q(2)=180;
%     end
   
%     if(q(2)>=155)
%         q(2)=0;
%     end
    %DHfk_WZW(q(1),q(2),q(3),q(4),q(5),q(6),q(7),1);
end