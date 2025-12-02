function q=IK_num_solu(W)%ÊıÖµ½âº¯Êı

global Link
lamda=0.5;%·¶Î§£¨0,1£© 0.5

e=zeros(1,6);
q=[0,-90,0,0,180,0,0]';
pref=W(1:3,4);
Rref=W(1:3,1:3);
ilimit=2000;

for count=1:ilimit
    DrawFrame(Rref,pref);     
 
    if count >= ilimit
        fprintf('iteration number %d final err %f \n',count,err);
        break
    end

    P=DHfk_nodraw(q(1),q(2),q(3),q(4),q(5),q(6),q(7));
    p=P(1:3,4);
    perr=pref-p;%¼ÆËãÎ»ÖÃÎó²îperr

    R=P(1:3,1:3);
%     Rerr=R'*Rref;%¼ÆËã×ËÌ¬Îó²îRerr
%     th=acos((Rerr(1,1)+Rerr(2,2)+Rerr(3,3)-1)/2);
    
%     if Rerr==eye(3)
%         werr=[0 0 0]';
%     else 
%         %%×ËÌ¬Îó²î£º½Ç¶ÈÎó²î
%         werr=(th/2*sin(th))*[Rerr(3,2)-Rerr(2,3),Rerr(1,3)-Rerr(3,1), Rerr(2,1)-Rerr(1,2)]';
%     end
    
    if isequal(R, Rref)
        werr = 0;
        else
            R_err = R' * Rref;
            Err_th = acos((R_err(1,1) + R_err(2,2) + R_err(3,3) - 1)/2);
        if Err_th == 0
            werr = [0,0,0]';
        else
             werr = (Err_th / (2 * sin(Err_th))) * [  R_err(3,2)-R_err(2,3),  R_err(1,3)-R_err(3,1),  R_err(2,1)-R_err(1,2)  ]';
        end
    end

    e(1:3)=perr(1:3);
    e(4:6)=werr(1:3);
    norm(e(1:3))
    e(4:6)
    err=norm(e(1:3))+norm(e(4:6))


    if err<=1
       fprintf(' iteration number %d final err %f \n',count,err);
      break
    end

    J=Creat_Jacobian(q(1),q(2),q(3),q(4),q(5),q(6),q(7));
    deta_q=lamda*pinv(J)*e';
    q=q+deta_q;
    if(q(7)<=30)
        q(7)=30;
    end

   DHfk_WZW(q(1),q(2),q(3),q(4),q(5),q(6),q(7),1);
end

    