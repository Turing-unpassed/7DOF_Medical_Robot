
function THETA=IK_AnalySolution(P,R)

ToDeg = 180/pi;
ToRad = pi/180;

d=[320,0,-50,300,0,30];
a=[0,200,25,0,0,0];
alpha=[-90*ToRad,0*ToRad,-90*ToRad,90*ToRad,90*ToRad,0*ToRad];

nx=R(1,1);ny=R(2,1);nz=R(3,1);
ox=R(1,2);oy=R(2,2);oz=R(3,2);
ax=R(1,3);ay=R(2,3);az=R(3,3);
px=P(1);py=P(2);pz=P(3);



theta11=NaN;theta12=NaN;
theta21=NaN;theta22=NaN;theta23=NaN;theta24=NaN;theta25=NaN;theta26=NaN;theta27=NaN;theta28=NaN;
theta31=NaN;theta32=NaN;theta33=NaN;theta34=NaN;theta35=NaN;theta36=NaN;theta37=NaN;theta38=NaN;
theta41=NaN;theta42=NaN;theta43=NaN;theta44=NaN;theta45=NaN;theta46=NaN;theta47=NaN;theta48=NaN;
theta51=NaN;theta52=NaN;theta53=NaN;theta54=NaN;
theta61=NaN;theta62=NaN;theta63=NaN;theta64=NaN;

%求解关节角1 前提(m^2+n^2-d4^2>=0)
m=d(6)*ay-py;  n=ax*d(6)-px;
condition1=m^2+n^2-d(4)^2;
if condition1>0
    
    theta11=atan2(m,n)-atan2(d(4),sqrt(m^2+n^2-(d(4))^2));
    theta12=atan2(m,n)-atan2(d(4),-sqrt(m^2+n^2-(d(4))^2));

  

    condition21=ax*sin(theta11)-ay*cos(theta11);
    condition22=ax*sin(theta12)-ay*cos(theta12);
    if condition21<=1 
        theta51=acos(ax*sin(theta11)-ay*cos(theta11));
        theta52=-acos(ax*sin(theta11)-ay*cos(theta11)) ;

        condition31=sin(theta51);
        condition32=sin(theta52);
        if condition31 ~=0 
            m1=nx*sin(theta11)-ny*cos(theta11);
            n1=ox*sin(theta11)-oy*cos(theta11);
            theta61=atan2(m1,n1)-atan2(sin(theta51),0);
            
            m2=d(5)*(sin(theta61)*(nx*cos(theta11)+ny*sin(theta11))+cos(theta61)*(ox*cos(theta11)+oy*sin(theta11))) -d(6)*(ax*cos(theta11)+ay*sin(theta11))+px*cos(theta11)+py*sin(theta11);
            n2=pz-d(1)-az*d(6)+d(5)*(oz*cos(theta61)+nz*sin(theta61));
            condition41=m2^2+n2^2-(a(2)+a(3))^2;
            if condition41<0
                theta31=acos((m2^2+n2^2-(a2)^2-(a3)^2)/(2*a2*a3));
                theta32=-acos((m2^2+n2^2-(a2)^2-(a3)^2)/(2*a2*a3));
                
                m_s2=m2;
                n_s2=n2;
                
                s21=((a3*cos(theta31)+a2)*n_s2-a3*sin(theta31)*m_s2)/ ((a2)^2+(a3)^2+2*a2*a3*cos(theta31));
                c21=(m_s2+a3*sin(theta31)*s21)/(a3*cos(theta31)+a2);
                
                s22=((a3*cos(theta32)+a2)*n_s2-a3*sin(theta32)*m_s2)/ ((a2)^2+(a3)^2+2*a2*a3*cos(theta32));
                c22=(m_s2+a3*sin(theta32)*s22)/(a3*cos(theta32)+a2);
                theta21=atan2(s21,c21);
                theta22=atan2(s22,c22);
                
                theta41=atan2(-sin(theta61)*(nx*cos(theta11)+ny*sin(theta11))-cos(theta61)* ...
                    (ox*cos(theta11)+oy*sin(theta11)),oz*cos(theta61)+nz*sin(theta61))-theta21-theta31;
                
                theta42=atan2(-sin(theta61)*(nx*cos(theta11)+ny*sin(theta11))-cos(theta61)* ...
                    (ox*cos(theta11)+oy*sin(theta11)),oz*cos(theta61)+nz*sin(theta61))-theta22-theta32;
            end
        end
        
        if condition32 ~=0 
            m1=nx*sin(theta11)-ny*cos(theta11);
            n1=ox*sin(theta11)-oy*cos(theta11);
            theta62=atan2(m1,n1)-atan2(sin(theta52),0);
            
            m2=d(5)*(sin(theta62)*(nx*cos(theta11)+ny*sin(theta11))+cos(theta62)*(ox*cos(theta11)+oy*sin(theta11))) -d(6)*(ax*cos(theta11)+ay*sin(theta11))+px*cos(theta11)+py*sin(theta11);
            n2=pz-d(1)-az*d(6)+d(5)*(oz*cos(theta62)+nz*sin(theta62));
            condition42=m2^2+n2^2-(a(2)+a(3))^2;
            if condition42<0
                theta33=acos((m2^2+n2^2-(a2)^2-(a3)^2)/(2*a2*a3));
                theta34=-acos((m2^2+n2^2-(a2)^2-(a3)^2)/(2*a2*a3));
                
                m_s2=m2;
                n_s2=n2;
                
                s23=((a3*cos(theta33)+a2)*n_s2-a3*sin(theta33)*m_s2)/ ((a2)^2+(a3)^2+2*a2*a3*cos(theta33));
                c23=(m_s2+a3*sin(theta33)*s23)/(a3*cos(theta33)+a2);
                
                s24=((a3*cos(theta34)+a2)*n_s2-a3*sin(theta34)*m_s2)/ ((a2)^2+(a3)^2+2*a2*a3*cos(theta34));
                c24=(m_s2+a3*sin(theta34)*s24)/(a3*cos(theta34)+a2);
                theta23=atan2(s23,c23);
                theta24=atan2(s24,c24);
                
                theta43=atan2(-sin(theta62)*(nx*cos(theta11)+ny*sin(theta11))-cos(theta62)* ...
                    (ox*cos(theta11)+oy*sin(theta11)),oz*cos(theta62)+nz*sin(theta62))-theta23-theta33;
                
                theta44=atan2(-sin(theta62)*(nx*cos(theta11)+ny*sin(theta11))-cos(theta62)* ...
                    (ox*cos(theta11)+oy*sin(theta11)),oz*cos(theta62)+nz*sin(theta62))-theta24-theta34;
            end
        end
    end
    
    
    
    if condition22<=1 
        theta53=acos(ax*sin(theta12)-ay*cos(theta12));
        theta54=-acos(ax*sin(theta12)-ay*cos(theta12)) ;
        
    
        condition33=sin(theta53);
        condition34=sin(theta54);
        
        
        
        if condition33 ~=0  
            m1=nx*sin(theta12)-ny*cos(theta12);
            n1=ox*sin(theta12)-oy*cos(theta12);
            theta63=atan2(m1,n1)-atan2(sin(theta53),0);
            
            m2=d(5)*(sin(theta63)*(nx*cos(theta12)+ny*sin(theta12))+cos(theta63)*(ox*cos(theta12)+oy*sin(theta12))) -d(6)*(ax*cos(theta12)+ay*sin(theta12))+px*cos(theta12)+py*sin(theta12);
            n2=pz-d(1)-az*d(6)+d(5)*(oz*cos(theta63)+nz*sin(theta63));
            condition43=m2^2+n2^2-(a(2)+a(3))^2;
            if condition43<0
                theta35=acos((m2^2+n2^2-(a2)^2-(a3)^2)/(2*a2*a3));
                theta36=-acos((m2^2+n2^2-(a2)^2-(a3)^2)/(2*a2*a3));
                
                m_s2=m2;
                n_s2=n2;
                
                s25=((a3*cos(theta35)+a2)*n_s2-a3*sin(theta35)*m_s2)/ ((a2)^2+(a3)^2+2*a2*a3*cos(theta35));
                c25=(m_s2+a3*sin(theta35)*s25)/(a3*cos(theta35)+a2);
                
                s26=((a3*cos(theta36)+a2)*n_s2-a3*sin(theta36)*m_s2)/ ((a2)^2+(a3)^2+2*a2*a3*cos(theta36));
                c26=(m_s2+a3*sin(theta36)*s26)/(a3*cos(theta36)+a2);
                theta25=atan2(s25,c25);
                theta26=atan2(s26,c26);
                
                theta45=atan2(-sin(theta63)*(nx*cos(theta12)+ny*sin(theta12))-cos(theta63)* ...
                    (ox*cos(theta12)+oy*sin(theta12)),oz*cos(theta63)+nz*sin(theta63))-theta25-theta35;
                
                theta46=atan2(-sin(theta63)*(nx*cos(theta12)+ny*sin(theta12))-cos(theta63)* ...
                    (ox*cos(theta12)+oy*sin(theta12)),oz*cos(theta63)+nz*sin(theta63))-theta26-theta36;
            end
        end
        
        
        if condition34 ~=0    %不等于0
            m1=nx*sin(theta12)-ny*cos(theta12);
            n1=ox*sin(theta12)-oy*cos(theta12);
            theta64=atan2(m1,n1)-atan2(sin(theta54),0);
            
            m2=d(5)*(sin(theta64)*(nx*cos(theta12)+ny*sin(theta12))+cos(theta64)*(ox*cos(theta12)+oy*sin(theta12))) -d(6)*(ax*cos(theta12)+ay*sin(theta12))+px*cos(theta12)+py*sin(theta12);
            n2=pz-d(1)-az*d(6)+d(5)*(oz*cos(theta64)+nz*sin(theta64));
            condition44=m2^2+n2^2-(a(2)+a(3))^2;
            if condition44<0
                theta37=acos((m2^2+n2^2-(a2)^2-(a3)^2)/(2*a2*a3));
                theta38=-acos((m2^2+n2^2-(a2)^2-(a3)^2)/(2*a2*a3));
                
                m_s2=m2;
                n_s2=n2;
                
                s27=((a3*cos(theta37)+a2)*n_s2-a3*sin(theta37)*m_s2)/ ((a2)^2+(a3)^2+2*a2*a3*cos(theta37));
                c27=(m_s2+a3*sin(theta37)*s27)/(a3*cos(theta37)+a2);
                
                s28=((a3*cos(theta38)+a2)*n_s2-a3*sin(theta38)*m_s2)/ ((a2)^2+(a3)^2+2*a2*a3*cos(theta38));
                c28=(m_s2+a3*sin(theta38)*s28)/(a3*cos(theta38)+a2);
                theta27=atan2(s27,c27);
                theta28=atan2(s28,c28);
                
                theta47=atan2(-sin(theta64)*(nx*cos(theta12)+ny*sin(theta12))-cos(theta64)* ...
                    (ox*cos(theta12)+oy*sin(theta12)),oz*cos(theta64)+nz*sin(theta64))-theta27-theta37;
                
                theta48=atan2(-sin(theta64)*(nx*cos(theta12)+ny*sin(theta12))-cos(theta64)* ...
                    (ox*cos(theta12)+oy*sin(theta12)),oz*cos(theta64)+nz*sin(theta64))-theta28-theta38;
            end
        end
    end
end

YI=[theta11,theta21,theta31,theta41,theta51,theta61];
ER=[theta11,theta22,theta32,theta42,theta51,theta61];
SAN=[theta11,theta23,theta33,theta43,theta52,theta62];
SI=[theta11,theta24,theta34,theta44,theta52,theta62];
WU=[theta12,theta25,theta35,theta45,theta53,theta63];
LIU=[theta12,theta26,theta36,theta46,theta53,theta63];
QI=[theta12,theta27,theta37,theta47,theta54,theta64];
BA=[theta12,theta28,theta38,theta48,theta54,theta64];
THETA=[YI;ER;SAN;SI;WU;LIU;QI;BA];
for i=1:1:8
    for j=1:1:6
        THETA(i,j)=Lim_pi(THETA(i,j));
    end
end

end
