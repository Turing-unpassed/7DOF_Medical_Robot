close all;
clear;

figure; 

th1=0;
th2=-90;
th3=0;
th4=0;
th5=180;
th6=0;
dz7=30;
stp=12;
dtime=0.02;
DHfk_WZW(th1,th2,th3,th4,th5,th6,dz7,0); 

pause;hold off;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Joint1
for th=1:stp:360
   DHfk_WZW(th,th2,th3,th4,th5,th6,dz7,1); 
end
% % for th=165:-stp:-165     %-165
% %    DHfk_WZW(th,th2,th3,th4,th5,dz6,1); 
% % end
% % 
% % for th=-165:stp:0
% %     if th~=0
% %        DHfk_WZW(th,th2,th3,th4,th5,dz6,1); 
% %     else
% %        DHfk_WZW(th,th2,th3,th4,th5,dz6,0);   
% %     end
% % end
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Joint2
for th=0:stp:100
   DHfk_WZW(th1,th-90,th3,th4,th5,th6,dz7,1); 
end
for th=100:-stp:-100
   DHfk_WZW(th1,th-90,th3,th4,th5,th6,dz7,1); 
end
% for th=100:-stp:-100
%    DHfk_WZW(th1,th-90,th,th4,th5,th6,dz7,1); 
% end

for th=-100:stp:0
    if th~=0
       DHfk_WZW(th1,th-90,th3,th4,th5,th6,dz7,1); 
    else
       DHfk_WZW(th1,th-90,th3,th4,th5,th6,dz7,0);   
    end
end
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Joint3
for th=0:stp:70
   DHfk_WZW(th1,th2,th,th4,th5,th6,dz7,1); 
end
for th=70:-stp:-90
   DHfk_WZW(th1,th2,th,th4,th5,th6,dz7,1);
end

for th=-90:stp:0
    if th~=0
       DHfk_WZW(th1,th2,th,th4,th5,th6,dz7,1); 
    else
       DHfk_WZW(th1,th2,th,th4,th5,th6,dz7,0);   
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Joint4
for th=0:stp:180
   DHfk_WZW(th1,th2,th3,th,th5,th6,dz7,1); 
%     drawnow;  
% 	MakeGif('forth.gif',i)  					%зЂвт
% 	hold on  
end
for th=180:-stp:0    %-160
   DHfk_WZW(th1,th2,th3,th,th5,th6,dz7,1); 
end

% for th=-160:stp:0
%     if th~=0
%        DHfk_WZW(th1,th2,th3,th,th5,dz6,1); 
%     else
%        DHfk_WZW(th1,th2,th3,th,th5,dz6,0);   
%     end
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Joint5
for th=0:stp:120
   DHfk_WZW(th1,th2,th3,th4,th+180,th6,dz7,1); 
end
for th=120:-stp:-120   %-120
   DHfk_WZW(th1,th2,th3,th4,th+180,th6,dz7,1); 
end
% 
% for th=-120:stp:0
%     if th~=0
%        DHfk_WZW(th1,th2,th3,th4,th+180,th6,dz7,1); 
%     else
%        DHfk_WZW(th1,th2,th3,th4,th+180,th6,dz7,0);   
%     end
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Joint6
for th=0:stp:180
   DHfk_WZW(th1,th2,th3,th4,th5,th,dz7,1); 
end
for th=180:-stp:0
   DHfk_WZW(th1,th2,th3,th4,th5,th,dz7,1); 
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Joint6
for th=1:stp:200
   DHfk_WZW(th1,th2,th3,th4,th5,th6,th+30,1); 
end
for th=200:-stp:0
   DHfk_WZW(th1,th2,th3,th4,th5,th6,th+30,1); 
end

% for th=-400:stp:400
%     if th~=0
%        DHfk_WZW(th1,th2,th3,th4,th5,th,1); 
%     else
%        DHfk_WZW(th1,th2,th3,th4,th5,th,0);   
%     end
% end



