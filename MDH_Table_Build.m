function Link = MDH_Table_Build()

%根据机械臂结构建立MDH表
%   Detailed explanation goes here


link    = struct('name','joint', 'theta',  0, 'd', 0, 'a', 0, 'alpha',0 );     
link(1) = struct('name','Base' , 'theta',  0, 'd', 0, 'a', 0, 'alpha',0 );        %BASE to 1
link(2) = struct('name','J1' , 'theta',   0, 'd', 300, 'a', 0, 'alpha',90 );       %1 TO 2  
link(3) = struct('name','J2' , 'theta',  0 , 'd', 150, 'a', 200, 'alpha',0 );    %2 TO 3  
link(4) = struct('name','J3' , 'theta',  0 , 'd', 0, 'a', 0, 'alpha',-90 );          %3 TO 4    
link(5) = struct('name','J4' , 'theta',  0 , 'd', 150, 'a', 0, 'alpha',0 );          %4 TO 5 
link(6) = struct('name','J5' , 'theta',  0 , 'd', 0, 'a', 0, 'alpha',90 );          %5 TO 6
link(7) = struct('name','J6' , 'theta',  0 , 'd', 0, 'a', 0, 'alpha',-90 );          %6 TO 7
link(8) = struct('name','J7',  'theta',  0 , 'd',  0,  'a', 0, 'alpha', 0);     %7 TO E
link(9) = struct('name','tool',  'theta',  0 , 'd',  80,  'a', 0, 'alpha', 0);  %end point to tool

Link = link;

end