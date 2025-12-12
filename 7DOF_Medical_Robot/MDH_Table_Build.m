function Link = MDH_Table_Build()

%根据机械臂结构建立MDH表
%   Detailed explanation goes here


link    = struct('name','joint', 'theta',  0, 'd', 0, 'a', 0, 'alpha',0, 'type', 'R' );     
link(1) = struct('name','Base' , 'theta',  0, 'd', 0, 'a', 0, 'alpha',0 , 'type', 'R' );        %BASE to 1
link(2) = struct('name','J1' , 'theta',   0, 'd', 153.7, 'a', 0, 'alpha',90, 'type', 'R' );       %1 TO 2  
link(3) = struct('name','J2' , 'theta',  0 , 'd', 0, 'a', 250.35, 'alpha',0, 'type', 'R' );    %2 TO 3  
link(4) = struct('name','J3' , 'theta',  0 , 'd', 0, 'a', 0, 'alpha',-90, 'type', 'R' );          %3 TO 4    
link(5) = struct('name','J4' , 'theta',  90 , 'd', 140, 'a', 0, 'alpha',-90, 'type', 'P' );          %4 TO 5 
link(6) = struct('name','J5' , 'theta',  0 , 'd', 84.8, 'a', 0, 'alpha',90, 'type', 'R' );          %5 TO 6
link(7) = struct('name','J6' , 'theta',  0 , 'd', 0, 'a', 0, 'alpha',-90, 'type', 'R' );          %6 TO 7
link(8) = struct('name','J7',  'theta',  0 , 'd',  0,  'a', 0, 'alpha', 0, 'type', 'R');     %7 TO E
link(9) = struct('name','tool',  'theta',  0 , 'd',  80,  'a', 0, 'alpha', 0, 'type', 'R');  %end point to tool

Link = link;

end