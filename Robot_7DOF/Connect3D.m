function Connect3D(p1,p2,option,pt)
%option 输入的是颜色 ‘b’
%pt输入的是线的粗细
h = plot3([p1(1) p2(1)],[p1(2) p2(2)],[p1(3) p2(3)],option);
set(h,'LineWidth',pt)