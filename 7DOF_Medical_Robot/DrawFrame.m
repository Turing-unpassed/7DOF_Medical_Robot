function DrawFrame(T)

axislen=100;

%% 设置局部坐标轴
px=[axislen,0,0,1]';
py=[0,axislen,0,1]';
pz=[0,0,axislen,1]';

px=T*px;
py=T*py;
pz=T*pz;

%% 设置固定参考坐标轴端点
wx=px(1:3);
wy=py(1:3);
wz=pz(1:3);
wo=T(1:3,4);


% %绘制固定参考坐标系
DrawLink(wo,wx,'r',3);
DrawLink(wo,wy,'k',3); 
DrawLink(wo,wz,'g',3);
