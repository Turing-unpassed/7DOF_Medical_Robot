function th=Lim_pi(thi)
ToDeg = 180/pi;  %转角度
ToRad = pi/180;  %转弧度
thi=thi*ToRad;
if thi>pi
    thi=thi-2*pi;
else
    if thi<-pi
        thi=thi+2*pi;
    end
end
th=thi*ToDeg;