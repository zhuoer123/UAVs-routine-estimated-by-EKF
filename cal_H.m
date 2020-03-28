function [out] = cal_H(x,y,z,Xs1,Xs2,Ys1,Ys2,Zs1,Zs2)
            Xrs1=x-Xs1;
            Xrs2=x-Xs2;
            Yrs1=y-Ys1;
            Yrs2=y-Ys2;
            Zrs1=z-Zs1;
            Zrs2=z-Zs2;
            S1=norm([Xrs1 Yrs1 Zrs1]);
            S2=norm([Xrs2 Yrs2 Zrs2]);
            s1=norm([Xrs1 Zrs1]);
            s2=norm([Xrs2 Zrs2]);
            dh1x=1/sqrt(1-(Yrs1/S1)^2)*(-Xrs1*Yrs1)/S1^3;
            dh1y=1/sqrt(1-(Yrs1/S1)^2)*(1/S1+(-Yrs1^2)/S1^3);
            dh1z=1/sqrt(1-(Yrs1/S1)^2)*(-Zrs1*Yrs1)/S1^3;
            dh2x=Zrs1/s1^2;
            dh2y=0;
            dh2z=-Xrs1/s1^2;
            dh3x=1/sqrt(1-(Yrs2/S2)^2)*(-Xrs2*Yrs2)/S2^3;
            dh3y=1/sqrt(1-(Yrs2/S2)^2)*(1/S2+(-Yrs2^2)/S2^3);
            dh3z=1/sqrt(1-(Yrs2/S2)^2)*(-Zrs2*Yrs2)/S2^3;
            dh4x=Zrs2/s2^2;
            dh4y=0;
            dh4z=-Xrs2/s2^2;
            O=zeros(1,6);
            out=[ dh1x dh1y dh1z O;
                      dh2x dh2y dh2z O;
                       dh3x dh3y dh3z O;
                       dh4x dh4y dh4z O];
end

