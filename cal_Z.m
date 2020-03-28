function [out] = cal_Z(x,y,z,Xs1,Xs2,Ys1,Ys2,Zs1,Zs2)
            Xrs1=x-Xs1;
            Xrs2=x-Xs2;
            Yrs1=y-Ys1;
            Yrs2=y-Ys2;
            Zrs1=z-Zs1;
            Zrs2=z-Zs2;
            S1=norm([Xrs1,Yrs1,Zrs1]);
            S2=norm([Xrs2,Yrs2,Zrs2]);
            gama1=asin(Yrs1/S1);
            gama2=asin(Yrs2/S2);
            eta1=atan2(-Zrs1,Xrs1);
            eta2=atan2(-Zrs2,Xrs2);
            out=[gama1 eta1 gama2 eta2]';
end
