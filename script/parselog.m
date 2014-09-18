function parselog(f)
format='%f %f %f %f';
size=[4 Inf];
A=fscanf(f,format,size);
A=A';

Vl=A(:,1);
Vr=A(:,2);

T=[0:0.01:(length(Vl)/100)-0.01]';
Vb=(Vr+Vl)./2;
Wb=(Vr-Vl)./0.23;
Tb=cumtrapz(T,Wb);
Vx=Vb.*(sin(Tb));
Vy=Vb.*(cos(Tb));
Px=cumtrapz(T,Vx);
Py=cumtrapz(T,Vy);

plot(Tb);
figure;
plot(Px,Py)
