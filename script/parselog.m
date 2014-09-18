%Questo script è utilizzato per parsare i valori di velocità inviati dal
%veicolo, ricavarne la posizione e plottarla.
% f = handler del file di log da utilizzare
% Formato file di log: "Vl Vr Vd Rd"

function parselog(f)

%Impostazione del formato del file
format='%f %f %f %f';
size=[4 Inf];

%Lettura dei dati in una matrice
A=fscanf(f,format,size);
A=A';

%Estrazione dei valori di velocità sinistra e destra e relativo timestamp
Vl=A(:,1);
Vr=A(:,2);
T=[0:0.01:(length(Vl)/100)-0.01]';

%Formule dell'uniciclo per ricavare velocità lineare e angolare del
%baricentro
Vb=(Vr+Vl)./2;
Wb=(Vr-Vl)./0.23;

%Integrazione della velocità angolare per ricavare l'angolo
Tb=cumtrapz(T,Wb);

%Velocità sul piano ricavata con formule trigonometriche da angolo e
%velocità del baricentro
Vx=Vb.*(sin(Tb));
Vy=Vb.*(cos(Tb));

%Posizione ricavata tramite integrazione delle velocità sul piano
Px=cumtrapz(T,Vx);
Py=cumtrapz(T,Vy);

%Traiettoria del veicolo
plot(Px,Py)
