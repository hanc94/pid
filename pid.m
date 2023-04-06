%Diseño de un PID usando el LGR
%suponga que se tiene el siguiente sistema
clc; clear;
r2g=180/pi
s=tf('s');
G= (-0.1431*s^2 + 1.609*s + 175.4)/(s^3 + 17*s^2 + 134.3*s +159.5)
%Se desea un tiempo de establecimiento de 5s y un sobre paso de 10%
%El polo deseado es:
 ts=1
 zwn=4.6/ts
 z=sqrt(log(0.05)^2/(pi^2+log(0.05)^2))
 wn=zwn/z
s0=-zwn+i*wn*sqrt(1-z^2)
AG=angle(s0-41.08068664)+ angle(s0+29.83680125)-angle(s0^3 + 17*s0^2 + 134.3*s0 +159.5)
AGgr=AG*r2g
psi=-pi-AG
%en grados:
psigr=psi*r2g
%El PID tiene la forma K(s+a)(s+b)/s
%El ángulo que aporta el PID está dado por <(s0+a)+<(s0+b)-<s0
%El ángulo que aporta el polo en el origen es:
pido=angle(s0) %en grados es 126.2390º
%Asi <(s0+a)+<(s0+b)-126.2390º=psigrº
%Es decir los dos ceros deben aportar <(s0+a)+<(s0+b)=126.2390º+psigrº=Az
%Exiten varias elecciones como:
%que (s+a) y (s+b) aporten Az/2
%que ubiquemos a=zwn, es decir que aporte 90º y calculamos b para que
a=abs(real(s0))
%aporte Azº-90º=Abº
Ab=pido+psi-pi/2
%entonces <(s0+b)=Abº
%<s0+b)=Abº
% donde Im(s0)/(Re(s0) + b)=tan(Abº)
%entonces b=-Re(s0) + Im(s0)/tan(Abº)
b= -real(s0)+ imag(s0)/tan(Ab)
%asi PID=K(s+a)(s+b)/s
%se usa el criterio de la magnitud para encontrar K
K=1/abs(((s0+a)*(s0+b)/s0)*((-0.1431*s0^2 + 1.609*s0 + 175.4)/(s0^3 + 17*s0^2 + 134.3*s0 +159.5)))
%del algebra del pid se tiene KD=K; KI/KD=ab; KP/KD=a+b
%donde
KD=K
KI=a*b*KD
KP=(a+b)*KD
% Implementación en simulink usando el bloque pid

%%CONTROLADOR EN ADELANTO

% KA=1/abs(((s0+a)/(s0+b))*(394.3/((s0+29.78)*(s0-29.78)*(s0+53.23))))
% LG=G*(((s+a)*(s+b))/s);
% PIDd=tf(((s+a)*(s+b))/s);
% Sdiscreto2 = c2d(PIDd,0.01,'zoh')