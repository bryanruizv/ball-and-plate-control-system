%Codigo de MATLAB utilizado:
clear all
close all
s=tf('s')
Kd= 0.067;
Kp = 0.0155;
Ki = 0.000897;
G=14.7346/s^2
C=Kd*s+Kp +Ki/s
pidtune(G,'PID')
%% Grafica
t = 0:0.05:30;
esc=heaviside(t);
T1=feedback(G*C,1)
ys=step(T1,t);
figure(1);
plot(t,ys,t,esc);
title("Respuesta al Escal´on Unitario");
xlabel("Tiempo(s)");
ylabel("Amplitud")
legend("Sistema en Lazo Cerrado","Se~nal Escal´on Unitario");
grid;
close all
clc
s = tf('s');
G = 14.7346/s^2;
figure(1)
step(G)
title('Respuesta del sistema a la entrada escal´on')
grid on
% Controlador Pidtune
pidtune(G,'PID')
C1 = (0.0155*s+8.97*10^-4 +0.067s^2)/s
Gc1 = feedback(G*C1, 1); % Sistema con control
figure(1)
step(Gc1)
title('Respuesta del sistema con PIDtune')
grid on
% Controlador LGR
C2 = 0.1602*(s+0.5011)^2/s
Gc1 = feedback(G*C2, 1); % Sistema con control
figure(2)
step(Gc2)
title('Respuesta del sistema con PID (LGR)')
grid on
G1=14.7346/s^2;
G_1=minreal(zpk(G1),1e-4);
disp('Funcion de transferencia de la planta')
G_1
%Cambiar valores aca
ki = 0;
Mr = 1;
Bw = 1;
xi = 0.5912
bloque1 = sqrt(1-2*xi^2 + sqrt((1-2*xi^2)^2 +1));
Mf = xi*100
wn = 0.6766
% Codigo para conseguir el modulo y fase de la planta
resp = evalfr(G_1, 1j*wn);
Mod_Planta = abs(resp)
Pha_Planta = angle(resp)
Pha_Kpid = ((Mf*2*pi)/360)-pi-Pha_Planta
Mod_Kpid = 1/Mod_Planta
Kp = Mod_Kpid*cos(Pha_Kpid)
Kd = (Mod_Kpid*sin(Pha_Kpid) + (ki/wn))/wn
ki
Kpid=0.0159+0.0394*s+0/s;
Glc1=feedback(G1,1);
Glc2=feedback(G1*Kpid,1);
t=0:0.01:50;
esc=heaviside(t);
ys1=step(Glc1,t);
ys2=step(Glc2,t);
figure(1)
hold on
plot(t,esc,'k',"DisplayName","Entrada Escal´on Unitario");
plot(t,ys1,'b',"DisplayName","Sistema Sin Controlador PID");
plot(t,ys2,'r',"DisplayName","Sistema Con Controlador PID");
title("Ley de Control");
xlabel("Tiempo(s)");
ylabel("Amplitud");
legend
hold off
