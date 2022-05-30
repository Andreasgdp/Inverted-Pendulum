% close all
clear
clc

s = tf('s');

%% Pendulum transfer function controller

G_p = 0.294*s/(0.2353*s^3 + 1.202*s^2 - 3.863*s - 14.44);

kp = 1326.9;

ti = 0.6;

td = 2;

ksFind = (1 + ti/(s) + td*s);

ks = kp*(1 + ti/(s) + td*s);
% 
% figure(1)
% rlocus(ksFind*G_p)
% 
% figure(2)
% step(ks*G_p/(1 + ks*G_p))

%% Cart transfer function controller (CAN'T SEEM TO GET THIS ONE TO STABILIZE)

G_c = 0.2401*s^2 + 0.0012*s - 2.887/(0.2353*s^4 + 1.202*s^3 - 3.863*s^2 - 14.44*s);

kp = 1.72;

ti = 0.0434531271556642;

td = 5.55644601779429;

ksFind = (1 + ti/(s) + td*s);
ks = kp*(1 + ti/(s) + td*s);

z = [-3 -4];
p = 0;
k = 1;
C = zpk(z,p,k)

pole(C*G_c)
zero(C*G_c)

% figure(1)
rlocus(ksFind*G_c)
% sisotool(ksFind*G_c)
% 
% figure(2)
% step(ks*G_c/(1 + ks*G_c))