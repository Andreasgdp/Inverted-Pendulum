clc
clear
close all

syms x_c v_c theta omega F m M l b_p b_c g I t s DDtheta DDx_c

%% Parameters
m = 0.084;
M = 0.5;
l = 0.35;
g = 9.82;
b_p = 0.0012;
b_c = 5;
I = (1/3)*m*(2*l)^2;

%% Vi fjoller rundt
%-------------------------------------------------------------------------------------------------------------------------------
Epot = m*g*l*cos(theta);

Ekintc = 1/2*M*v_c^2;

% Ekintp = 1/2*m*(diff(x_c+l*sin(theta))^2+(diff(-l*cos(theta))^2));
% (diff(x_c+l*sin(theta)) = v_c + omega*l*cos(theta)
% diff(-l*cos(theta))^2) = l*omega*sin(theta)
% diff(l*cos(theta))^2) = -l*omega*sin(theta)
Ekintp = 1/2*m*((v_c + omega*l*cos(theta))^2+(-l*omega*sin(theta))^2);

Ekint = Ekintc + Ekintp;

Ekinr = 1/2 * I * omega^2;

Ekin = Ekint + Ekinr;

L1 = 1/2*M*v_c^2 + 1/2*m*v_c^2 + 1/2*m*l^2*omega^2 + m*l*v_c*omega*cos(theta) + 1/2*I*omega^2 + m*g*l*cos(theta);

L = Ekin - Epot;

q  = [x_c, theta];
Dq = [v_c, omega];
Eq = LagrangeDynamicEqDeriver(L, q, Dq);

eq1 = Eq(1) == F - b_c*v_c;
eq2 = Eq(2) == b_p*omega;

ddTheta = ((-b_c*v_c + F)*l*m*cos(theta) + cos(theta)*sin(theta)*l^2*m^2*omega^2 - M*sin(theta)*g*l*m - sin(theta)*g*l*m^2 - M*b_p*omega - b_p*m*omega)/(cos(theta)^2*l^2*m^2 - M*l^2*m - l^2*m^2 - m*I - M*I);

ddX_c = ((b_p*omega + g*l*m*sin(theta))*l*m*cos(theta) - sin(theta)*l^3*m^2*omega^2 - sin(theta)*l*m*omega^2*I + b_c*l^2*m*v_c - F*l^2*m + b_c*v_c*I - F*I)/(cos(theta)^2*l^2*m^2 - M*l^2*m - l^2*m^2 - m*I - M*I);

ddThetaNegativeCos = ((-b_c*v_c + F)*l*m*-cos(theta) + -cos(theta)*sin(theta)*l^2*m^2*omega^2 - M*sin(theta)*g*l*m - sin(theta)*g*l*m^2 - M*b_p*omega - b_p*m*omega)/(-cos(theta)^2*l^2*m^2 - M*l^2*m - l^2*m^2 - m*I - M*I);

ddX_cNegativeCos = ((b_p*omega + g*l*m*sin(theta))*l*m*-cos(theta) - sin(theta)*l^3*m^2*omega^2 - sin(theta)*l*m*omega^2*I + b_c*l^2*m*v_c - F*l^2*m + b_c*v_c*I - F*I)/(-cos(theta)^2*l^2*m^2 - M*l^2*m - l^2*m^2 - m*I - M*I);

% -------------------------------------------------------------------------
% f = [v_c; 
%     ddX_c; 
%     omega; 
%     ddTheta];
% -------------------------------------------------------------------------
% -------------------------------------------------------------------------
f = [v_c; 
    ddX_cNegativeCos; 
    omega; 
    ddThetaNegativeCos];
% -------------------------------------------------------------------------


%-------------------------------------------------------------------------------------------------------------------------------
%% Definition of state space
x = [x_c; v_c; theta; omega];
u = F;
% Original f
% -------------------------------------------------------------------------
% f = [v_c; 
%     (b_p*m*l*omega*cos(theta) + m^2*l^2*g*sin(theta)*cos(theta) + (I + m*l^2)*(-b_c*v_c + F + m*l*omega^2*sin(theta)))/(M*m*l^2 + (M+m)*I + m^2*l^2*sin(theta)^2); 
%     omega; 
%     -(F*m*l*cos(theta) - b_c*m*l*v_c*cos(theta) + m^2*l^2*omega^2*sin(theta)*cos(theta) + (M + m)*(b_p*omega + g*m*l*sin(theta)))/(M*m*l^2 + (M + m)*I + m^2*l^2*sin(theta)^2)];
% -------------------------------------------------------------------------

% f where cos(theta) is changed to (-cos(theta)) due to theta being changed
% to 0 instead of pi.
% -------------------------------------------------------------------------
% f = [v_c; 
%     (b_p*m*l*omega*(-cos(theta)) + m^2*l^2*g*sin(theta)*(-cos(theta)) + (I + m*l^2)*(-b_c*v_c + F + m*l*omega^2*sin(theta)))/(M*m*l^2 + (M+m)*I + m^2*l^2*sin(theta)^2); 
%     omega; 
%     -(F*m*l*(-cos(theta)) - b_c*m*l*v_c*(-cos(theta)) + m^2*l^2*omega^2*sin(theta)*(-cos(theta)) + (M + m)*(b_p*omega + g*m*l*sin(theta)))/(M*m*l^2 + (M + m)*I + m^2*l^2*sin(theta)^2)];
% -------------------------------------------------------------------------

%% A
A = [diff(f(1), x(1)) diff(f(1), x(2)) diff(f(1), x(3)) diff(f(1), x(4)); 
    diff(f(2), x(1)) diff(f(2), x(2)) diff(f(2), x(3)) diff(f(2), x(4)); 
    diff(f(3), x(1)) diff(f(3), x(2)) diff(f(3), x(3)) diff(f(3), x(4)); 
    diff(f(4), x(1)) diff(f(4), x(2)) diff(f(4), x(3)) diff(f(4), x(4))];
% A = subs(A,theta,pi);   % Replace theta with pi
A = subs(A,theta,0);   % Replace theta with pi
A = subs(A,omega,0);   % Replace omega with diff(theta,t) = 0

%% B
B = [diff(f(1), u); diff(f(2), u); diff(f(3), u); diff(f(4), u)];
% B = subs(B,theta,pi);   % Replace theta with pi
B = subs(B,theta,0);   % Replace theta with pi

%% C
C = [1 0 0 0; 0 0 1 0];

%% D
D = [0; 0];

%% Rank (should be 4 to be controlable)
rankCtrl = rank(ctrb(A,B));

%% Rank (should be 4 to be observable)
rankObs = rank(obsv(A,C));

%% Convert to doubles
A = double(A);
B = double(B);
C = double(C);
D = double(D);

%% Transfer function for cart pos 
% TF_Cart = zpk(ss(A,B,C(1,:),D(1)));
% G_c = minreal(TF_Cart);
% 
% %% Transfer function for pendulum angle
% TF_Pend = zpk(ss(A,B,C(2,:),D(2)));
% G_p = minreal(TF_Pend);

% %% State space model to Transfer function
sys = ss(A, B, C, D);
G = tf(sys);
G_c = G(1);
G_p = G(2);

% Pole and zero
cart_poles = pole(G_c);
cart_zeros = zero(G_c);

pendulum_poles = pole(G_p);
pendulum_zeros = zero(G_p);

% Pzmap
% figure, pzmap(G_c)
% figure, pzmap(G_p)

% Step
% figure, step(G_c)
% figure, step(G_p)

% Root Locus
% figure, rlocus(G_p)
% figure, rlocus(G_c)

%% test P
s = tf('s');
kp = 25;

% pole((kp*G_p)/(1+kp*G_p))
% zero((kp*G_p)/(1+kp*G_p))
% figure, pzmap((kp*G_p)/(1+kp*G_p))
% figure, rlocus(kp*G_p)
%% test PD
s = tf('s');
kp = 250;
td = 0.08;

kpdFind = kp * (1 + td*s);

% pole((kpdFind*G_p)/(1+kpdFind*G_p))
% zero((kpdFind*G_p)/(1+kpdFind*G_p))
% figure, pzmap((kpdFind*G_p)/(1+kpdFind*G_p))
% figure, rlocus(kpdFind*G_p)
%% test PI
kp = 250;
ti = 10;

kpiFind = kp * (1 + ti/(s));

% pole((kpiFind*G_p)/(1+kpiFind*G_p))
% zero((kpiFind*G_p)/(1+kpiFind*G_p))
% figure, pzmap((kpidFind*G_p)/(1+kpidFind*G_p))

% figure, rlocus(kpiFind*G_p)
%% test PID
kp = 250;
ti = 10;
td = 0.08;

kpidFind = kp * (1 + ti/(s) + td*s);

% pole((kpidFind*G_p)/(1+kpidFind*G_p))
% zero((kpidFind*G_p)/(1+kpidFind*G_p))
% figure, pzmap((kpidFind*G_p)/(1+kpidFind*G_p))

%figure(69), rlocus(ksFind*G_p)
%% Performance specification 
t_r = 0.2;         %s
M_p = 20/100;
t_s = 2;         %s
alpha = 0.5/100;

omega_n = 1.8/t_r;
xi = sqrt((log(M_p)/(-pi))^2 / (1+(log(M_p)/(-pi))^2));
sigma = -(log(alpha))/(t_s);

figure, rlocus(kp*G_p)
% figure, rlocus(kpdFind*G_p)
% figure, rlocus(kpidFind*G_p)

% performancePlotColor = [0.9294, 0.6941, 0.1255];
% performancePlotColor = [0.40, 0.40, 0.40];
performancePlotColor = [0.1, 0.1, 0.1];

% Plot rise time
th = pi/2:pi/100:(3*pi)/2;
xunit = omega_n * cos(th);
yunit = omega_n * sin(th);
hold on, axis equal, grid on;
h = plot(xunit, yunit,'color',performancePlotColor,'LineWidth',1.5);

% Plot overshoot
x = -100:0.01:0;
% x = -6:0.01:0;
y = x/tan(asin(xi));
plot(x,-y,'color',performancePlotColor,'LineWidth',1.5)
plot(x,y,'color',performancePlotColor,'LineWidth',1.5)

% Plot settling time
% xline(-sigma,'color',[0.40, 0.40, 0.40],'LineWidth',1.5)
plot([-sigma -sigma],[-100 100],'color',performancePlotColor,'LineWidth',1.5);
% plot([-sigma -sigma],[-11.711887594987030 11.711887594987030],'color',performancePlotColor,'LineWidth',1.5);

% Draw axes
% xline(0,'-');
% yline(0,'-');
set(gca, 'XAxisLocation', 'origin')
set(gca, 'YAxisLocation', 'origin')
set(gca,'Xlim',[-25 10],'YLim',[-15 15])

set(findall(gcf,'type','line'),'linewidth',1.5);

% % Labels
% ylabel('$Im(s)$','interpreter','latex')
% xlabel('$Re(s)$','interpreter','latex')

hold off
%% Positon and size of window
%Positions
gca = figure(1);
% set figure hight and width
xSize = 900; ySize = 800; % 1 plot
xLeft = 100; yTop = 100;
set(gca,'Position',[xLeft yTop xSize ySize])

% exportgraphics(gca,'performancePlot.pdf','ContentType','vector')
exportgraphics(gca,'p_controller_pendulum_rlocus.pdf','ContentType','vector')
% exportgraphics(gca,'pd_controller_pendulum_rlocus.pdf','ContentType','vector')
% exportgraphics(gca,'pid_controller_pendulum_rlocus.pdf','ContentType','vector')

%% Modern control 

% Controllability
% See further up in document

% LQR controller
Q = [1000 0 0 0;
     0 1 0 0;
     0 0 10 0;
     0 0 0 1];
R = 0.05;
K = lqr(A,B,Q,R);

lqr_sys = ss((A-B*K), B, C, D);

% Observability
% See further up in document

% Observer
obsPoles = eig(A + B * K) * 5;
L = (-place(A', C', obsPoles))';

% stepinfo(ksFind*G_p, 'SettlingTimeThreshold', 0.005)
