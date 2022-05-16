clc
clear
close all

syms x_c v_c theta omega F m M l b_p b_c g I t s

%% Parameters
m = 0.084;
M = 0.5;
l = 0.35;
g = 9.82;
b_p = 0.0012;
b_c = 5;
I = (1/3)*m*(2*l)^2;

%% Definition of state space
x = [x_c; v_c; theta; omega];
u = F;
f = [v_c; 
    (b_p*m*l*omega*cos(theta) + m^2*l^2*g*sin(theta)*cos(theta) + (I + m*l^2)*(-b_c*v_c + F + m*l*omega^2*sin(theta)))/(M*m*l^2 + (M+m)*I + m^2*l^2*sin(theta)^2); 
    omega; 
    -(F*m*l*cos(theta) - b_c*m*l*v_c*cos(theta) + m^2*l^2*omega^2*sin(theta)*cos(theta) + (M + m)*(b_p*omega + g*m*l*sin(theta)))/(M*m*l^2 + (M + m)*I + m^2*l^2*sin(theta)^2)];

%% A
A = [diff(f(1), x(1)) diff(f(1), x(2)) diff(f(1), x(3)) diff(f(1), x(4)); 
    diff(f(2), x(1)) diff(f(2), x(2)) diff(f(2), x(3)) diff(f(2), x(4)); 
    diff(f(3), x(1)) diff(f(3), x(2)) diff(f(3), x(3)) diff(f(3), x(4)); 
    diff(f(4), x(1)) diff(f(4), x(2)) diff(f(4), x(3)) diff(f(4), x(4))];
A = subs(A,theta,pi);   % Replace theta with pi
A = subs(A,omega,0);   % Replace omega with diff(theta,t) = 0

%% B
B = [diff(f(1), u); diff(f(2), u); diff(f(3), u); diff(f(4), u)];
B = subs(B,theta,pi);   % Replace theta with pi

%% C
%C = [diff(x(1), x(1)) diff(x(2), x(2)) diff(x(3), x(3)) diff(x(4),x(4))];
%C = subs(C,theta,pi);   % Replace theta with pi
C = [1 0 0 0; 0 0 1 0];

%% D
D = [0; 0];

%% Rank (should be 4 to be controlable)
rank = rank(ctrb(A,B));

%% Convert to doubles
A = double(A);
B = double(B);
C = double(C);
D = double(D);


%% zero pole system 
TF_Cart = zpk(ss(A,B,C(1,:),D(1)));
G_c = minreal(TF_Cart);

%% Transfer function for pendulum angle
TF_Pend = zpk(ss(A,B,C(2,:),D(2)));
G_p = minreal(TF_Pend);



% %% State space model to Transfer function
sys = ss(A, B, C, D)

%pzmap(sys)

G = tf(sys);

G_c = G(1);
G_p = G(2);

% 
% % Pole and zero
% cart_poles = pole(G_c);
% cart_zeros = zero(G_c);

pendulum_poles = pole(G_p);
pendulum_zeros = zero(G_p);

% Pzmap
figure, pzmap(G_c)
%figure, pzmap(G_p)

% Step
% figure, step(G_c)
%figure, step(G_p)



%figure, rlocus(G_p)

%% test P
s = tf('s');
kp = 120;

%figure(42), rlocus(G_p)
%% test PD
s = tf('s');
kp = 120;
td = 0.08;

ksFind = (1 + td*s);

%figure(43), rlocus(ksFind*G_p)
%% test PID
% kp = 120;
% ti = 10;
% td = 0.08;

% kp = 60;
% ti = 10.0545991325963;
% td = 0.0244071621390575;

% kp = 558;
% ti = 3233.12;
% td = 22.38;

% kp = 37.5559806691701;
% ti = 4.79031708469581;
% td = 0.0396308247236052;

% Cart
% kp = -0.0603910190701767;
% ti = 0.0784455862847791;
% td = 3.01881554246508;

kp = -0.0163648626701333;
ti = 0.0434531271556642;
td = 5.55644601779429;

ksFind = (1 + ti/(s) + td*s);

pole((ksFind*G_c)/(1+ksFind*G_c))
zero((ksFind*G_c)/(1+ksFind*G_c))
figure, pzmap((ksFind*G_c)/(1+ksFind*G_c))

%figure(69), rlocus(ksFind*G_p)
%% test PID 1 1 1
% Kp = 120;
% Ki = 35;
% Kd = 8;
% Contr = pid(Kp,Ki,Kd);
% T = feedback(G_p,Contr);
% figure()
% t=0:0.01:10;
% impulse(T,t)

%% Performance specification 
t_r = 0.2;         %s
M_p = 40/100;
t_s = 2;         %s
alpha = 0.5/100;

omega_n = 1.8/t_r;
xi = sqrt((log(M_p)/(-pi))^2 / (1+(log(M_p)/(-pi))^2));
sigma = -(log(alpha))/(t_s);

% Plot rise time
th = pi/2:pi/100:(3*pi)/2;
xunit = omega_n * cos(th);
yunit = omega_n * sin(th);
figure(42), rlocus(ksFind*G_c)
hold on, axis equal, grid on;
h = plot(xunit, yunit,'r');

% Plot overshoot
x = -100:0.01:0;
y = x/tan(asin(xi));
plot(x,-y,'r')
plot(x,y,'r')

% Plot settling time
xline(-sigma,'-r')

% Draw axes
xline(0,'-');
yline(0,'-');

hold off

