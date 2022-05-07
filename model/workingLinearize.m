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
% [b,a] = ss2tf(A,B,C,D);
% 
% sys = ss(A, B, C, D)
% 
% % numerator = b;
% % denominator = a;
% % G = tf(numerator,denominator);
% 
% G = tf(sys);
% 
% G_c = G(1);
% G_p = G(2);
% 
% 
% % Pole and zero
cart_poles = pole(G_c);
cart_zeros = zero(G_c);

pendulum_poles = pole(G_p);
pendulum_zeros = zero(G_p);

% Pzmap
figure, pzmap(G_c)
figure, pzmap(G_p)

% Step
figure, step(G_c)
figure, step(G_p)

%% test PID 1 1 1
Kp = 120;
Ki = 35;
Kd = 8;
Contr = pid(Kp,Ki,Kd);
T = feedback(G_p,Contr);
figure()
t=0:0.01:10;
impulse(T,t)


