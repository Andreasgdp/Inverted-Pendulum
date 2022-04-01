clear all, close all, clc

m = 0.084;
M = 0.5;
l = 0.35;
g = 9.82;
b_p = 0.0012;
b_c = 5;

tspan = 0:.1:25;
y0 = [0; 0; pi; .5];
[t,y] = ode45(@(t,y)cartpend(y,m,M,l,g,b_c,b_p,0),tspan,y0);

for k=1:length(t)
    drawcartpend(y(k,:),m,M,l);
end

% function dy = pendcart(y,m,M,L,g,d,u)