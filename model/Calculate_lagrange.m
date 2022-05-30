syms xC theta  xDotC thetaDot 
syms l m M I g F c b

%% Kinetic and Potential Energy
L = 1/2*M*xDotC^2 + 1/2*m*xDotC^2 + 1/2*m*l^2*thetaDot^2 + m*l*xDotC*thetaDot*cos(theta) + 1/2*I*thetaDot^2 + m*g*l*cos(theta);
%%
q  = [xC, theta];
Dq = [xDotC, thetaDot];
Eq = LagrangeDynamicEqDeriver(L, q, Dq)

eq1 = Eq(1)
eq2 = Eq(2)