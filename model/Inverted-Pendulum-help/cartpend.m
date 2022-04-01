function dy = cartpend(y,m,M,l,g,b_c,b_p,F)

I = (1/3)*m*(2*l)^2;

dy(1,1) = y(2);
dy(2,1) = (b_p*m*l*y(4)*cos(y(3)) + m^2*l^2*g*sin(y(3))*cos(y(3)) + (I + m*l^2)*(-b_c*y(2) + F + m*l*y(4)^2*sin(y(3))))/(M*m*l^2 + (M+m)*I + m^2*l^2*sin(y(3))^2);
dy(3,1) = y(4);
dy(4,1) = -(F*m*l*cos(y(3)) - b_c*m*l*y(2)*cos(y(3)) + m^2*l^2*y(4)^2*sin(y(3))*cos(y(3)) + (M + m)*(b_p*y(4) + g*m*l*sin(y(3))))/(M*m*l^2 + (M + m)*I + m^2*l^2*sin(y(3))^2);