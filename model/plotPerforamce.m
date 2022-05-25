clc
clear all
close all

%% Performance specification 
t_r = 0.2;         %s
M_p = 20/100;
t_s = 2;         %s
alpha = 0.5/100;

omega_n = 1.8/t_r;
xi = sqrt((log(M_p)/(-pi))^2 / (1+(log(M_p)/(-pi))^2));
sigma = -(log(alpha))/(t_s);

% Plot rise time
th = pi/2:pi/100:(3*pi)/2;
xunit = omega_n * cos(th);
yunit = omega_n * sin(th);
hold on, axis equal, grid on;
risetimePlot = plot(xunit, yunit,'color',[0.70, 0.70, 0.70],'LineWidth',1.5);

% Plot overshoot
x = -6:0.01:0;
y = x/tan(asin(xi));
overshoot1Plot = plot(x,-y,'color',[0.70, 0.70, 0.70],'LineWidth',1.5);
overshoot2Plot = plot(x,y,'color',[0.70, 0.70, 0.70],'LineWidth',1.5);

% Plot settling time
% xline(-sigma,'-m','LineWidth',1.5);
settlingtimePlot = plot([-sigma -sigma],[-11.711887594987030 11.711887594987030],'color',[0.70, 0.70, 0.70],'LineWidth',1.5);

% Draw axes
% xline(0,'-');
% yline(0,'-');
set(gca, 'XAxisLocation', 'origin')
set(gca, 'YAxisLocation', 'origin')

% Labels
ylabel('$Im(s)$','interpreter','latex')
xlabel('$Re(s)$','interpreter','latex')

hold off

%% Positon and size of window
%Positions
gca = figure(1);
% set figure hight and width
xSize = 900; ySize = 800; % 1 plot
xLeft = 100; yTop = 100;
set(gca,'Position',[xLeft yTop xSize ySize])

exportgraphics(gca,'performancePlot.pdf','ContentType','vector')

