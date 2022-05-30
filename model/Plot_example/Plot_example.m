%% convert UR recorded orientation  trajectory to RPY
clc
clear all
close all

load Data.mat

%% Convert orientation
Pos =Data(:,1:3);
Ori = Data(:,4:6);

%% calculate time based on the input signal and sampling rate
dt = 1/500;
time= length(Pos);
T= time*dt;
Time = 0:dt:T;
Time = Time(:,1:length(Pos))';

%% plot 
%Positions
gca= figure(1)
% set figure hight and width
xSize = 900; ySize = 800; % 1 plot
xLeft = 100; yTop = 100;
set(gca,'Position',[xLeft yTop xSize ySize])
%% PLot data 
subplot(3,1,1)
H1=plot(Time,Pos(:,1),'LineWidth',1.5);
ylabel('$X_{pos}$ [m]','interpreter','latex')
legend([H1],'UR trajectory','Location','southwest');
grid on

subplot(3,1,2)
plot(Time,Pos(:,2),'LineWidth',1.5)
ylabel('$Y_{pos}$ [m]','interpreter','latex')
grid on

subplot(3,1,3)
plot(Time,Pos(:,3),'LineWidth',1.5)
ylabel('$Z_{pos}$ [m]','interpreter','latex')
xlabel('$Time$ [s]','interpreter','latex')
grid on

%% setup figure parameters 
set(gca,'Units','Inches');
pos = get(gca,'Position');
set(gca,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
% save figure as vector .pdf
exportgraphics(gca,'Positions.pdf','ContentType','vector')

%% orientations in RPY
%% plot 
%Orientations
gca= figure(2)
% set figure hight and width
xSize = 400; ySize = 400; % 1 plot
xLeft = 100; yTop = 100;
set(gca,'Position',[xLeft yTop xSize ySize])

H1=plot(Time,Ori(:,1),'LineWidth',1.5);
hold on 
plot(Time,Ori(:,2),'r','LineWidth',1.5);
ylabel('$Roll$ [deg]','interpreter','latex')
legend([H1],'UR trajectory','Location','southwest');
grid on

%% setup figure parameters 
set(gca,'Units','Inches');
pos = get(gca,'Position');
set(gca,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
% save figure as vector .pdf
exportgraphics(gca,'orientations.pdf','ContentType','vector')
