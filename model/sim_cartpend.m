clear all, close all, clc

%% Initialize the model (Parameters)
modelObject = matfile('model.mat');

mRod = modelObject.mRod; 
mEnd = modelObject.mEnd; 
M = modelObject.M; 
L = modelObject.L; 
g = modelObject.g; 
dampeningPendulum = modelObject.dampeningPendulum; 
dampeningConveyor = modelObject.dampeningConveyor;


% Combined mass of pendulum
m = modelObject.m;

%% Simulate closed-loop system without control
tspan = 0:.01:10;
x0 = [-1; 0; pi+.1; 0]; % initial condition
[t1,x1] = ode45(@(t,x)cartpend(x,m,M,L,g,dampeningPendulum,0),tspan,x0);

for k=1:length(t1)
   drawcartpend(x1(k,:),m,M,L);
end
