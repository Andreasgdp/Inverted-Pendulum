%% Initialize the model (Parameters)
mRod = .082; % [kg]
mEnd = .0002; % [kg]
M = .5; % [kg]
L = .35; % [m]
g = -9.82; % [m/s2]
lenConveyor = 1.72; % [m]
radiosConveyor = .05; % [m]
dampeningPendulum = .0012; % [N/(m/s)]
dampeningConveyor = 5; % [N/(m/s)]


% Combined mass of pendulum
m = mRod + mEnd; % [kg]


save model.mat mRod mEnd M m L g dampeningPendulum dampeningConveyor lenConveyor radiosConveyor -v7.3;
clear mRod mEnd M m L g dampeningPendulum dampeningConveyor lenConveyor radiosConveyor