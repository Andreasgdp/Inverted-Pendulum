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

%% Define A and B matrices
b = 1; % Pendulum up (b=1)

A = [0 1 0 0;
    0 -dampeningPendulum/M b*m*g/M 0;
    0 0 0 1;
    0 -b*dampeningPendulum/(M*L) -b*(m+M)*g/(M*L) 0];

B = [0; 1/M; 0; b*1/(M*L)];
%% Check the system. Is it stable/controlable?
% Confirm that the the open-loop system is unstable by checking the eigenvalues of A
lambda = eig(A)


% Rank should be 4 in order to be able to control the system (Get documentation on why this is the case)
rank(ctrb(A,B))

%% Use Matlab LQR controller
Q = eye(4); % 4x4 identify matrix
R = .0001;
K = lqr(A,B,Q,R);

%% Simulate closed-loop system with control
tspan = 0:.05:10;
x0 = [-1; 0; pi+.1; 0]; % initial condition
rp = [1; 0; pi; 0]; % reference position
u=@(x)-K*(x - rp); % control law
[t2,x2] = ode45(@(t,x)cartpend(x,m,M,L,g,dampeningPendulum,u(x)),tspan,x0);

for k=1:length(t2)
    drawcartpend(x2(k,:),m,M,L);
end