
% Aditya Swaroop Attawar

% For more information, contact me:
% Email: asattawar@gmail.com
% GitHub: https://github.com/aditya-attawar
% LinkedIn: https://www.linkedin.com/in/attawar

%%%%%%%%%%%%%%%%%%

clc;

% parameters
par.L = 0.132; % leg length
par.R = 0.06;  % foot radius
par.m = 0.07;  % leg mass
par.I = 1.8e-4;  % leg inertia
par.B = 0;    % CoM B
par.C = 0.061; % CoM C 
par.gamma = 0.005; % ground slope
par.g = 9.81;

% initial conditions
s0 = [  0.2; -0.2; -2.5; -2.0];
t0 = 0;

% Step
[s_end1, t_end1, data1] = Step(s0, t0, par);
Walk_and_Step_Animate(data1, par, 0.2);

% Walk
[s_end2, t_end2, data2] = Walk(s0, t0, par, 10);
Walk_and_Step_Animate(data2, par, 3);
