% Filename : Main
% Created using the guidance given in AE4351: Robust Flight Control
%
% Bo Lee (5225604)
% Matei Dinescu
%%


%% FLIGHT CONDITION
alpha = 20 * pi/180; % rad - angle of attack
M = 3;               % [-] - Mach number
h = 6096;            % m - altitude


%% CONSTANTS
g = 9.80665;         % m/s^2 - standard gravity acceleration
a = 316.0561;        % m/s - speed of sound at 6096m
Z_alpha = 1236.8918; % m/s^2 - normal force derivative
M_alpha = -300.4211; % 1/s^2 - pitch moment derivative
M_q = 0;             % 1/s - damping derivative
Z_delta = 108.1144;  % m/s^2 - control force derivative
M_delta = -131.3944; % 1/s^2 - control moment derivative
A_alpha = 1434.7783; % m/s^2/rad - normal acceleration derivative
A_delta = 115.0529;  % m/s^2/rad - control acceleration derivative
omega_a = 150;   % rad/s - actuator natural frequency
zeta_a = 0.7;    % [-] - actuator damping ratio


%% Q1.1: FLIGHT DYNAMICS
V = M * a; % m/s

% Actuator state space
A_a = [0 1; -omega_a^2 -2*zeta_a*omega_a];
B_a = [0; omega_a^2];
C_a = [1 0; 0 1];
D_a = [0; 0];

G_a.InputName = 'u_cmd';
G_a.StateName = {'x3', 'x4'};
G_a.OutputName = {'u_m', 'udot_m'};
save G_a

% Missile state space
A_m = [-Z_alpha/V 1; M_alpha M_q];
B_m = [-Z_delta; M_delta];
C_m = [-A_alpha 0; 0 1];
D_m = [-A_delta/g; 0];

G_m = ss(A_m, B_m, C_m, D_m);
G_m.InputName = 'u_m';
G_m.StateName = {'x1', 'x2'};
G_m.OutputName = {'y1', 'y2'};
save G_m

% Run Airframe.slx
G_am = linearize('Airframe');





