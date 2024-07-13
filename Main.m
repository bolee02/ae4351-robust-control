%%%
% Filename : Main
% Created using the guidance given in AE4351: Robust Flight Control
%
% Bo Lee (5225604)
% Matei Dinescu
%%%

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
omega_a = 150;       % rad/s - actuator natural frequency
zeta_a = 0.7;        % [-] - actuator damping ratio

V = M * a;           % m/s - missile velocity


%% Q1: SYSTEM MODELING
% Create missile state space
A_m = [-Z_alpha/V 1; M_alpha M_q];
B_m = [-Z_delta/V; M_delta];
C_m = [-A_alpha/g 0; 0 1];
D_m = [-A_delta/g; 0];

G_m = ss(A_m, B_m, C_m, D_m);
G_m.InputName = 'u_m';
G_m.StateName = {'x1', 'x2'};
G_m.OutputName = {'y1', 'y2'};
save G_m

% Create actuator state space
A_a = [0 1; -omega_a^2 -2*zeta_a*omega_a];
B_a = [0; omega_a^2];
C_a = [1 0; 0 1];
D_a = [0; 0];

G_a = ss(A_a, B_a, C_a, D_a);
G_a.InputName = 'u_cmd';
G_a.StateName = {'x3', 'x4'};
G_a.OutputName = {'u_m', 'udot_m'};
save G_a

% Run Airframe.slx and visualize pole-zero map
G_am = linearize('Airframe');
T_state = [0 0 1 0; 0 0 0 1; 1 0 0 0; 0 1 0 0];
G_am = ss2ss(G_am, T_state);

G_am_nz = zpk(G_am(1, 1));
G_am_q = zpk(G_am(2,1));

%figure;
%iopzmap(G_am);
%grid on;
%title('iopzmap(G_{am})');


%% Q2: LOOP SHAPING
% Damping gain design
%figure;
%G_ol_q = G_am(2, 1);
%rlocusplot(-G_ol_q);
%sgrid;
%title('rlocusplot(-G_{ol_q})');

C_q = -0.163; % from rlocusplot

G_cl_q_unsc = linearize('ClosedLoop_Cq');
G_cl_q_unsc = ss2ss(G_cl_q_unsc, T_state);
G_cl_q_unsc_zpk = zpk(G_cl_q_unsc);

% Scaling gain design
C_sc = 1 / dcgain(G_cl_q_unsc);
G = linearize('ClosedLoop_CqCsc');
G_zpk = zpk(G);

%figure;
%step(G);
%grid on;
%title('step(G)');

%figure;
%iopzmap(G);
%grid on;
%title('iopzmap(G)');

% Integral Gain Design
C_i = 1;

G_ol_nz = linearize('ClosedLoop_CqCscCi');
G_ol_nz = ss2ss(G_ol_nz, [0 0 0 1 0; ...
    0 0 0 0 1; ...
    0 1 0 0 0; ...
    0 0 1 0 0; ...
    1 0 0 0 0]);
G_ol_nz_zpk = zpk(G_ol_nz);

% sisotool(G_ol_nz);
C_i_pm60 = 6.2754; % from sisotool


%% Q3: MIXED SENSITIVITY DESIGN
% Q3A: Weighting filter computation
M_s_min = 0.5 * (1 / sin(deg2rad(30/2)));

dcgain_W1 = db2mag(-60);
freq_W1 = 4;
mag_W1 = db2mag(-3.01);
hfgain_W1 = M_s_min;

W1  = inv(makeweight(dcgain_W1, [freq_W1,mag_W1], hfgain_W1));
% W1  = makeweight(dcgain_W1, [freq_W1,mag_W1], hfgain_W1);

dcgain_W2 = hfgain_W1;
freq_W2 = bandwidth(G_a(1, 1), -3.01);
mag_W2 = db2mag(-15);
hfgain_W2 = dcgain_W1;

W2  = inv(makeweight(dcgain_W2, [freq_W2,mag_W2], hfgain_W2));
% W2  = makeweight(dcgain_W2, [freq_W2,mag_W2], hfgain_W2);

% Q3B: Reference model
t_sd = 0.18;
M_d = 0.05;
z_m = 36.6; % from iopzmap(G)

% omega_d = 0;
% zeta_d = 0;
% error = inf;
% 
% omega_d_range = linspace(0, 30, 300);
% zeta_d_range = linspace(0, 1, 100);
% 
% for omega_d_temp = omega_d_range
%     for zeta_d_temp = zeta_d_range
%         num_temp = [-omega_d_temp^2/z_m, omega_d_temp^2];
%         den_temp = [1, 2 * zeta_d_temp * omega_d_temp, omega_d_temp^2];
%         T_d_temp = tf(num_temp, den_temp);
% 
%         step_response = stepinfo(T_d_temp, 'SettlingTimeThreshold', 0.05);
%         st_error = abs(step_response.SettlingTime - t_sd);
%         os_error = abs(step_response.Overshoot/100 - M_d);
%         tot_error = os_error + st_error;
% 
%         if tot_error < error
%             error = tot_error;
%             omega_d = omega_d_temp;
%             zeta_d = zeta_d_temp;
%         end
%     end
% end

% omega_d = 28.2838; % from for loop
omega_d = z_m/2; %from robust theory
zeta_d = 0.7071; % from for loop

num = [-omega_d^2 / z_m, omega_d^2];
den = [1, 2 * zeta_d * omega_d, omega_d^2];
T_d = tf(num, den);

A1 = dcgain_W1;
M1 = hfgain_W1;
omega1 = sqrt(((1/mag_W1^2) * freq_W1^2 - (freq_W1^2)/hfgain_W1^2)/ ...
    (1 - (1/mag_W1^2) * dcgain_W1^2));

A2 = dcgain_W2;
M2 = hfgain_W2;
omega2 = sqrt(((1/mag_W2^2) * (freq_W2 * hfgain_W2)^2 - freq_W2^2)/ ...
    (1/dcgain_W2^2 - (1/mag_W2^2)));

figure;
step(T_d);
grid on;
title('step(T_d)');

T_d_zpk = zpk(T_d);

%% 
%%Q3C: Feedback Controller design (hinfsyn)

W3 = W1;
P = linearize('Design');
T_P = [0 0 1 0 0 0 0 0 0;
       0 0 0 1 0 0 0 0 0;
       1 0 0 0 0 0 0 0 0;
       0 1 0 0 0 0 0 0 0;
       0 0 0 0 1 0 0 0 0;
       0 0 0 0 0 1 0 0 0;
       0 0 0 0 0 0 1 0 0;
       0 0 0 0 0 0 0 1 0;
       0 0 0 0 0 0 0 0 1];
P = ss2ss(P,T_P);
n_meas = 1;
n_cont = 1;
% P_zpk = zpk(P);

%Hinf control. 
hinf_opts = hinfsynOptions("RelTol",1e-6);
[C_e,T_wz,gamma] = hinfsyn(P,n_meas,n_cont,hinf_opts);

sigma_opts = sigmaoptions;
sigma_opts.MagUnits = 'abs';

%Ploting singular values
figure
sigma(T_wz, T_wz(1,1), T_wz(2,1), T_wz(3,1), sigma_opts)
legend('T_wz(s)', 'T_wz_1(s)', 'T_wz_2(s)', 'T_wz_3(s)')
grid on

%Calculating performance levels
T_wz_inf = norm(T_wz, 'inf');
gamma_1 =  norm(T_wz(1,1),'inf');
gamma_2 =  norm(T_wz(2,1),'inf');
gamma_3 =  norm(T_wz(3,1),'inf');
%%
%Initialising W3
dcgain_W3 = db2mag(-60);
freq_W3 = 4;
hfgain_W3 = M_s_min;

i = 0.00;
% while gamma_1 <1 && gamma_2 <1 && gamma_3<1
%     i = i + 0.01;
%     mag_W3 = db2mag(-10 - i);
%     W3  = inv(makeweight(dcgain_W3, [freq_W3,mag_W3], hfgain_W3));
% 
%     P = linearize('Design');
%     P = ss2ss(P,T_P);
%     [C_e,T_wz,gamma] = hinfsyn(P,n_meas,n_cont,hinf_opts);
%     gamma_1 =  norm(T_wz(1,1),'inf');
%     gamma_2 =  norm(T_wz(2,1),'inf');
%     gamma_3 =  norm(T_wz(3,1),'inf');
% end
mag_W3 = db2mag(-18.08);
W3  = inv(makeweight(dcgain_W3, [freq_W3,mag_W3], hfgain_W3));

P = linearize('Design');
P = ss2ss(P,T_P);

hinf_opts = hinfsynOptions("RelTol",1e-6);
[C0_e,T_wz,gamma] = hinfsyn(P,n_meas,n_cont,hinf_opts);

sigma_opts = sigmaoptions;
sigma_opts.MagUnits = 'abs';

%Ploting singular values
figure
sigma(T_wz, T_wz(1,1), T_wz(2,1), T_wz(3,1), sigma_opts)
legend('T_wz(s)', 'T_wz_1(s)', 'T_wz_2(s)', 'T_wz_3(s)')
grid on

%Calculating performance levels
T_wz_inf = norm(T_wz, 'inf');
gamma_1 =  norm(T_wz(1,1),'inf');
gamma_2 =  norm(T_wz(2,1),'inf');
gamma_3 =  norm(T_wz(3,1),'inf');



%% Q3C2: Feedback Controller 

[z, p, k] = zpkdata(C0_e, 'v');

%% Q3D: Feedback controller (hinfstruct)
s= tf('s');

C_e_red_star0 = tunableTF('C_e_red_star0', 2, 2)*1/s;
P = linearize('Design');
T_P = [0 0 1 0 0 0 0 0 0;
       0 0 0 1 0 0 0 0 0;
       1 0 0 0 0 0 0 0 0;
       0 1 0 0 0 0 0 0 0;
       0 0 0 0 1 0 0 0 0;
       0 0 0 0 0 1 0 0 0;
       0 0 0 0 0 0 1 0 0;
       0 0 0 0 0 0 0 1 0;
       0 0 0 0 0 0 0 0 1];
P = ss2ss(P,T_P);

struct_options = hinfstructOptions('UseParallel', true, 'TolGain', 1e-6);
[C_e_red_star, gamma_star, info] = hinfstruct(P,C_e_red_star0, struct_options);
C_e_red_star = zpk(C_e_red_star);

T_wz_star = lft(P, C_e_red_star, 1, 1);
gamma_1_star =  norm(T_wz_star(1,1),'inf');
gamma_2_star =  norm(T_wz_star(2,1),'inf');
gamma_3_star =  norm(T_wz_star(3,1),'inf');

%% Q3D2

