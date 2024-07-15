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
%set(gcf, "Color", "white")
%title('G_{am} Pole-zero Map');

%damp(G_am)
%zero(G_am(1,1))
%zero(G_am(2,1))

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
G_ol_q = zpk(G_am(1, 1));
%iopzmap(G_cl_q_unsc)

% Scaling gain design
C_sc = 1 / dcgain(G_cl_q_unsc);
G = linearize('ClosedLoop_CqCsc');
G_zpk = zpk(G);

%figure;
%step(G);
%grid on;
%set(gcf, "Color", "white")
%title('Step Response of G');

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

%sisotool(G_ol_nz);
C_i_pm60 = 6.2498; % from sisotool
C_i = 5.4662;

G_ol_nz2 = linearize('ClosedLoop_CqCscCi');
G_ol_nz2 = ss2ss(G_ol_nz2, [0 0 0 1 0; ...
    0 0 0 0 1; ...
    0 1 0 0 0; ...
    0 0 1 0 0; ...
    1 0 0 0 0]);
G_ol_nz2_zpk = zpk(G_ol_nz2);

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
sigma(inv(W1),'r-',inv(W2),'b-')
set(gcf, "Color", "white")
grid on;
xlim([1e-3 1e4]);
legend('W1^{-1}', 'W2^{-1}');

figure;
step(T_d);
grid on;
set(gcf, "Color", "white")
title('Step Response of T_d');

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
mag_W3 = db2mag(-18.07);
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
s= tf('s');
% C0_e = minreal(C0_e);
[z, p, k] = zpkdata(C0_e, 'v');
%by inspection

z_min = z(2:7);
p_min = p(2:8);
k_min = k*z(1)/p(1)*z(8)/p(9);
C_e_min = zpk(z_min, p_min, k_min);


figure;
bode(C0_e, C_e_min)
grid on 
legend('C0_e', 'C_e_min')

C_i_min = zpk(z_min, p_min(1:6), k_min);
C_i_zpk = zpk(C_i_min);

C_i_red = balred(C_i_zpk, 3);

figure;
bode(C_i_red, 'r', C_i_min, 'b')
grid on 
legend('C_i_{red}', 'C_i_{min}')

figure;
iopzmap(C_i_red, 'r', C_i_min, 'b');
grid on;
legend('C_i_{red}', 'C_i_{min}')

%% Q3C3 part 1

%Assinging controller values
C_i = C_i_red;
F_f = 1;

%Retrieveing matrix transfer function
T = linearize('ClosedLoop_Test');
T = zpk(T);

%Isolating required transfer functions
So = T(1,1);
Ce_So = T(2,1);
To = T(3,1);
Tm = T(4,1);
Tr_udot = T(6,1);
Ti = -T(2,2); %Need to plot Ti
So_G = T(3,2);
Si = T(5,2);

%First figure
figure;
subplot(2,3,1);

sigma(inv(W1), 'r', So, 'b*', Si, 'b', sigma_opts)
grid on


subplot(2,3,2);
sigma(inv(W2), 'r', Ce_So, 'b', sigma_opts)
grid on


subplot(2,3,3);
sigma(inv(W3), 'r', Tm, 'b', sigma_opts)
grid on


subplot(2,3,4);
sigma(Ti, 'b*', To, 'b', sigma_opts)
grid on

subplot(2,3,5);
sigma(So_G, 'b', sigma_opts)
grid on

subplot(2,3,6);
sigma(C0_e, 'b', C_i_red/s, 'b', sigma_opts)
grid on

sigma_opts2 = sigmaoptions;
sigma_opts2.MagUnits = 'abs';
sigma_opts2.XLim = [1e1, 1e3];

figure;
sigma(C0_e, 'b', C_i_red/s, 'b', sigma_opts2)
grid on

%% Q3C3 part 2:
T_OL = linearize("OpenLoop_Test");

[GM, PM, omega_cg, omega_cp] = margin(T_OL);
DM = deg2rad(PM)/ omega_cp * 1000;

figure;
bode(T_OL)
grid on

%% Q3C3 part 3

figure;
subplot(2,2,1)
step(So)
grid on

subplot(2,2,2)
step(T_d, 'r',  To, 'b')
grid on

subplot(2,2,3)
step(So_G)
grid on

subplot(2,2,4)
step(Tr_udot*180/pi)
grid on


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
[C_e_red_star, gamma_star, ~] = hinfstruct(P,C_e_red_star0, struct_options);
C_e_red_star = zpk(C_e_red_star);

T_wz_star = lft(P, C_e_red_star, 1, 1);
gamma_1_star =  norm(T_wz_star(1,1),'inf');
gamma_2_star =  norm(T_wz_star(2,1),'inf');
gamma_3_star =  norm(T_wz_star(3,1),'inf');

figure
sigma(T_wz_star, T_wz_star(1,1), T_wz_star(2,1), T_wz_star(3,1), sigma_opts)
legend('T_wz(s)', 'T_wz_1(s)', 'T_wz_2(s)', 'T_wz_3(s)')
grid on

C_i_red_star = s* C_e_red_star;
figure;
bode(C_i_min, 'b*', C_i_red, 'r', C_i_red_star, 'm')
grid on
legend('C_{i_min}', 'C_{i_red}', 'C_{i_{red_star}}')

%% Q3D2

%Assinging controller values
C_i = s* C_e_red_star;
F_f = 1;

%Retrieveing matrix transfer function
T = linearize('ClosedLoop_Test');
T = zpk(T);

%Isolating required transfer functions
So_star = T(1,1);
Ce_So_star = T(2,1);
To_star = T(3,1);
Tm_star = T(4,1);
Tr_udot_star = T(6,1);
Ti_star = -T(2,2); %Need to plot Ti
So_G_star = T(3,2);
Si_star = T(5,2);

%First figure
figure;
subplot(2,3,1);

sigma(inv(W1), 'r', So_star, 'b*', Si_star, 'b', sigma_opts)
grid on


subplot(2,3,2);
sigma(inv(W2), 'r', Ce_So_star, 'b', sigma_opts)
grid on


subplot(2,3,3);
sigma(inv(W3), 'r', Tm_star, 'b', sigma_opts)
grid on


subplot(2,3,4);
sigma(Ti_star, 'b*', To_star, 'b', sigma_opts)
grid on

subplot(2,3,5);
sigma(So_G_star, 'b', sigma_opts)
grid on

subplot(2,3,6);
sigma(C_i_red/s, 'b', C_e_red_star, 'b', sigma_opts)
grid on

%% Q3D part 2:
T_OL_star = linearize("OpenLoop_Test");

[GM_star, PM_star, omega_cg_star, omega_cp_star] = margin(T_OL_star);
DM_star = deg2rad(PM_star)/ omega_cp_star * 1000;

figure;
bode(T_OL_star)
grid on

%% Q3D part 3

figure;
subplot(2,2,1)
step(So_star)
grid on

subplot(2,2,2)
step(T_d, 'r',  To_star, 'b')
grid on

subplot(2,2,3)
step(So_G_star)
grid on

subplot(2,2,4)
step(Tr_udot_star*180/pi)
grid on

%% Q3E


F_f0 = T_d /minreal(To_star);

F_f0_zpk = zpk(F_f0);

[z_F, p_F, k_F] = zpkdata(F_f0_zpk, 'v');

%Selecting poles and zeroes based on inspection
z_F_del = z_F([1 3 4]);
p_F_del = p_F(3);
z_F([1 3 4]) = [];
k_F = k_F/abs(p_F_del(1)) * abs(z_F_del(1)) * abs(z_F_del(2)) * abs(z_F_del(3));
p_F(3) = [];
F_f_lf = zpk(z_F, p_F, k_F);

%Checking wheteher gain is restored
dcgain_f0 = dcgain(F_f0_zpk);
dcgain_f = dcgain(F_f_lf);
sigma_opts.XLim = [1e0, 0.1e3];

% figure;
% sigma(F_f0_zpk, F_f_lf, sigma_opts)
% grid on

[F_f, info] = balred(F_f_lf, 3);

% figure;
% bode(F_f, F_f_lf)
% grid on



%% Q3E.3


%Assinging controller values
C_i = s* C_e_red_star;

%Retrieveing matrix transfer function
T_F= linearize('ClosedLoop_Test');
T_F = zpk(T);

%Isolating required transfer functions
So_F = T_F(1,1);
Ce_So_F = T_F(2,1);
To_F = T_F(3,1);
Tm_F = T_F(4,1);
Tr_udot_F = T_F(6,1);
Ti_F = -T_F(2,2); %Need to plot Ti
So_G_F = T_F(3,2);
Si_F = T_F(5,2);

figure;
subplot(2,2,1)
sigma(inv(W3), 'r', Tm, 'b', Tm_star, 'm', Tm_F, 'g')
grid on
legend('W3^(-1)', 'hinfsyn', 'hinfstruct', 'feedforward')

subplot(2,2,3)
sigma(zpk(C_i_pm60), 'r', C_i_red, 'b', C_e_red_star, 'm', F_f, 'g')
grid on
legend('C_i_pm60', 'hinfsyn', 'hinfstruct', 'feedforward')

subplot(2,2,2)
step(T_d, 'r', To, 'b', To_star, 'm', To_F, 'g')
grid on
legend('T_d', 'hinfsyn', 'hinfstruct', 'feedforward')

subplot(2,2,4)
step(Tr_udot*180/pi, 'r', Tr_udot_star*180/pi, 'b', Tr_udot_F*180/pi, 'm')
grid on
legend('hinfsyn', 'hinfstruct', 'feedforward')

figure;
sigma(F_f)
grid on