% Jeffrey Paine
% ME621 Project Part 6

% TF Coeffs
[num, den] = tfdata(T_phi_delta_a, 'v');
a_phi2 = num(3);
a_phi1 = den(2);

[num, den] = tfdata(T_theta_delta_e, 'v');
a_theta1 = den(2);
a_theta2 = den(3);
a_theta3 = num(3);

[num, den] = tfdata(T_Va_delta_t, 'v');
a_V1 = den(2);
a_V2 = num(2);

[num, den] = tfdata(T_Va_theta, 'v');
a_V3 = -num(2);


%% Roll Loop Gains
% Input
delta_a_max = 30;
e_phi_max = 15;

% Tuning Parameters
zeta_phi = 1.5;
ki_phi = 0.1;

%  Natural Frequency
w_n_phi = sqrt( (delta_a_max / e_phi_max)*abs(a_phi2) );

% Gains
kp_phi = (delta_a_max / e_phi_max)*sign(a_phi2);

kd_phi = (2*zeta_phi*w_n_phi - a_phi1) / a_phi2;

%% Course Loop Gains
% Input

% Tuning Parameters
zeta_chi = .9;
W_chi = 15;

% Natural Frequency
w_n_chi = w_n_phi / W_chi;

% Gains
Vg = P.Va0;

kp_chi = (2*zeta_chi*w_n_chi*Vg) / P.gravity;

ki_chi = (w_n_chi^2)*Vg / P.gravity;

kd_chi = 0;

%% Sideslip Loop Gains



%% Pitch Loop Gains
% Input

delta_e_max = 45 *pi/180;
e_theta_max = 10 *pi/180;

% Tuning Parameters
zeta_theta = .9;

% Natural Frequency
w_n_theta = sqrt( ((delta_e_max/e_theta_max)*abs(a_theta3)) + a_theta2);

% Gains
kp_theta = (delta_e_max/e_theta_max)*sign(a_theta3);
kd_theta = -(2*zeta_theta*w_n_theta - a_theta1) / a_theta3;

%% Altitude from Pitch Loop Gains
% Input
K_theta_DC = (kp_theta*a_theta3) / (a_theta2 + kp_theta*a_theta3);

% Tuning Parameters
zeta_h = 0.7;
W_h = 18;

% Natural Frequency
w_n_h = (1/W_h) * w_n_theta;

% Gains
ki_h = (w_n_h^2) / (K_theta_DC*Vg);
kp_h = (2*zeta_h*w_n_h) / (K_theta_DC*Vg);

%% Airspeed from Pitch Loop Gains
% Input


% Tuning Parameters
zeta_V2 = .9;
W_V2 = 15;

% Natural Frequency
w_n_V2 = (1/W_V2) * w_n_theta;

% Gains

ki_V2 = -(w_n_V2^2) / (K_theta_DC*P.gravity);

kp_V2 = (a_V1 - 2*zeta_V2*w_n_V2) / (K_theta_DC*P.gravity);

%% Airspeed from Throttle Loop Gains
% Input

% Tuning Parameters
zeta_V = 1;
W_V = 4;

% Natural Frequency
w_n_V = (1/W_V) * w_n_V2;

% Gains
ki_V = (w_n_V^2) / a_V2;

kp_V = (2*zeta_V*w_n_V - a_V1) / a_V2;

%% Set Variable

P.kp_phi = kp_phi;
P.kd_phi = kd_phi;
P.ki_phi = ki_phi;

P.kp_chi = kp_chi;
P.ki_chi = ki_chi;
P.kd_chi = kd_chi;

P.kp_theta = kp_theta;
P.kd_theta = kd_theta;

P.ki_h = ki_h;
P.kp_h = kp_h;

P.ki_V2 = ki_V2;
P.kp_V2 = kp_V2;

P.ki_V = ki_V;
P.kp_V = kp_V;