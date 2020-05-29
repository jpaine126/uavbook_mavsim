clear;clc;

P.gravity = 9.8;
   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Params for Zagi Flying Wing
%physical parameters of airframe
P.mass = 1.56;
P.Jx   = 0.1147;
P.Jy   = 0.0576;
P.Jz   = 0.1712;
P.Jxz  = 0.0015;
% aerodynamic coefficients
P.S_wing        = 0.2589;
P.b             = 1.4224;
P.c             = 0.3302;
P.S_prop        = 0.0314;
P.rho           = 1.2682;
P.k_motor       = 20;
P.k_T_P         = 0;
P.k_Omega       = 0;
P.e             = 0.9;
P.AR            = P.b^2/P.S_wing;

P.C_L_0         = 0.28;
P.C_L_alpha     = 3.45;
P.C_L_q         = 0.0;
P.C_L_delta_e   = -0.36;
P.C_D_0         = 0.03;
P.C_D_alpha     = 0.30;
P.C_D_p         = 0.0437;
P.C_D_q         = 0.0;
P.C_D_delta_e   = 0.0;
P.C_m_0         = -0.02338;
P.C_m_alpha     = -0.38;
P.C_m_q         = -3.6;
P.C_m_delta_e   = -0.5;
P.C_Y_0         = 0.0;
P.C_Y_beta      = -0.98;
P.C_Y_p         = 0.0;
P.C_Y_r         = 0.0;
P.C_Y_delta_a   = 0.0;
P.C_Y_delta_r   = -0.17;
P.C_ell_0       = 0.0;
P.C_ell_beta    = -0.12;
P.C_ell_p       = -0.26;
P.C_ell_r       = 0.14;
P.C_ell_delta_a = 0.08;
P.C_ell_delta_r = 0.105;
P.C_n_0         = 0.0;
P.C_n_beta      = 0.25;
P.C_n_p         = 0.022;
P.C_n_r         = -0.35;
P.C_n_delta_a   = 0.06;
P.C_n_delta_r   = -0.032;
P.C_prop        = 1.0;
P.M             = 50;
P.epsilon       = 0.1592;
P.alpha0        = 0.4712;

P.Gamma   = P.Jx*P.Jz - P.Jxz^2;
P.Gamma_1 = P.Gamma \ (P.Jxz*(P.Jx-P.Jy+P.Jz));
P.Gamma_2 = P.Gamma\(P.Jz*(P.Jz-P.Jy)+P.Jxz^2);
P.Gamma_3 = P.Jz/P.Gamma;
P.Gamma_4 = P.Jxz/P.Gamma;
P.Gamma_5 = (P.Jz - P.Jx)/P.Jy;
P.Gamma_6 = P.Jxz/P.Jy;
P.Gamma_7 = P.Gamma\((P.Jx-P.Jy)*P.Jx+P.Jxz^2);
P.Gamma_8 = P.Jx/P.Gamma;

P.C_p_0         = P.Gamma_3*P.C_ell_0+P.Gamma_4*P.C_n_0;
P.C_p_beta      = P.Gamma_3*P.C_ell_beta+P.Gamma_4*P.C_n_beta;
P.C_p_p         = P.Gamma_3*P.C_ell_p+P.Gamma_4*P.C_n_p;
P.C_p_r         = P.Gamma_3*P.C_ell_r+P.Gamma_4*P.C_n_r;
P.C_p_delta_a   = P.Gamma_3*P.C_ell_delta_a+P.Gamma_4*P.C_n_delta_a;
P.C_p_delta_r   = P.Gamma_3*P.C_ell_delta_r+P.Gamma_4*P.C_n_delta_r;
P.C_r_0         = P.Gamma_4*P.C_ell_0+P.Gamma_8*P.C_n_0;
P.C_r_beta      = P.Gamma_4*P.C_ell_beta+P.Gamma_8*P.C_n_beta;
P.C_r_p         = P.Gamma_4*P.C_ell_p+P.Gamma_8*P.C_n_p;
P.C_r_r         = P.Gamma_4*P.C_ell_r+P.Gamma_8*P.C_n_r;
P.C_r_delta_a   = P.Gamma_4*P.C_ell_delta_a+P.Gamma_8*P.C_n_delta_a;
P.C_r_delta_r   = P.Gamma_4*P.C_ell_delta_r+P.Gamma_8*P.C_n_delta_r;

% wind parameters
P.wind_n = 0;%3;
P.wind_e = 0;%1;
P.wind_d = 0;
P.L_u = 200;
P.L_v = 200;
P.L_w = 50;
P.sigma_u = 1; 
P.sigma_v = 1;
P.sigma_w = .7;


% compute trim conditions using 'mavsim_chap5_trim.slx'
% initial airspeed
P.Va0 = 15;
gamma = 0*pi/180;  % desired flight path angle (radians)
R     = inf;       % desired radius (m) - use (+) for right handed orbit, 

% autopilot sample rate
P.Ts = 0.01;
P.Tout = 0.001;

% first cut at initial conditions
P.pn0    = 0;  % initial North position
P.pe0    = 0;  % initial East position
P.pd0    = -100;%0;  % initial Down position (negative altitude)
P.u0     = P.Va0; % initial velocity along body x-axis
P.v0     = 0;  % initial velocity along body y-axis
P.w0     = 0;  % initial velocity along body z-axis
P.phi0   = 0;  % initial roll angle
P.theta0 = 0;  % initial pitch angle
P.psi0   = 0;  % initial yaw angle
P.p0     = 0;  % initial body frame roll rate
P.q0     = 0;  % initial body frame pitch rate
P.r0     = 0;  % initial body frame yaw rate

% run trim commands
[x_trim, u_trim]=compute_trim('mavsim_trim',P.Va0,gamma,R);
P.u_trim = u_trim;
P.x_trim = x_trim;

% set initial conditions to trim conditions
% initial conditions
P.pn0    = 0;  % initial North position
P.pe0    = 0;  % initial East position
%P.pd0    = 0;  % initial Down position (negative altitude)
P.u0     = x_trim(4);  % initial velocity along body x-axis
P.v0     = x_trim(5);  % initial velocity along body y-axis
P.w0     = x_trim(6);  % initial velocity along body z-axis
P.phi0   = x_trim(7);  % initial roll angle
P.theta0 = x_trim(8);  % initial pitch angle
P.psi0   = x_trim(9);  % initial yaw angle
P.p0     = x_trim(10);  % initial body frame roll rate
P.q0     = x_trim(11);  % initial body frame pitch rate
P.r0     = x_trim(12);  % initial body frame yaw rate

% sensor parameters
P.sigma_gyro = 0.13 * pi/180;
P.sigma_accel = 0.0025;

P.bias_diff_pres = 0.02;
P.bias_static_pres = 0.125;
P.sigma_diff_pres = 0.002;
P.sigma_static_pres = 0.01;

P.Ts_gps = 1;
P.k_gps = 1/1100;
P.sigma_gps_e = 0.21;
P.sigma_gps_n = 0.21;
P.sigma_gps_d = 0.4;

P.sigma_gps_V = 0.01;


% Low Pass Filter Parameters

P.fc_gyro  = 5;
P.fc_sp    = 20;
P.fc_dp    = 150;
P.fc_phi   = 20;
P.fc_theta = 50;
P.fc_gps   = 1;

% Estimator Parameters

P.Q_att = [1 0;
           0 1]*0;
       
P.Q_gps = [1 0 0 0 0 0 0;
           0 1 0 0 0 0 0;
           0 0 1 0 0 0 0;
           0 0 0 1 0 0 0;
           0 0 0 0 1 0 0;
           0 0 0 0 0 1 0;
           0 0 0 0 0 0 1]*0.01;
       
P.R_att = [1 0 0;
           0 1 0;
           0 0 1]*P.sigma_accel^2;

P.R_gps = [0.21^2 0 0 0 0 0;
           0 0.21^2 0 0 0 0;
           0 0 0.01^2 0 0 0;
           0 0 0 0.01^2 0 0;
           0 0 0 0 0.21^2 0;
           0 0 0 0 0 0.21^2];       
       
% compute different transfer functions
[T_phi_delta_a,T_chi_phi,T_theta_delta_e,T_h_theta,T_h_Va,T_Va_delta_t,T_Va_theta,T_v_delta_r]...
    = compute_tf_model(x_trim,u_trim,P);

% linearize the equations of motion around trim conditions
[A_lon, B_lon, A_lat, B_lat] = compute_ss_model('mavsim_trim',x_trim,u_trim);

% calculate eigenvalues and omega_n/zeta for modes
eig_lon = eigs(A_lon);
syms s;

mode_1 = eig_lon(1:2);
mode_2 = eig_lon(3:4);

eq_1 = simplify( (s - mode_1(1))*(s - mode_1(2)) );
coeff = sym2poly(eq_1);
wn_1 = sqrt(coeff(3));
zeta_1 = coeff(2)/(2*wn_1);

eq_2 = simplify( (s - mode_2(1))*(s - mode_2(2)) );
coeff = sym2poly(eq_2);
wn_2 = sqrt(coeff(3));
zeta_2 = coeff(2)/(2*wn_2);

eig_lat = eigs(A_lat);

% gain on dirty derivative
P.tau = 5;

% autopilot gains
% altitude parameters and gains
P.altitude_take_off_zone = 10;
P.altitude_hold_zone = 10;
gain_calculations;

P.bias_gyro_x = 0;
P.bias_gyro_y = 0;
P.bias_gyro_z = 0; 
