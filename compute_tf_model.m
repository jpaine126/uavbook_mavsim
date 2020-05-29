function [T_phi_delta_a,T_chi_phi,T_theta_delta_e,T_h_theta,T_h_Va,T_Va_delta_t,T_Va_theta,T_v_delta_r]= compute_tf_model(x_trim,u_trim,P)
% x_trim is the trimmed state,
u_x     = x_trim(4);  % velocity along body x-axis
v_x     = x_trim(5);  % velocity along body y-axis
w_x     = x_trim(6);  % velocity along body z-axis
phi_x   = x_trim(7);  % roll angle
theta_x = x_trim(8);  % pitch angle
psi_x   = x_trim(9);  % yaw angle
p_x     = x_trim(10);  % body frame roll rate
q_x     = x_trim(11);  % body frame pitch rate
r_x     = x_trim(12);  % body frame yaw rate
Va_x    = P.Va0;
alpha = atan((w_x)/(u_x));
beta_x = asin(v_x/Va_x);
chi = atan2(Va_x*sin(phi_x), Va_x*cos(phi_x));
% u_trim is the trimmed input
delta_e_x =u_trim(1);
delta_a_x =u_trim(2);
delta_r_x =u_trim(3);
delta_t_x =u_trim(4);

%%
a_phi1 = -0.5*P.rho*Va_x^2*P.S_wing*P.b*P.C_p_p*P.b/(2*Va_x);
a_phi2 = 0.5*P.rho*Va_x^2*P.S_wing*P.b*P.C_p_delta_a;

a_theta1 = -P.rho*Va_x^2*P.c*P.S_wing/(2*P.Jy)*P.C_m_q*P.c/(2*Va_x);
a_theta2 = -P.rho*Va_x^2*P.c*P.S_wing/(2*P.Jy)*P.C_m_alpha;
a_theta3 = P.rho*Va_x^2*P.c*P.S_wing/(2*P.Jy)*P.C_m_delta_e;

a_V1 = P.rho*Va_x*P.S_wing/P.mass*[P.C_D_0+P.C_D_alpha*alpha+P.C_D_delta_e*delta_e_x];
a_V2 = P.rho*P.S_prop/P.mass*P.C_prop*P.k_motor^2*delta_t_x;
a_V3 = P.gravity*cos(theta_x-chi);

a_beta1 = -P.rho*Va_x*P.S_wing/2*P.mass*P.C_Y_beta;
a_beta2 = P.rho*Va_x*P.S_wing/2*P.mass*P.C_Y_delta_r;

%% define transfer functions
T_phi_delta_a   = tf([a_phi2],[1,a_phi1,0]);
T_chi_phi       = tf([P.gravity/Va_x],[1,0]);
T_theta_delta_e = tf(a_theta3,[1,a_theta1,a_theta2]);
T_h_theta       = tf([Va_x],[1,0]);
T_h_Va          = tf([theta_x],[1,0]);
T_Va_delta_t    = tf([a_V2],[1,a_V1]);
T_Va_theta      = tf([-a_V3],[1,a_V1]);
T_v_delta_r     = tf([a_beta2],[1,a_beta1]);
