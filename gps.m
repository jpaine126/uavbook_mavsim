% gps.m
%   Compute the output of gps sensor
%
%  Revised:
%   3/5/2010 - RB 
%   5/14/2010 - RB

function y = gps(uu, P)

    % relabel the inputs
    Va      = uu(1);
%    alpha   = uu(2);
%    beta    = uu(3);
    wn      = uu(4);
    we      = uu(5);
%    wd      = uu(6);
    pn      = uu(7);
    pe      = uu(8);
    pd      = uu(9);
%    u       = uu(10);
%    v       = uu(11);
%    w       = uu(12);
%    phi     = uu(13);
%    theta   = uu(14);
    psi     = uu(15);
%    p       = uu(16);
%    q       = uu(17);
%    r       = uu(18);
    t       = uu(19);
    
    Ts_gps = P.Ts_gps;
    k_gps = P.k_gps;
    
    sigma_gps_n = P.sigma_gps_n;
    sigma_gps_e = P.sigma_gps_e;
    sigma_gps_d = P.sigma_gps_d;
    
    sigma_gps_V = P.sigma_gps_V;
    
    persistent v_n;
    persistent v_e;
    persistent v_d;
    
    if t == 0
        v_n = 0;
        v_e = 0;
        v_d = 0;
    end
 
    % construct North, East, and altitude GPS measurements
    
    % update position noise
    eta_gps_n = normrnd(0, sigma_gps_n);
    eta_gps_e = normrnd(0, sigma_gps_e);
    eta_gps_d = normrnd(0, sigma_gps_d);
    
    v_n = exp(-k_gps*Ts_gps)*v_n + eta_gps_n;
    v_e = exp(-k_gps*Ts_gps)*v_e + eta_gps_e;
    v_d = exp(-k_gps*Ts_gps)*v_d + eta_gps_d;
    
    y_gps_n = pn + v_n;
    y_gps_e = pe + v_e; 
    y_gps_h = -pd + v_d; 
    
    % construct groundspeed and course measurements
    V_n = Va*cos(psi) + wn;
    V_e = Va*sin(psi) + we;
    
    V_g = sqrt(V_n^2 + V_e^2);
    
    sigma_gps_chi = sigma_gps_V / V_g;
    
    eta_V = normrnd(0, sigma_gps_V);
    eta_chi = normrnd(0, sigma_gps_chi);
    
    y_gps_Vg     = V_g + eta_V;
    y_gps_course = atan2(V_e, V_n) + eta_chi;

    % construct total output
    y = [...
        y_gps_n;...
        y_gps_e;...
        y_gps_h;...
        y_gps_Vg;...
        y_gps_course;...
        ];
    
end



