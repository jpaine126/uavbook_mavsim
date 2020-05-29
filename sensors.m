% sensors.m
%   Compute the output of rate gyros, accelerometers, and pressure sensors
%
%  Revised:
%   3/5/2010  - RB 
%   5/14/2010 - RB

function y = sensors(uu, P)

    % relabel the inputs
%    pn      = uu(1);
%    pe      = uu(2);
    pd      = uu(3);
%    u       = uu(4);
%    v       = uu(5);
%    w       = uu(6);
    phi     = uu(7);
    theta   = uu(8);
%    psi     = uu(9);
    p       = uu(10);
    q       = uu(11);
    r       = uu(12);
    F_x     = uu(13);
    F_y     = uu(14);
    F_z     = uu(15);
%    M_l     = uu(16);
%    M_m     = uu(17);
%    M_n     = uu(18);
    Va      = uu(19);
%    alpha   = uu(20);
%    beta    = uu(21);
%    wn      = uu(22);
%    we      = uu(23);
%    wd      = uu(24);
    
g = P.gravity;
m = P.mass;
rho = P.rho;

sigma_gyro = P.sigma_gyro;

sigma_accel = P.sigma_accel;

bias_diff_pres = P.bias_diff_pres;
bias_static_pres = P.bias_static_pres;
sigma_diff_pres = P.sigma_diff_pres;
sigma_static_pres = P.sigma_static_pres;

    % simulate rate gyros (units are rad/sec)
    eta_gyro_x = normrnd(0, sigma_gyro);
    eta_gyro_y = normrnd(0, sigma_gyro);
    eta_gyro_z = normrnd(0, sigma_gyro);
    
    y_gyro_x = p + eta_gyro_x;
    y_gyro_y = q + eta_gyro_y;
    y_gyro_z = r + eta_gyro_z;

    % simulate accelerometers (units of g)
    eta_accel_x = normrnd(0, sigma_accel);
    eta_accel_y = normrnd(0, sigma_accel);
    eta_accel_z = normrnd(0, sigma_accel);
    
    y_accel_x = (F_x/m) + g*sin(theta) + eta_accel_x;
    y_accel_y = (-F_y/m) - g*cos(theta)*sin(phi) + eta_accel_y;
    y_accel_z = (-F_z/m) - g*cos(theta)*cos(phi) + eta_accel_z;

    y_accel_y = y_accel_y;
    y_accel_z = -y_accel_z;
    % simulate pressure sensors
    
    beta_diff_pres = normrnd(0, bias_diff_pres);
    beta_static_pres = normrnd(0, bias_static_pres);
    eta_diff_pres = normrnd(beta_diff_pres, sigma_diff_pres);
    eta_static_pres = normrnd(beta_static_pres, sigma_static_pres);
    
    y_static_pres = rho*g*pd + eta_static_pres;
    y_diff_pres = (rho*Va^2)/2 + eta_diff_pres;

    % construct total output
    y = [...
        y_gyro_x;...
        y_gyro_y;...
        y_gyro_z;...
        y_accel_x;...
        y_accel_y;...
        y_accel_z;...
        y_static_pres;...
        y_diff_pres;...
    ];

end



