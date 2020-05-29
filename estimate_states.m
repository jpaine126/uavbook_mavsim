% estimate_states
%   - estimate the MAV states using gyros, accels, pressure sensors, and
%   GPS.
%
% Outputs are:
%   pnhat    - estimated North position, 
%   pehat    - estimated East position, 
%   hhat     - estimated altitude, 
%   Vahat    - estimated airspeed, 
%   alphahat - estimated angle of attack
%   betahat  - estimated sideslip angle
%   phihat   - estimated roll angle, 
%   thetahat - estimated pitch angel, 
%   chihat   - estimated course, 
%   phat     - estimated roll rate, 
%   qhat     - estimated pitch rate, 
%   rhat     - estimated yaw rate,
%   Vghat    - estimated ground speed, 
%   wnhat    - estimate of North wind, 
%   wehat    - estimate of East wind
%   psihat   - estimate of heading angle
% 
% 
% Modified:  3/15/2010 - RB
%            5/18/2010 - RB
%

function xhat = estimate_states(uu, P)

    % rename inputs
    y_gyro_x      = uu(1);
    y_gyro_y      = uu(2);
    y_gyro_z      = uu(3);
    y_accel_x     = uu(4);
    y_accel_y     = uu(5);
    y_accel_z     = uu(6);
    y_static_pres = uu(7);
    y_diff_pres   = uu(8);
    y_gps_n       = uu(9);
    y_gps_e       = uu(10);
    y_gps_h       = uu(11);
    y_gps_Vg      = uu(12);
    y_gps_course  = uu(13);
    t             = uu(14);
   
    persistent xhat_mem;
    persistent y_mem;
    
    persistent P_att;
    persistent P_gps;
   
    % not estimating these states 
    alphahat = 0;
    betahat  = 0;
    bxhat    = 0;
    byhat    = 0;
    bzhat    = 0;
    
    rho = P.rho;
    g = P.gravity;
    
    fc_gyro  = P.fc_gyro;
    fc_sp    = P.fc_sp;
    fc_dp    = P.fc_dp;
    fc_phi   = P.fc_phi;
    fc_theta = P.fc_theta;
    fc_gps   = P.fc_gps;
     
    Q_att = P.Q_att;
    Q_gps = P.Q_gps;
    
    R_att = P.R_att;
    R_gps = P.R_gps;
    
    Ts = P.Ts;
    Ts_gps = P.Ts_gps;
    
    % 1 for lowpass only
    % 2 for kalman filters 
    mode = 2; 
   
    
    %% initialize
    if t == 0
        pnhat_mem = P.pn0;
        pehat_mem = P.pe0;
        hhat_mem = -P.pd0;
        Vahat_mem = P.Va0;
        phihat_mem = P.phi0;
        thetahat_mem = P.theta0;
        chihat_mem = 0;
        phat_mem = P.p0;
        qhat_mem = P.q0;
        rhat_mem = P.r0;
        Vghat_mem = Vahat_mem;
        wnhat_mem = 0;
        wehat_mem = 0;
        psihat_mem = 0;
        
        y_mem = uu(1:13);
        y_gyro_x_mem      = y_mem(1);
        y_gyro_y_mem      = y_mem(2);
        y_gyro_z_mem      = y_mem(3);
        y_accel_x_mem     = y_mem(4);
        y_accel_y_mem     = y_mem(5);
        y_accel_z_mem     = y_mem(6);
        y_static_pres_mem = y_mem(7);
        y_diff_pres_mem   = y_mem(8);
        y_gps_n_mem       = y_mem(9);
        y_gps_e_mem       = y_mem(10);
        y_gps_h_mem       = y_mem(11);
        y_gps_Vg_mem      = y_mem(12);
        y_gps_course_mem  = y_mem(13);
        
        xhat_mem = [pnhat_mem;pehat_mem;hhat_mem;Vahat_mem;alphahat;betahat;phihat_mem;thetahat_mem;chihat_mem;phat_mem;qhat_mem;rhat_mem;Vghat_mem;wnhat_mem;wehat_mem;psihat_mem;bxhat;byhat;bzhat];
        
        P_att = Q_att;
        P_gps = Q_gps;
    
    else % pull out variables from xhat
        pnhat_mem    = xhat_mem(1);
        pehat_mem    = xhat_mem(2);
        hhat_mem     = xhat_mem(3);
        Vahat_mem    = xhat_mem(4);
        alphahat_mem = xhat_mem(5);
        betahat_mem  = xhat_mem(6);
        phihat_mem   = xhat_mem(7);
        thetahat_mem = xhat_mem(8);
        chihat_mem   = xhat_mem(9);
        phat_mem     = xhat_mem(10);
        qhat_mem     = xhat_mem(11);
        rhat_mem     = xhat_mem(12);
        Vghat_mem    = xhat_mem(13);
        wnhat_mem    = xhat_mem(14);
        wehat_mem    = xhat_mem(15);
        psihat_mem   = xhat_mem(16);
        bxhat_mem    = xhat_mem(17);
        byhat_mem    = xhat_mem(18);
        bzhat_mem    = xhat_mem(19);
        
        y_gyro_x_mem      = y_mem(1);
        y_gyro_y_mem      = y_mem(2);
        y_gyro_z_mem      = y_mem(3);
        y_accel_x_mem     = y_mem(4);
        y_accel_y_mem     = y_mem(5);
        y_accel_z_mem     = y_mem(6);
        y_static_pres_mem = y_mem(7);
        y_diff_pres_mem   = y_mem(8);
        y_gps_n_mem       = y_mem(9);
        y_gps_e_mem       = y_mem(10);
        y_gps_h_mem       = y_mem(11);
        y_gps_Vg_mem      = y_mem(12);
        y_gps_course_mem  = y_mem(13);
    end


    %% low pass filters

    y_gyro_x_filt = lowpass(y_gyro_x, y_gyro_x_mem, fc_gyro*Ts);
    phat = y_gyro_x_filt;
    y_gyro_y_filt = lowpass(y_gyro_y, y_gyro_y_mem, fc_gyro*Ts);
    qhat = y_gyro_y_filt;
    y_gyro_z_filt = lowpass(y_gyro_z, y_gyro_z_mem, fc_gyro*Ts);
    rhat = y_gyro_z_filt;
    
    y_static_pres_filt = lowpass(y_static_pres, y_static_pres_mem, fc_sp*Ts);
    hhat = -y_static_pres_filt / (rho*g);
        
    y_diff_pres_filt = lowpass(y_diff_pres, y_diff_pres_mem, fc_dp*Ts);
    Vahat = sqrt( (2/rho) * y_diff_pres_filt );

    
    %% Attitude Estimation
    switch mode
        % low pass only
        case 1
                        
            y_accel_y_filt = lowpass(y_accel_y, y_accel_y_mem, fc_phi*Ts);
            y_accel_z_filt = lowpass(y_accel_z, y_accel_z_mem, fc_phi*Ts);
            phihat = -atan2(y_accel_y_filt, y_accel_z_filt);
                        
            y_accel_x_filt = lowpass(y_accel_x, y_accel_x_mem, fc_theta*Ts);
            thetahat = asin( y_accel_x_filt / g );
            
            if mod(t, Ts_gps) == 0                
                y_gps_n_filt = lowpass(y_gps_n, y_gps_n_mem, fc_gps*Ts_gps);
                pnhat = y_gps_n_filt;
                
                y_gps_e_filt = lowpass(y_gps_e, y_gps_e_mem, fc_gps*Ts_gps);
                pehat = y_gps_e_filt;

                y_gps_course_filt = lowpass(y_gps_course, y_gps_course_mem, fc_gps*Ts_gps);
                chihat = y_gps_course_filt;
                
                y_gps_Vg_filt = lowpass(y_gps_Vg, y_gps_Vg_mem, fc_gps*Ts_gps);
                Vghat = y_gps_Vg_filt;
            
            else
                pnhat = pnhat_mem;
                pehat = pehat_mem;
                chihat = chihat_mem;
                Vghat = Vghat_mem;
                
                y_gps_n_filt = y_gps_n_mem;
                y_gps_e_filt = y_gps_e_mem;
                y_gps_course_filt = y_gps_course_mem;
                y_gps_Vg_filt = y_gps_Vg_mem;
            end
            
            y_gps_h_filt = 0;
            
            wnhat = 0;
            wehat = 0;
            psihat = 0;

            % Kalman Filters for phi, theta
        case 2
            xhat_att = [phihat_mem thetahat_mem]';
            u_att = [phat qhat rhat Vahat]';
            y_att = [y_accel_x; y_accel_y; y_accel_z];

            xhat_att = xhat_att + Ts*f_att(xhat_att, u_att);
            A_att = A_attj(xhat_att, u_att);
            P_att = P_att + Ts*(A_att*P_att + P_att*A_att' + Q_att);

            % sensor update
            if mod(t, Ts) == 0
                C_att = C_attj(xhat_att, u_att, g);
                L_att = P_att*C_att'*inv(R_att + C_att*P_att*C_att');
                P_att = (eye(size(P_att)) - L_att*C_att) * P_att;
                xhat_att = xhat_att + L_att*(y_att - h_att(xhat_att, u_att, g));
            end
            
            phihat = xhat_att(1);
            thetahat = xhat_att(2);
            
            
            % kalman filter for pn, pe, Vg, chi, wn, we, psi
            
            xhat_gps = [pnhat_mem pehat_mem Vghat_mem chihat_mem wnhat_mem wehat_mem psihat_mem]';
            u_gps = [Vahat qhat rhat phihat thetahat]';
            y_gps = [y_gps_n;y_gps_e;y_gps_Vg;y_gps_course;Vahat*cos(psihat_mem) + wnhat_mem - Vghat_mem*cos(chihat_mem);Vahat*sin(psihat_mem) + wehat_mem - Vghat_mem*sin(chihat_mem)];
            
            xhat_gps = xhat_gps + Ts*f_gps(xhat_gps, u_gps, g);
            A_gps = A_gpsj(xhat_gps, u_gps, g);
            P_gps = P_gps + Ts*(A_gps*P_gps + P_gps*A_gps' + Q_gps);
            
            % sensor update
            if mod(t, Ts_gps) == 0
                C_gps = C_gpsj(xhat_gps, u_gps);
                L_gps = P_gps*C_gps'*inv(R_gps + C_gps*P_gps*C_gps');
                P_gps = (eye(size(P_gps)) - L_gps*C_gps) * P_gps;
                xhat_gps = xhat_gps + L_gps*(y_gps - h_gps(xhat_gps, u_gps));
            end
            
            pnhat  = xhat_gps(1);
            pehat  = xhat_gps(2);
            Vghat  = xhat_gps(3);
            chihat = xhat_gps(4);
            wnhat  = xhat_gps(5);
            wehat  = xhat_gps(6);
            psihat = xhat_gps(7);
            
            y_accel_x_filt = 0;
            y_accel_y_filt = 0;
            y_accel_z_filt = 0;
            
            y_gps_n_filt = 0;
            y_gps_e_filt = 0;
            y_gps_h_filt = 0;
            y_gps_Vg_filt = 0;
            y_gps_course_filt = 0;
            
    end
    %% Assemble xhat
    
    
    xhat = [...
        pnhat;...
        pehat;...
        hhat;...
        Vahat;...
        alphahat;...
        betahat;...
        phihat;...
        thetahat;...
        chihat;...
        phat;...
        qhat;...
        rhat;...
        Vghat;...
        wnhat;...
        wehat;...
        psihat;...
        bxhat;...
        byhat;...
        bzhat;...
    ];

    y_mem = [...
        y_gyro_x_filt;...
        y_gyro_y_filt;...
        y_gyro_z_filt;...
        y_accel_x_filt;...
        y_accel_y_filt;...
        y_accel_z_filt;...
        y_static_pres_filt;...
        y_diff_pres_filt;...
        y_gps_n_filt;...
        y_gps_e_filt;...
        y_gps_h_filt;...
        y_gps_Vg_filt;...
        y_gps_course_filt;...
        ];

    xhat_mem = xhat;
end
    
%% Utility Functions
function f = f_att(xhat_att, uhat_att)
    p = uhat_att(1);
    q = uhat_att(2);
    r = uhat_att(3);
    Va = uhat_att(4);
    phi = xhat_att(1);
    theta = xhat_att(2);
    
    f = [p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta);
         q*cos(phi) - r*sin(phi)];
end
    
function h = h_att(xhat_att, uhat_att, g)
    p = uhat_att(1);
    q = uhat_att(2);
    r = uhat_att(3);
    Va = uhat_att(4);
    phi = xhat_att(1);
    theta = xhat_att(2);
    
    
    h = [q*Va*sin(theta) + g*sin(theta);
         r*Va*cos(theta) - p*Va*sin(theta) - g*cos(theta)*sin(phi);
         -q*Va*cos(theta) - g*cos(theta)*cos(phi)];
end

function A = A_attj(xhat_att, uhat_att)
    q = uhat_att(2);
    r = uhat_att(3);
    phi = xhat_att(1);
    theta = xhat_att(2);
    
    A = [q*cos(phi)*tan(theta) - r*sin(phi)*tan(theta), (q*sin(phi) + r*cos(phi)) / cos(theta)^2;
        -q*sin(phi) - r*cos(phi)                   , 0];
    
end

function C = C_attj(xhat_att, uhat_att, g)
    p = uhat_att(1);
    q = uhat_att(2);
    r = uhat_att(3);
    Va = uhat_att(4);
    phi = xhat_att(1);
    theta = xhat_att(2);

    C = [0                    , q*Va*cos(theta) + g*cos(theta);
        -g*cos(phi)*cos(theta), -r*Va*sin(theta) - p*Va*cos(theta) + g*sin(phi)*sin(theta);
         g*sin(phi)*cos(theta), (q*Va + g*cos(phi))*sin(theta)];
end

function f = f_gps(xhat_gps, u_gps, g)
    
    pn = xhat_gps(1);
    pe = xhat_gps(2);
    Vg = xhat_gps(3);
    chi = xhat_gps(4);
    wn = xhat_gps(5);
    we = xhat_gps(6);
    psi = xhat_gps(7);
    
    Va = u_gps(1);
    q = u_gps(2);
    r = u_gps(3);
    phi = u_gps(4);
    theta = u_gps(5);
    
    psi_dot = q*(sin(phi)/cos(theta)) + r*(cos(phi)/cos(theta));
    
    f = [Vg*cos(chi);
         Vg*sin(chi);
       ((Va*cos(psi)+wn)*(-Va*psi_dot*sin(psi)) + (Va*sin(psi)+we)*(Va*psi_dot*cos(psi)))/Vg;
        (g/Vg)*tan(phi)*cos(chi-psi);
         0;
         0;
         psi_dot];
end

function h = h_gps(xhat_gps, u_gps)
    pn = xhat_gps(1);
    pe = xhat_gps(2);
    Vg = xhat_gps(3);
    chi = xhat_gps(4);
    wn = xhat_gps(5);
    we = xhat_gps(6);
    psi = xhat_gps(7);
    
    Va = u_gps(1);
    q = u_gps(2);
    r = u_gps(3);
    phi = u_gps(4);
    theta = u_gps(5);
    
    h = [pn;
         pe;
         Vg;
         chi;
         Va*cos(psi) + wn - Vg*cos(chi);
         Va*sin(psi) + we - Vg*sin(chi)];
    
end

function A = A_gpsj(xhat_gps, u_gps, g)
    pn = xhat_gps(1);
    pe = xhat_gps(2);
    Vg = xhat_gps(3);
    chi = xhat_gps(4);
    wn = xhat_gps(5);
    we = xhat_gps(6);
    psi = xhat_gps(7);
    
    Va = u_gps(1);
    q = u_gps(2);
    r = u_gps(3);
    phi = u_gps(4);
    theta = u_gps(5);
    
    psi_dot = q*(sin(phi)/cos(theta)) + r*(cos(phi)/cos(theta));
    Vg_dot = (Va/Vg)*( (cos(psi)+wn)*(-psi_dot*sin(psi)) + (sin(psi)+we)*(psi_dot*cos(psi)) );
    
    
    dVg_dot_psi = (-psi_dot*Va*(wn*cos(psi) + we*sin(psi))) / Vg;
    dchi_dot_Vg = -(g/Vg^2)*tan(phi)*cos(chi-psi);
    dchi_dot_chi = -(g/Vg)*tan(phi)*sin(chi-psi);
    dchi_dot_psi =  (g/Vg)*tan(phi)*sin(chi-psi);
    
    A = [0 0 cos(chi)    -Vg*sin(chi)   0                        0                        0;
         0 0 sin(chi)     Vg*cos(chi)   0                        0                        0;
         0 0 -Vg_dot/Vg   0            -psi_dot*Va*sin(psi)/Vg   psi_dot*Va*cos(psi)/Vg   dVg_dot_psi;
         0 0 dchi_dot_Vg  dchi_dot_chi  0                        0                        dchi_dot_psi;
         0 0 0            0             0                        0                        0;
         0 0 0            0             0                        0                        0;
         0 0 0            0             0                        0                        0];
    
end

function C = C_gpsj(xhat_gps, u_gps)
    pn = xhat_gps(1);
    pe = xhat_gps(2);
    Vg = xhat_gps(3);
    chi = xhat_gps(4);
    wn = xhat_gps(5);
    we = xhat_gps(6);
    psi = xhat_gps(7);
    
    Va = u_gps(1);
    q = u_gps(2);
    r = u_gps(3);
    phi = u_gps(4);
    theta = u_gps(5);
    
    C = [1 0  0         0            0 0  0;
         0 1  0         0            0 0  0;
         0 0  1         0            0 0  0;
         0 0  0         1            0 0  0;
         0 0 -cos(chi)  Vg*sin(chi)  1 0 -Va*sin(psi);
         0 0 -sin(chi) -Vg*cos(chi)  0 1  Va*cos(psi)];
    
end

function y = lowpass(this, last, f_c)
    alpha = (2*pi*f_c) / (2*pi*f_c + 1);
    y = last + alpha*(this - last);
end