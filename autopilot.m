function [y] = autopilot(uu,P)
%
% autopilot for mavsim
% 
% Modification History:
%   2/11/2010 - RWB
%   5/14/2010 - RWB
%   9/30/2014 - RWB
%   

    % process inputs
    NN = 0;
%    pn       = uu(1+NN);  % inertial North position
%    pe       = uu(2+NN);  % inertial East position
    h        = uu(3+NN);  % altitude
    Va       = uu(4+NN);  % airspeed
%    alpha    = uu(5+NN);  % angle of attack
%    beta     = uu(6+NN);  % side slip angle
    phi      = uu(7+NN);  % roll angle
    theta    = uu(8+NN);  % pitch angle
    chi      = uu(9+NN);  % course angle
    p        = uu(10+NN); % body frame roll rate
    q        = uu(11+NN); % body frame pitch rate
    r        = uu(12+NN); % body frame yaw rate
%    Vg       = uu(13+NN); % ground speed
%    wn       = uu(14+NN); % wind North
%    we       = uu(15+NN); % wind East
%    psi      = uu(16+NN); % heading
%    bx       = uu(17+NN); % x-gyro bias
%    by       = uu(18+NN); % y-gyro bias
%    bz       = uu(19+NN); % z-gyro bias
    NN       = NN+19;
    Va_c     = uu(1+NN);  % commanded airspeed (m/s)
    h_c      = uu(2+NN);  % commanded altitude (m)
    chi_c    = uu(3+NN);  % commanded course (rad)
    NN       = NN+3;
    t        = uu(1+NN);   % time
    
    autopilot_version = 3;
        % autopilot_version == 1 <- used for tuning
        % autopilot_version == 2 <- PID with state machine takeoff
        % autopilot_version == 3 <- just PID
    switch autopilot_version
        case 1
           [delta, x_command] = autopilot_tuning(Va_c,h_c,chi_c,Va,h,chi,phi,theta,p,q,r,t,P);
        case 2
           [delta, x_command] = autopilot_uavbook(Va_c,h_c,chi_c,Va,h,chi,phi,theta,p,q,r,t,P);
        case 3
           [delta, x_command] = autopilot_kalman_tests(Va_c,h_c,chi_c,Va,h,chi,phi,theta,p,q,r,t,P);
    end
    y = [delta; x_command];
end
    
   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Autopilot versions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% autopilot_tuning
%   - used to tune each loop
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [delta, x_command] = autopilot_tuning(Va_c,h_c,chi_c,Va,h,chi,phi,theta,p,q,r,t,P)

    mode = 3;
    switch mode
        case 1 % tune the roll loop
            phi_c = chi_c; % interpret chi_c to autopilot as course command
            if t==0
                delta_a = roll_hold(phi_c, phi, p, 1, P);
            else
                delta_a = roll_hold(phi_c, phi, p, 0, P);
            end
            delta_r = 0; % no rudder
            % use trim values for elevator and throttle while tuning the lateral autopilot
            delta_e = P.u_trim(1);
            delta_t = P.u_trim(4);
            theta_c = 0;
        case 2 % tune the course loop
            if t==0
                phi_c   = course_hold(chi_c, chi, r, 1, P);
                delta_a = roll_hold(phi_c, phi, p, 1, P);
            else
                phi_c   = course_hold(chi_c, chi, r, 0, P);
                delta_a = roll_hold(phi_c, phi, p, 0, P);
            end                
            delta_r = 0; % no rudder
            % use trim values for elevator and throttle while tuning the lateral autopilot
            delta_e = P.u_trim(1);
            delta_t = P.u_trim(4);
            theta_c = 0;
        case 3 % tune the throttle to airspeed loop and pitch loop simultaneously
            theta_c = 20*pi/180 + h_c;
            chi_c = 0;
            if t==0
                phi_c   = course_hold(chi_c, chi, r, 1, P);
                delta_t = airspeed_with_throttle_hold(Va_c, Va, 1, P);
                delta_a = roll_hold(phi_c, phi, p, 1, P);
           else
                phi_c   = course_hold(chi_c, chi, r, 0, P);
                delta_t = airspeed_with_throttle_hold(Va_c, Va, 0, P);
                delta_a = roll_hold(phi_c, phi, p, 0, P);
            end
            delta_e = pitch_hold(theta_c, theta, q, P);
            delta_r = 0; % no rudder
            % use trim values for elevator and throttle while tuning the lateral autopilot
        case 4 % tune the pitch to airspeed loop 
            chi_c = 0;
            delta_t = P.u_trim(4);
            if t==0
                phi_c   = course_hold(chi_c, chi, r, 1, P);
                theta_c = airspeed_with_pitch_hold(Va_c, Va, 1, P);
                delta_a = roll_hold(phi_c, phi, p, 1, P);
           else
                phi_c   = course_hold(chi_c, chi, r, 0, P);
                theta_c = airspeed_with_pitch_hold(Va_c, Va, 0, P);
                delta_a = roll_hold(phi_c, phi, p, 0, P);
            end
            delta_e = pitch_hold(theta_c, theta, q, P);
            delta_r = 0; % no rudder
            % use trim values for elevator and throttle while tuning the lateral autopilot
        case 5 % tune the pitch to altitude loop 
            chi_c = 0;
            if t==0
                phi_c   = course_hold(chi_c, chi, r, 1, P);
                theta_c = altitude_hold(h_c, h, 1, P);
                delta_t = airspeed_with_throttle_hold(Va_c, Va, 1, P);
                delta_a = roll_hold(phi_c, phi, p, 1, P);
           else
                phi_c   = course_hold(chi_c, chi, r, 0, P);
                theta_c = altitude_hold(h_c, h, 0, P);
                delta_t = airspeed_with_throttle_hold(Va_c, Va, 0, P);
                delta_a = roll_hold(phi_c, phi, p, 0, P);
            end
            delta_e = pitch_hold(theta_c, theta, q, P);
            delta_r = 0; % no rudder
            % use trim values for elevator and throttle while tuning the lateral autopilot
        case 6
            delta_e = P.u_trim(1);
            delta_t = P.u_trim(4);
            delta_a = P.u_trim(2);
            phi_c = phi;
            theta_c = theta;
            delta_r = 0; % no rudder
    end
    %----------------------------------------------------------
    % create outputs
    
    % control outputs
    delta = [delta_e; delta_a; delta_r; delta_t];
    % commanded (desired) states
    x_command = [...
        0;...                    % pn
        0;...                    % pe
        h_c;...                  % h
        Va_c;...                 % Va
        0;...                    % alpha
        0;...                    % beta
        phi_c;...                % phi
        %theta_c*P.K_theta_DC;... % theta
        theta_c;
        chi_c;...                % chi
        0;...                    % p
        0;...                    % q
        0;...                    % r
        ];
            
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% autopilot_uavbook
%   - autopilot defined in the uavbook
%   - Utilizes PID for control surfaces and a state machine
%     to perform a takeoff procedure.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [delta, x_command] = autopilot_uavbook(Va_c,h_c,chi_c,Va,h,chi,phi,theta,p,q,r,t,P)

    %----------------------------------------------------------
    % longitudinal autopilot
    
    % define persistent variable for state of altitude state machine
    persistent altitude_state;
    persistent initialize_integrator;
    % initialize persistent variable
    if t==0
        initialize_integrator = 1;
        [~] = airspeed_with_pitch_hold(Va_c, Va, initialize_integrator, P);
        [~] = airspeed_with_throttle_hold(Va_c, Va, initialize_integrator, P);
        [~] = altitude_hold(h_c, h, initialize_integrator, P);
    else
        initialize_integrator = 0;
    end
    
    %
    if h<=P.altitude_take_off_zone  
        altitude_state = 1;
    elseif h<=h_c-P.altitude_hold_zone
        altitude_state = 2;
    elseif h>=h_c+P.altitude_hold_zone
        altitude_state = 3;
    else
        altitude_state = 4;
    end
    
    
    % implement state machine
    switch altitude_state
        case 1  % in take-off zone
            delta_t = 3;
            theta_c = 30*pi/180;
            
        case 2  % climb zone
            delta_t = 3; 
            theta_c = airspeed_with_pitch_hold(Va_c, Va, initialize_integrator, P);
            
        case 3 % descend zone
            delta_t = 0;
            theta_c = airspeed_with_pitch_hold(Va_c, Va, initialize_integrator, P);
            
        case 4 % altitude hold zone
            delta_t = airspeed_with_throttle_hold(Va_c, Va, initialize_integrator, P);
            theta_c = altitude_hold(h_c, h, initialize_integrator, P);
            
    end
    
    
    phi_c   = course_hold(chi_c, chi, r, initialize_integrator, P);
    delta_e = pitch_hold(theta_c, theta, q, P);
    delta_a = roll_hold(phi_c, phi, p, initialize_integrator, P);
    
    % artificially saturation delta_t
    delta_t = sat(delta_t,1.5,0);
    delta_r = 0;
    %----------------------------------------------------------
    % create outputs
    
    % control outputs
    delta = [delta_e; delta_a; delta_r; delta_t];
    % commanded (desired) states
    x_command = [...
        0;...                    % pn
        0;...                    % pe
        h_c;...                  % h
        Va_c;...                 % Va
        0;...                    % alpha
        0;...                    % beta
        phi_c;...                % phi
        %theta_c*P.K_theta_DC;... % theta
        theta_c;
        chi_c;...                % chi
        0;...                    % p
        0;...                    % q
        0;...                    % r
        ];
            
    y = [delta; x_command];
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% autopilot
%   - Just foolows the setpoints 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [delta, x_command] = autopilot_kalman_tests(Va_c,h_c,chi_c,Va,h,chi,phi,theta,p,q,r,t,P)
    
    % define persistent variable for state of altitude state machine
    persistent initialize_integrator;
    % initialize persistent variable
    if t==0
        initialize_integrator = 1;
    else
        initialize_integrator = 0;
    end
    
    delta_t = airspeed_with_throttle_hold(Va_c, Va, initialize_integrator, P);
    theta_c = altitude_hold(h_c, h, initialize_integrator, P);

    phi_c   = course_hold(chi_c, chi, r, initialize_integrator, P);
    
    delta_e = pitch_hold(theta_c, theta, q, P);
    delta_a = roll_hold(phi_c, phi, p, initialize_integrator, P);
    
    % artificially saturation delta_t
    delta_t = sat(delta_t,1.5,0);
    delta_r = 0;
    %----------------------------------------------------------
    % create outputs
    
    % control outputs
    delta = [delta_e; delta_a; delta_r; delta_t];
    % commanded (desired) states
    x_command = [...
        0;...                    % pn
        0;...                    % pe
        h_c;...                  % h
        Va_c;...                 % Va
        0;...                    % alpha
        0;...                    % beta
        phi_c;...                % phi
        %theta_c*P.K_theta_DC;... % theta
        theta_c;
        chi_c;...                % chi
        0;...                    % p
        0;...                    % q
        0;...                    % r
        ];
            
    y = [delta; x_command];
 
end
   


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Autopilot functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function delta_a = roll_hold(phi_c, phi, p, flag, P)
    persistent integrator;
    persistent error_d1;
    persistent differentiator;
    persistent error_der_dl;

    if flag == 1
        integrator = 0;
        differentiator = 0;
        error_d1 = 0;
        error_der_dl = 0;
    end
    
    kp_phi = P.kp_phi;
    ki_phi = P.ki_phi;
    kd_phi = P.kd_phi;
    Ts = P.Ts;
    tau = P.tau;
    error = phi_c - phi;
    error_der = error*kp_phi - p*kd_phi;
    
    integrator = integrator + (Ts/2)*(error+error_d1);
    
    differentiator = (2*tau-Ts)/(2*tau+Ts)*differentiator...
+ 2/(2*tau+Ts)*(error_der - error_der_dl);
    
    error_d1 = error;
        
    delta_a = sat(kp_phi*error + ki_phi*integrator - p*kd_phi,...%+ kd_phi*differentiator,...
        45*pi/180, -45*pi/180);
    
    if ki_phi ~= 0
        u_unsat = kp_phi*error + ki_phi*integrator - p*kd_phi;
        integrator = integrator + Ts/ki_phi * (delta_a - u_unsat);
    end
    
   
end

function phi_c = course_hold(chi_c, chi, r, flag, P)
    persistent integrator;
    persistent error_d1;
    if flag == 1
        integrator = 0;
        error_d1 = 0; % _d1 means delayed by one time step
        error_der_dl = 0;
    end
    kp_chi = P.kp_chi;
    ki_chi = P.ki_chi;
    kd_chi = P.kd_chi;
    Ts = P.Ts;
    error = chi_c - chi;
    
    integrator = integrator + (Ts/2)*(error+error_d1);
    
    error_d1 = error;
    
    phi_c = sat(kp_chi*error + ki_chi*integrator + kd_chi*r,...
        30*pi/180, -30*pi/180);
    
    if ki_chi ~= 0
        u_unsat = kp_chi*error + ki_chi*integrator + kd_chi*r;
        integrator = integrator + Ts/ki_chi * (phi_c - u_unsat);
    end

end



function delta_t = airspeed_with_throttle_hold(Va_c, Va, flag, P)
    persistent integrator;
    persistent error_d1;
    if flag == 1
        integrator = 0;
        error_d1 = 0;
    end
    kp_V = P.kp_V;
    ki_V = P.ki_V;
    Ts = P.Ts;
    error = Va_c - Va;
    
    integrator = integrator + (Ts/2)*(error+error_d1);
    
    error_d1 = error;
    
    delta_t = sat(P.u_trim(4) + kp_V*error + ki_V*integrator,...
        3, 0);
    
    if ki_V ~= 0
        u_unsat = P.u_trim(4) + kp_V*error + ki_V*integrator;
        integrator = integrator + Ts/ki_V * (delta_t - u_unsat);
    end
end


function delta_e = pitch_hold(theta_c, theta, q, P)
    kp_theta = P.kp_theta;
    kd_theta = P.kd_theta;

    error = theta_c - theta;
            
    delta_e = sat(kp_theta*error + kd_theta*q,...
        45*pi/180, -45*pi/180);
    
end

function theta_c = airspeed_with_pitch_hold(Va_c, Va, flag, P)
    persistent integrator;
    persistent error_d1;
    if flag == 1
        integrator = 0;
        error_d1 = 0;
    end
    kp_V2 = P.kp_V2;
    ki_V2 = P.ki_V2;
    Ts = P.Ts;
    error = Va_c - Va;
    
    integrator = integrator + (Ts/2)*(error+error_d1);
    
    error_d1 = error;
    
    theta_c = sat(kp_V2*error + ki_V2*integrator,...
        45*pi/180, -45*pi/180);
    
    if ki_V2 ~= 0
        u_unsat = kp_V2*error + ki_V2*integrator;
        integrator = integrator + Ts/ki_V2 * (theta_c - u_unsat);
    end
end

function theta_c = altitude_hold(h_c, h, flag, P)
    persistent integrator;
    persistent error_d1;
    if flag == 1
        integrator = 0;
        error_d1 = 0;
    end
    kp_h = P.kp_h;
    ki_h = P.ki_h;
    Ts = P.Ts;
    error = h_c - h;
    
    integrator = integrator + (Ts/2)*(error+error_d1);
    
    error_d1 = error;
    
    theta_c = sat(kp_h*error + ki_h*integrator,...
        30*pi/180, -30*pi/180);
    
    if ki_h ~= 0
        u_unsat = kp_h*error + ki_h*integrator;
        integrator = integrator + Ts/ki_h * (theta_c - u_unsat);
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sat
%   - saturation function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function out = sat(in, up_limit, low_limit)
  if in > up_limit
      out = up_limit;
  elseif in < low_limit
      out = low_limit;
  else
      out = in;
  end
end
  
 