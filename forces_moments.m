%Jeffrey Paine
%ME621 Project

% forces_moments.m
%   Computes the forces and moments acting on the airframe. 
%
%   Output is
%       F     - forces
%       M     - moments
%       Va    - airspeed
%       alpha - angle of attack
%       beta  - sideslip angle
%       wind  - wind vector in the inertial frame
%

function out = forces_moments(x, delta, wind, P)

    % relabel the inputs
    pn      = x(1);
    pe      = x(2);
    pd      = x(3);
    u       = x(4);
    v       = x(5);
    w       = x(6);
    phi     = x(7);
    theta   = x(8);
    psi     = x(9);
    p       = x(10);
    q       = x(11);
    r       = x(12);
    delta_e = delta(1);
    delta_a = delta(2);
    delta_r = delta(3);
    delta_t = delta(4);
    w_ns    = wind(1); % steady wind - North
    w_es    = wind(2); % steady wind - East
    w_ds    = wind(3); % steady wind - Down
    u_wg    = wind(4); % gust along body x-axis
    v_wg    = wind(5); % gust along body y-axis    
    w_wg    = wind(6); % gust along body z-axis
    
    % compute wind data in NED 
    ct = cos(theta);cu = cos(psi);cp = cos(phi);
    st = sin(theta);su = sin(psi);sp = sin(phi);
    %     t = tan(theta);
    R1 = [ct*cu , sp*st*cu - cp*su , cp*st*cu + sp*su;
        ct*su , sp*st*su + cp*cu , cp*st*su - sp*cu;
        - st   , sp*ct            , cp*ct]; 
    
    % body to vehicle
    R2 = R1'; 
    % vehicle to body
    
    % compute wind data in NED
    w_NED = R1*[u_wg; v_wg; w_wg] + [w_ns; w_es; w_ds];
    
    w_n = w_NED(1);
    w_e = w_NED(2);
    w_d = w_NED(3);
    
    % compute air data
    V_bw = R2*[w_ns; w_es; w_ds] + [u_wg; v_wg; w_wg];
    
    u_w = V_bw(1);
    v_w = V_bw(2);
    w_w = V_bw(3);
    
    V_ba = [u - u_w; v - v_w; w - w_w];
    
    u_r = V_ba(1);
    v_r = V_ba(2);
    w_r = V_ba(3);
    
    Va = sqrt(u_r^2+v_r^2+w_r^2);
    alpha = atan((w_r)/(u_r));
    beta = asin(v_r/Va);
    
    % compute external forces and torques on aircraft
    f = [-P.mass*P.gravity*st; P.mass*P.gravity*ct*sp; P.mass*P.gravity*ct*cp] +...
        1/2*P.rho*Va^2*P.S_wing*...
        [CX(alpha) + CXq(alpha)*P.c/(2*Va)*q + CXde(alpha)*delta_e;P.C_Y_0 + P.C_Y_beta*beta + P.C_Y_p*P.b/(2*Va)*p + P.C_Y_r*P.b/(2*Va)*r+...
        P.C_Y_delta_a*delta_a + P.C_Y_delta_r*delta_r;CZ(alpha) + CZq(alpha)*P.c/(2*Va)*q + CZde(alpha)*delta_e] +...
        1/2*P.rho*P.S_prop*P.C_prop*[(P.k_motor*delta_t)^2 - Va^2; 0; 0];
    
    Force(1) =  f(1);
    Force(2) =  f(2);
    Force(3) =  f(3);
    
    t = 1/2*P.rho*Va^2*P.S_wing*...
        [P.b*(P.C_ell_0 + P.C_ell_beta*beta + P.C_ell_p*P.b/(2*Va)*p +...
        P.C_ell_r*P.b/(2*Va)*r + P.C_ell_delta_a*delta_a + P.C_ell_delta_r*delta_r);P.c*(P.C_m_0 + P.C_m_alpha*alpha + P.C_m_q*P.c/(2*Va)*q + P.C_m_delta_e*delta_e);P.b*(P.C_n_0 + P.C_n_beta*beta + P.C_n_p*P.b/(2*Va)*p + P.C_n_r*P.b/(2*Va)*r +...
        P.C_n_delta_a*delta_a + P.C_n_delta_r*delta_r)] +...
        [-P.k_T_P*(P.k_Omega*delta_t)^2; 0; 0];
    
    Torque(1) = t(1);
    Torque(2) = t(2);
    Torque(3) = t(3);
    
    function y = CD(alpha)
        y = P.C_D_p + (P.C_L_0 + P.C_L_alpha*alpha)^2/(pi*P.e*P.AR);
    end

    function y = CL(alpha)
        y = (1 - sigma(alpha))*(P.C_L_0 + P.C_L_alpha*alpha) +...
        sigma(alpha)*(2*sign(alpha)*sin(alpha)^2*cos(alpha));
    end
    
    function y = sigma(alpha)
        y = (1 + exp(-P.M*(alpha-P.alpha0)) + exp(P.M*(alpha+P.alpha0)))/...
        ((1 + exp(-P.M*(alpha-P.alpha0)))*(1 + exp(P.M*(alpha+P.alpha0))));
    end

    function y = CX(alpha)
        y = -CD(alpha)*cos(alpha) + CL(alpha)*sin(alpha);
    end

    function y = CXq(alpha)
        y = -P.C_D_q*cos(alpha) + P.C_L_q*sin(alpha);
    end

    function y = CXde(alpha)
        y = -P.C_D_delta_e*cos(alpha) + P.C_L_delta_e*sin(alpha);
    end

    function y = CZ(alpha)
        y = -CD(alpha)*sin(alpha) - CL(alpha)*cos(alpha);
    end

    function y = CZq(alpha)
        y = -P.C_D_q*sin(alpha) - P.C_L_q*cos(alpha);
    end

    function y = CZde(alpha)
        y = -P.C_D_delta_e*sin(alpha) - P.C_L_delta_e*cos(alpha);
    end
   
    out = [Force'; Torque'; Va; alpha; beta; w_n; w_e; w_d];
end
