%Jeffrey Paine
%ME621 Project

function [sys,x0,str,ts,simStateCompliance] = mav_dynamics(t,x,u,flag,P)

switch flag

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(P);

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1
    sys=mdlDerivatives(t,x,u,P);

  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
  case 2
    sys=mdlUpdate(t,x,u);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3
    sys=mdlOutputs(t,x,u);

  %%%%%%%%%%%%%%%%%%%%%%%
  % GetTimeOfNextVarHit %
  %%%%%%%%%%%%%%%%%%%%%%%
  case 4
    sys=mdlGetTimeOfNextVarHit(t,x,u);

  %%%%%%%%%%%%%
  % Terminate %
  %%%%%%%%%%%%%
  case 9
    sys=mdlTerminate(t,x,u);

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end
end
% end sfuntmpl

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(P)

%
% call simsizes for a sizes structure, fill it in and convert it to a
% sizes array.
%
% Note that in this example, the values are hard coded.  This is not a
% recommended practice as the characteristics of the block are typically
% defined by the S-function parameters.
%
sizes = simsizes;

sizes.NumContStates  = 12;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 12;
sizes.NumInputs      = 6;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

%
% initialize the initial conditions
%
x0  = [...
    P.pn0;...
    P.pe0;...
    P.pd0;...
    P.u0;...
    P.v0;...
    P.w0;...
    P.phi0;...
    P.theta0;...
    P.psi0;...
    P.p0;...
    P.q0;...
    P.r0;...
    ];

%
% str is always an empty matrix
%
str = [];

%
% initialize the array of sample times
%
ts  = [0 0];

% Specify the block simStateCompliance. The allowed values are:
%    'UnknownSimState', < The default setting; warn and assume DefaultSimState
%    'DefaultSimState', < Same sim state as a built-in block
%    'HasNoSimState',   < No sim state
%    'DisallowSimState' < Error out when saving or restoring the model sim state
simStateCompliance = 'UnknownSimState';

% end mdlInitializeSizes
end
%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
function x_dot = mdlDerivatives(t,x,uu, P)
    x_dot = zeros(12,1);
    
    pn    = x(1);
    pe    = x(2);
    pe    = x(3);
    u     = x(4);
    v     = x(5);
    w     = x(6);
    phi   = x(7);
    theta = x(8);
    psi   = x(9);
    p     = x(10);
    q     = x(11);
    r     = x(12);
    fx    = uu(1);
    fy    = uu(2);
    fz    = uu(3);
    ell   = uu(4);
    m     = uu(5);
    n     = uu(6);
    
    ct = cos(theta);cu = cos(psi);cp = cos(phi);
    st = sin(theta);su = sin(psi);sp = sin(phi);
    tt = tan(theta);
    
    % p_n_dot, p_e_dot, p_d_dot
    R1 = [ct*cu , sp*st*cu - cp*su , cp*st*cu - sp*su;
          ct*su , sp*st*su + cp*cu , cp*st*su - sp*cu;
          -st   , sp*ct            , cp*ct];
    x_dot(1:3) = R1 * x(4:6);
    
    % u_dot, v_dot, w_dot
    x_dot(4:6) = [r*v - q*w;p*w - r*u;q*u - p*v] + ([fx;fy;fz] / P.mass);
    
    % phi_dot, theta_dot, upsilon_dot
    R2 = [1 , sp*tt , cp*tt;
          0 , cp    , -sp;
          0 , sp/ct , cp/ct];
    x_dot(7:9) = R2*x(10:12);
    
    % p_dot, q_dot, r_dot
    g  = P.Jx*P.Jz - P.Jxz^2;
    g1 = g \ (P.Jxz*(P.Jx-P.Jy+P.Jz));
    g2 = g\(P.Jz*(P.Jz-P.Jy)+P.Jxz^2);
    g3 = P.Jz/g;
    g4 = P.Jxz/g;
    g5 = (P.Jz - P.Jx)/P.Jy;
    g6 = P.Jxz/P.Jy;
    g7 = g\((P.Jx-P.Jy)*P.Jx+P.Jxz^2);
    g8 = P.Jx/g;
    
    x_dot(10) = g1*p*q - g2*q*r + g3*ell + g4*n;
    x_dot(11) = g5*p*r - g6*(p^2 - r^2) + m/P.Jy;
    x_dot(12) = g7*p*q - g1*q*r + g4*ell + g8*n;
  
end

% end mdlDerivatives

%
%=============================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.
%=============================================================================
%
function sys=mdlUpdate(t,x,u)

sys = [];

end
% end mdlUpdate

%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u)

sys = x;

end
% end mdlOutputs

%
%=============================================================================
% mdlGetTimeOfNextVarHit
% Return the time of the next hit for this block.  Note that the result is
% absolute time.  Note that this function is only used when you specify a
% variable discrete-time sample time [-2 0] in the sample time array in
% mdlInitializeSizes.
%=============================================================================
%
function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;

end
% end mdlGetTimeOfNextVarHit

%
%=============================================================================
% mdlTerminate
% Perform any end of simulation tasks.
%=============================================================================
%
function sys=mdlTerminate(t,x,u)

sys = [];

end
% end mdlTerminate
