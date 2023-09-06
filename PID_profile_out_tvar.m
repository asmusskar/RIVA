function [o,Zp,Zs_dotdot] = PID_profile_out_tvar(X,sys)

% Identify variable parameters
Kp = X(1);
Ki = X(2);
Kd = X(3);

% Constant input parameters
Zsm_dotdot = sys.accm;
Zs_dotdot  = sys.acc0;
Zp         = sys.Zp0;
%dt         = sys.dt;
Zu         = sys.Zu0;
Zs         = sys.Zs0;
K1         = sys.K1;
K2         = sys.K2;
C          = sys.C;
U          = sys.U;
time       = sys.time;        % varying time vector

% Initialize error terms
ep = sys.acc0; ei = sys.acc0; ed = sys.acc0;

n   = length(Zsm_dotdot);
for i = 2:n-1
    % estimate the time increment/speed for the curret step
    dt  = ((time(i)-time(i-1)) + (time(i+1)-time(i)))/2;
    if dt<0
        error('negative time increment')
    end
    % calculate unsprung mass displacement
    Zu(i+1) = ((dt*C+2)...
        *(dt^2*K1*(Zp(i)-Zu(i))-U*(Zu(i-1)-2*Zu(i))+2*Zs(i)-Zs(i-1))...
        + 2*dt^2*K2*(Zs(i)-Zu(i))+dt*C*(Zu(i-1)-Zs(i-1))+2*Zs(i-1)-4*Zs(i))...
        /(dt*C*(1+U)+2*U);
    
    % calculate sprung mass displacement
    Zs(i+1) = dt^2*K1*(Zp(i)-Zu(i))-U*(Zu(i+1)-2*Zu(i)+Zu(i-1))+2*Zs(i)-Zs(i-1);
    
    % calculate sprung mass acceleration
    Zs_dotdot(i) = (Zs(i+1)-2*Zs(i)+Zs(i-1))/dt^2;
    
    % error terms
    ep(i) = Zsm_dotdot(i)-Zs_dotdot(i);
    ei(i) = 0.5*dt*(ep(i)+ep(i-1)) + ei(i-1);
    ed(i) = (Zsm_dotdot(i+1)-Zsm_dotdot(i-1))/(2*dt)-(Zs_dotdot(i)-Zs_dotdot(i-1))/dt;
    
    % calculate profile
    Zp(i+1) = Kp*ep(i)+Ki*ei(i)+Kd*ed(i);
    
    % update error term for summation 
    ei(i-1) = ei(i); 
end

% Calculate residual
residual = Zsm_dotdot-Zs_dotdot;

% Select solver type
if strcmp(sys.solver,'nms') > 0       % Nelder-Mead simplex
    o = sum(abs(residual));
elseif strcmp(sys.solver,'grad') > 0  % Unconstrained gradient-descent
    o = sum(abs(residual));
elseif strcmp(sys.solver,'lst') > 0   % Non-linear least squares method
    o = residual;
end