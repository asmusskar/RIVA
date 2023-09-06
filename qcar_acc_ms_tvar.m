% Quarter-car model with varying time/speed

function ams = qcar_acc_ms_tvar(sys)
K1        = sys.K1;
K2        = sys.K2;
C         = sys.C;
U         = sys.U;
Zu        = sys.Zu0;
Zs        = sys.Zs0;
Zs_dotdot = sys.acc0;
time      = sys.time;
Zp        = sys.Zp; %P79 data

for i = 2:length(Zu)-1
    % estimate the time increment/speed for the curret step
    dt  = ((time(i)-time(i-1)) + (time(i+1)-time(i)))/2;
    %dt = sys.dt;
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
end

% % combine the two quarter-car models
% if ~isfield(sys,'shift')
%     disp('combine the two quarter-car models')
%     shift = max(find(sys.x < sys.wb))+1;
%     Zs_dotdot(shift:end) = Zs_dotdot(shift:end) + Zs_dotdot(1:end-(shift-1));
% end

ams = Zs_dotdot;