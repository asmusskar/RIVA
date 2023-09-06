%% Authors
%   Asmus Skar
%   Copyright DTU Sustain, Technical University of Denmark.

% DESCRIPTION:
% This script calculates the inverted road profile from synthetic car body
% accelerations utilizing a PID conroller algorithm

% Clear workspace and set plotting flags
clear all, close all, clc

% -------------------------------------------------------------------------
% Initialize system 
% -------------------------------------------------------------------------

% Get measurements
load pid_control                            % Input accelerations
sys.dx = distance(2)-distance(1);           % Sampling interval [m]
sys.dt = time(2)-time(1);                   % Time step [s]
sys.v  = sys.dx/sys.dt;                     % Velocity in [m/s]

% Manipulate velocity for testing of code
x    = 1:1:length(time)-1;
vvar = 0;                                   % Variation in velocity
fv   = 0.5;                                 % Frequency 
sys.vvar = vvar.*sin(2*pi*fv.*time(1:end))...
    +sys.v;                                 % Varying velocity in [m/s]
sys.dtvar = sys.dx./sys.vvar;
sys.time  = cumsum(sys.dtvar);

sys.Zpfm  = inpfilt.*1e3;                   % Road profile (filtered)
sys.Zraw  = severity.*1e3;
sys.Zpfm  = sys.Zpfm-sys.Zpfm(1);

% Model parameters 
sys.K1 = 653; 
sys.K2 = 63.3; 
sys.C  = 6; 
sys.U  = 0.15; 

% Initialize response
sys.Zu0  = zeros(length(acceleration),1);            
sys.Zs0  = sys.Zu0; 
sys.Zp0  = sys.Zu0; 
sys.acc0 = sys.Zu0;

%--------------------------------------------------------------------------
% Generate synthetic accelerations
%--------------------------------------------------------------------------
sys.Zp   = sys.Zpfm;
sys.accm = qcar_acc_ms_tvar(sys);

%--------------------------------------------------------------------------
% Optimization options
%--------------------------------------------------------------------------
sys.solver = 'nms';                     % Solver type

% Variable input
Kp0 = 0; Ki0 = 0; Kd0 = 0;              % PID coefficients
x0  = [Kp0 Ki0 Kd0];

%--------------------------------------------------------------------------
% Run optimization algorithm
%--------------------------------------------------------------------------

options = optimset('MaxIter',1e6,'MaxFunEvals',1e6,'TolFun',1e-6,'TolX',1e-6);
[xmin,fval,exitflag,output] = fminsearch(@(x)PID_profile_inv_tvar(x,sys),x0,options);

%--------------------------------------------------------------------------
% Post-processing
%--------------------------------------------------------------------------
% Get optimal profile
[o,pinv,accinv] = PID_profile_out_tvar(xmin,sys);

%--------------------------------------------------------------------------
% Plots
%--------------------------------------------------------------------------

for i = 2:length(sys.Zpfm)
    if sys.Zpfm(i-1) == 0 && sys.Zpfm(i) == 0
        sys.Zpfm(i) = NaN;
    elseif isnan(sys.Zpfm(i-1)) && sys.Zpfm(i)== 0
        sys.Zpfm(i) = NaN;
    end
end

Zp = [distance sys.Zraw sys.Zpfm pinv];
nan_rows = any(isnan(Zp(:,3)),2);
Zp(nan_rows,:) = [];

hf1=figure(1);
title('Synthetic profile');
hold on
plot(Zp(:,1),Zp(:,2),'-k','LineWidth',0.5)
hold on, grid on
plot(Zp(:,1),Zp(:,3),':b','LineWidth',1.5)
hold on, grid on
legend('Raw','Filtered','Location','NorthWest')
ylabel('Elevation [mm]')
xlabel('Distance [m]')
% %yticks([-50,-25,0,25,50])
ylim([-100 100])
xlim([0 50])
axes('parent',hf1,'position',[0.65 0.665 0.25 0.25]); 
box on
plot(Zp(:,1),Zp(:,2),'-k','LineWidth',0.5);
hold on, grid on
plot(Zp(:,1),Zp(:,3),':b','LineWidth',1.5);
hold on % inset plot
xlim([29.75 31.5])
set(gca,'YTickLabel',[]);
set(gca,'XTickLabel',[]);
hold off

pcas = corrcoef(Zp(:,4),Zp(:,3));
PCA  = pcas(1,2);

figure
subplot(2,1,1), title('Profile inversion');
hold on
plot(Zp(:,1),Zp(:,3),':r','LineWidth',1.5)
hold on, grid on
plot(Zp(:,1),Zp(:,4),'--b','LineWidth',1.2,'MarkerSize',3.0)
hold on
legend('True','Calculated')
ylabel('Elevation [mm]')
%xlabel('Distance [m]')
%yticks([-50,-25,0,25,50])
%ylim([-50 50])
hold on

%figure
subplot(2,1,2), title('Acceleration');
hold on
plot(distance(1:10:end),sys.accm(1:10:end).*1e-3,':r','LineWidth',1.5)
hold on, grid on
plot(distance(1:10:end),accinv(1:10:end).*1e-3,'--b','LineWidth',1.2,'MarkerSize',3.0)
hold on
%legend('Measured','Inverted')
xlabel('Distance [m]')
ylabel('Acceleration [m/s^2]')
% yticks([-3,-2,-1,0,1,2,3])
% ylim([-3 3])
hold off

% Plot object function
ulim   = 1;
llim   = -1;
pts    = 501;
dist   = linspace(llim,ulim,pts);
Kpe    = xmin(1) + xmin(1).*dist;
Kie    = xmin(2) + xmin(2).*dist;
Kde    = xmin(3) + xmin(3).*dist;
op     = zeros(length(dist),1);
oi     = op;
od     = op;

for i = 1:length(Kpe)
    x00 = [Kpe(i) xmin(2) xmin(3)];
    op(i) = PID_profile_out_tvar(x00,sys);
end

for i = 1:length(Kie)
    x00 = [xmin(1) Kie(i) xmin(3)];
    oi(i) = PID_profile_out_tvar(x00,sys);
end

for i = 1:length(Kde)
    x00 = [xmin(1) xmin(2) Kde(i)];
    od(i) = PID_profile_out_tvar(x00,sys);
end

opm = [Kpe' op];
nan_rows_p = any(isnan(opm),2);
opm(nan_rows_p,:) = [];
oim = [Kie' oi];
nan_rows_i = any(isnan(oim),2);
oim(nan_rows_i,:) = [];
odm = [Kde' od];
nan_rows_d = any(isnan(odm),2);
odm(nan_rows_d,:) = [];

figure
title('Object function');
hold on
plot(opm(:,1)./xmin(1),opm(:,2),'--k','LineWidth',1.5)
hold on, grid on
plot(oim(:,1)./xmin(2),oim(:,2),'-.k','LineWidth',1.5)
hold on
plot(odm(:,1)./xmin(3),odm(:,2),':k','LineWidth',1.5)
hold on
legend('K_P^{FS}/K^0','K_I^{FS}/K^0','K_D^{FS}/K^0')
xlabel('K^{FS}/K^0')
ylabel('\psi')
%yticks([-50,-25,0,25,50])
ylim([o*0.9 1e6])
xlim([1+llim 1+ulim])
hold off
