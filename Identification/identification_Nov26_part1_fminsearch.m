clear all; close all;
addpath(genpath('.'));

%% loading data
data=load('./Data/DieuDonne_AStep_NoiseOn_U_V_R_A.mat');
u=[data.vehicle_state.nu.rAUV_WaterSpeed.Uwater_ms.Data(:)];
v=[data.vehicle_state.nu.rAUV_WaterSpeed.Vwater_ms.Data(:)];
r=[data.vehicle_state.nu.AngularSpeed.R_rads.Data(:)];
Fy_hydro=[data.forces_values.HydrodynamicForces.Fy_N.Data(:)];

%% constant values
rho=1026;
Sref=0.385;
Lref=5;

%% parameters identification: CYuv & CYur
% usage: x=FMINSEARCHCON(fun,x0,LB,UB)

% Starting values based on symmetry
CZuw=-3.1582;
CZuq=-1.93935143120878;
X0 = [CZuw, -CZuq]; % Initial guesses for CYuv and CYur

% Bounds
lower_bound = [CZuw - 0.2 * abs(CZuw), -CZuq - 0.025 * abs(CZuq)];  % Lower bounds
upper_bound = [CZuw + 0.2 * abs(CZuw), -CZuq + 0.025 * abs(CZuq)];  % Upper bounds

% Cost function to minimize (Fy_hydro equation)
fun = @(CY) norm((Fy_hydro - 0.5 * rho * Sref * (CY(1) .* v .* u + Lref * CY(2) .* r .* u)).^2);

% Optimization using fmincon
[C_Y, ~] =fminsearchcon(fun, X0, lower_bound, upper_bound);
CYuv=C_Y(1);
CYur=C_Y(2);

%% plots
Fy_identified=0.5*rho*Sref*(CYuv.*u.*v+ Lref * CYur.*u.*r);

figure('name','identfied_Fy_hydrodynamics'); hold on; grid on;
plot(Fy_hydro);
plot(Fy_identified);
legend('fy data', 'fy identified');
ylabel('force (N)'); title('Identification of Fy');