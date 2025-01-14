clear all; close all;
addpath(genpath('.'));

%% loading data
data=load('./Data/DieuDonne_AStep_NoiseOn_U_V_R_A.mat');
u=[data.vehicle_state.nu.rAUV_WaterSpeed.Uwater_ms.Data(:)];
v=[data.vehicle_state.nu.rAUV_WaterSpeed.Vwater_ms.Data(:)];
r=[data.vehicle_state.nu.AngularSpeed.R_rads.Data(:)];
Mz_hydro =[data.forces_values.HydrodynamicForces.Mz_Nm.Data(:)];

%% constant values
rho=1026;
Sref=0.385;
Lref=5;

%% parameters identification: CMuv and CMur
% usage: x=FMINSEARCHCON(fun,x0,LB,UB)


% Starting values based on symmetry
CMuw = 0.91329;
CMuq = -0.86729;
X0 = [-CMuw, CMuq]; % Initial guesses for CYuv and CYur

% Bounds
lower_bound = [-CMuw - 0.1 * abs(CMuw), CMuq - 0.1 * abs(CMuq)];   % Lower bounds
upper_bound = [-CMuw + 0.1 * abs(CMuw), CMuq + 0.1 * abs(CMuq)];   % Upper bounds

% Cost function for Mz ()
fun = @(M_Y) norm(Mz_hydro - 0.5 * rho * Sref * Lref * (M_Y(1) .* u .* v + Lref * M_Y(2) .* u .* r));

% Optimization using fmincon
[C_Y, ~] =fminsearchcon(fun, X0, lower_bound, upper_bound);
CNuv=C_Y(1);
CNur=C_Y(2);

%% plots
Mz_identified=0.5*rho*Sref*Lref*(CNuv.*u.*v+ Lref * CNur.*u.*r);

figure('name','identfied_Mz_hydrodynamics'); hold on; grid on;
plot(Mz_hydro);
plot(Mz_identified);
legend('Mz data', 'Mz identified');
ylabel('Moment (Nm)'); title('Identification of Mz');