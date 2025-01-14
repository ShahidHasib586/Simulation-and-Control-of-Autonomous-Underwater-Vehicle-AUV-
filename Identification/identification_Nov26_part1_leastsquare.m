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

%% parameters identification: CZuw & CZuq
%sum(Ai*Xi)=b -> least square equation
b=Fy_hydro/(0.5*rho*Sref);
A1=u.*v;
A2=u.*r;
%identification step using least squares
A=[A1,A2];
X=lsqr(A,b);
CYuv=X(1);
CYur=X(2);

%% plots
Fy_identified=0.5*rho*Sref*(CYuv.*A1+ CYur.*A2);
   
figure('name','identfied_Fz_hydrodynamics'); hold on; grid on;
plot(Fy_hydro);
plot(Fy_identified);
legend('fy data', 'fy identified');
ylabel('force (N)'); title('Identification of Fy');