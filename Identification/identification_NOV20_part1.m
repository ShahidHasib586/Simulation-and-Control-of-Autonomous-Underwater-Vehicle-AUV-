clear all; close all
addpath(genpath('.'));

%% loading data
data=load('./Data/PitchSteps_NoiseOn_Pitch_W_Q.mat');
u=[data.vehicle_state.nu.rAUV_WaterSpeed.Uwater_ms.Data(:)];
w=[data.vehicle_state.nu.rAUV_WaterSpeed.Wwater_ms.Data(:)];
q=[data.vehicle_state.nu.AngularSpeed.Q_rads.Data(:)];
Fz_hydro=[data.forces_values.HydrodynamicForces.Fz_N.Data(:)];

%% constant values
rho=1026;
nu=1.2e-06;
Sref=0.385;
Lref=5;
CZ0=-0.01;

%% parameters identification: CZuw & CZuq
%sum(Ai*Xi)=b -> least square equation
b=Fz_hydro/(0.5*rho*Sref)-CZ0.*u.*abs(u);
A1=u.* w;
A2=Lref.*u.*q;
%identification step using least squares
A=[A1,A2];
X=lsqr(A,b);
CZuw=X(1);
CZuq=X(2);

%% plots
Fz_identified=0.5*rho*Sref*(CZ0.*u.*abs(u)+CZuw.*A1+CZuq.*A2);
   
figure('name','identfied_Fz_hydrodynamics'); hold on; grid on;
plot(Fz_hydro);
plot(Fz_identified);
legend('fz data', 'fz identified');
ylabel('force (N)'); title('Identification of Fz');