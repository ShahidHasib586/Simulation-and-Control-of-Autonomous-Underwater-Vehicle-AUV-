clear all; close all
addpath(genpath('.'));

%% loading data
data=load('./Data/PitchSteps_NoiseOn_Pitch_W_Q.mat');
u=[data.vehicle_state.nu.rAUV_WaterSpeed.Uwater_ms.Data(:)];
w=[data.vehicle_state.nu.rAUV_WaterSpeed.Wwater_ms.Data(:)];
q=[data.vehicle_state.nu.AngularSpeed.Q_rads.Data(:)];
My_hydro=[data.forces_values.HydrodynamicForces.My_Nm.Data(:)];

%% constant values
rho=1026;
nu=1.2e-06;
Sref=0.385;
Lref=5;
CM0=-0.02;

%% parameters identification: CMuw & CMuq
%sum(Ai*Xi)=b -> least square equation
b=My_hydro/(0.5*rho*Sref) - Lref*CM0.*u.*abs(u);
A1=Lref*u.*w;
A2=(Lref^2).*u.*q;
%identification step using least squares
A=[A1,A2];
X=lsqr(A,b);
CMuw=X(1);
CMuq=X(2);

%% plots
My_identified=0.5*rho*Sref*(Lref*CM0.*u.*abs(u)+CMuw.*A1+CMuq.*A2);

figure('name','identfied_My_hydrodynamics'); hold on; grid on;
plot(My_hydro);
plot(My_identified);
legend('My data', 'My identified');
ylabel('force (N)'); title('Identification of My');