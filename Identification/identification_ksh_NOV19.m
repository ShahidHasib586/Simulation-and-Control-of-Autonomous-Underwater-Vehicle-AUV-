%% Identification steps:
% 1-load data
% 2-find physical equation with variables to be identified
% 3- identify the variable using least squares method or another methods
% 4- update the identified values in the jason file to be used in the simulation

%% identification of drag coefficient ksh 
clear all; close all

addpath(genpath('.'));

data=load('./Data/SpeedSteps_Noise_U.mat');

u=[data.vehicle_state.nu.rAUV_WaterSpeed.Uwater_ms.Data(:)];
Fx=[data.forces_values.HydrodynamicForces.Fx_N.Data(:)];

rho=1026;
nu=1.2e-06;
Sref=0.385;
Lref=5;

Re=abs(u)*Lref/nu;
CXF= 0.075./((log10(Re)-2).^2);

b=Fx;
A=0.5*rho*Sref*CXF.*u.*abs(u);
%identification step using least squares
ksh=lsqr(A,b); %edit ksh value in conf/AUVParameters.json

%% plots
CX0=ksh*CXF;
Fx_identified=0.5*rho*Sref*CX0.*u.*abs(u);

figure('name','ident_ksh'); hold on; grid on;
plot(Fx);
plot(Fx_identified);
legend('fx data', 'fx identified');
ylabel('force (N)'); title('Identification of ksh');