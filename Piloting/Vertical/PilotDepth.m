function out = PilotDepth(in, memory, parameters)
% Depth piloting
coder.extrinsic('lqrd');
%% Inputs
u0 = 2.5 ;
%u0 = in.u_ms;
%dt = in.delta_time_s;
q = in.q_rads;
w = in.w_ms;
%% Computations

rho = 1026;
Sref = 0.385;
Lref = 5;

CMuw = 0.913291283211931;
CMuq = -0.867289970655682;

CZuw = -3.158171262880661;
CZuq = -1.939351431208779;

CZ0 = -0.01;
CM0 = -0.02;

zg=3e-2;
zb=0;

CZ = -0.41;
CM = -0.17;

m = 1005.275;
gravity = 9.81;
V = 0.98;

W = m * gravity;
B = V*rho*gravity;
%%
%mass matrix inverse
m_11t = m + 68.6;
m_33t = m + 1845.9;
m_55t = 4173.5 + 3388.76;
m_35 = 0;
m_13 = 0; 
m_15 = 0;
Det_M = m_11t * m_33t * m_55t - m_11t * m_35^2 - m_13^2 * m_55t + 2 * m_13 * m_15 * m_35 - m_15^2 * m_33t;
Det_33 = (m_11t * m_55t - m_15^2) / Det_M;
Det_55 = (m_11t * m_33t - m_13^2) / Det_M;

%%
%equilibrium point at theta0 and BAR0
theta_0 = (-(Det_33 * 0.5 * rho * Sref * CZ * u0 * abs(u0)) * (Det_55 * 0.5 * rho * Sref * Lref * CM0 * abs(u0) * u0) + ...
           (Det_33 * 0.5 * rho * Sref * CZ0 * abs(u0) * u0) * (Det_55 * 0.5 * rho * Sref * Lref * CM * u0 * abs(u0)) + ...
           (Det_33 * (W - B)) * (Det_55 * 0.5 * rho * Sref * Lref * CM * u0 * abs(u0))) / ...
          ((Det_33 * 0.5 * rho * Sref * CZ * u0 * abs(u0)) * (Det_55 * 0.5 * rho * Sref * CMuw * Lref * u0) * u0 + ...
           (Det_33 * 0.5 * rho * Sref * CZ * u0 * abs(u0)) * (-Det_55 * (zg * W - zb * B)) - ...
           (Det_33 * 0.5 * rho * Sref * CZuw * u0) * (Det_55 * 0.5 * rho * Sref * Lref * CM * u0 * abs(u0)) * u0);

% Calculate BAR_0 directly
BAR_0 = (-(Det_55 * 0.5 * rho * Sref * CMuw * Lref * u0) * u0 * theta_0 - ...
         (Det_55 * 0.5 * rho * Sref * Lref * CM0 * abs(u0) * u0) - ...
         (-Det_55 * (zg * W - zb * B)) * theta_0) / ...
        (Det_55 * 0.5 * rho * Sref * Lref * CM * u0 * abs(u0));

%% X=[ q w z theta int_z]' 
% A matrix
A_11 = Det_55*(0.5*rho*Sref* Lref^2 * CMuq*u0 - m*zg*w );
A_12 = Det_55*(-m*zg*q + 0.5*rho*Sref*CMuw*Lref*u0);
A_13 = 0;
A_14 = -Det_55*(zg*W - zb*B)*cos(theta_0);
A_15 = 0;

A_21 = Det_33*(2*m*zg*q + m*u0 + 0.5*rho*Sref*CZuq*Lref*u0);
A_22 = Det_33*0.5*rho*Sref*CZuw*u0;
A_23 = 0;
A_24 = -Det_33*(W - B)*sin(theta_0);
A_25 = 0;

A_31 = 0;
A_32 = cos(theta_0);
A_33 = 0;
A_34 = -u0*cos(theta_0) - w*sin(theta_0);
A_35 = 0;

A_41 = 1;
A_42 = 0;
A_43 = 0;
A_44 = 0;
A_45 = 0;

A_51 = 0;
A_52 = 0;
A_53 = 1;
A_54 = 0;
A_55 = 0;

A = [A_11 A_12 A_13 A_14 A_15;
     A_21 A_22 A_23 A_24 A_25;
     A_31 A_32 A_33 A_34 A_35;
     A_41 A_42 A_43 A_44 A_45;
     A_51 A_52 A_53 A_54 A_55];

%B matrix
B_11 = Det_55*0.5*rho*Sref*Lref*CM*u0*abs(u0);
B_21 = Det_33*0.5*rho*Sref*CZ*u0*abs(u0);
B_31 = 0;
B_41 = 0;
B_51 = 0;

B = [B_11;
     B_21;
     B_31;
     B_41;
     B_51];

% Calculating K matrix

Q = diag([1000, 100, 1000, 1000, 100]);
R = 1e6;
C = eye(5);
Q = C'*Q*C;

T = in.delta_time_s;
K_lqr = zeros(1, 5);
K_lqr = lqrd(A,B,Q,R,T);

%% Compute deltas
delta_q = - in.q_rads;

w_e = in.u_ms * tan(theta_0);
delta_w = w_e- in.w_ms;

delta_z = in.zc_m - in.z_m;
delta_z = EcaF_Saturate(delta_z,-parameters.delta_z_sat_m,parameters.delta_z_sat_m);

delta_theta = DiffAngle(theta_0, in.theta_rad);

BARc_rad = BAR_0 + delta_q .* K_lqr(1) + delta_w .* K_lqr(2) + delta_z .* K_lqr(3) + delta_theta .* K_lqr(4) + memory.int_z .* K_lqr(5);
%% Output saturation
out.BARc = EcaF_Saturate(BARc_rad,parameters.BAR_sat_rad(1),parameters.BAR_sat_rad(2));

end
