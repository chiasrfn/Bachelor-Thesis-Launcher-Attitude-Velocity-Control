clc;
clear;

% --- Output file naming ---
filetxt = 'velocity_control_4';

% ------- VARIANT BLOCKS / SETTINGS -------
% Save simulation results (1 to save, 0 to skip)
stampa = 0; 
% Control type selection: 1 = PID, 6 = Frequency-based, 0 = Tuning mode
tipo_controllo = 6; 
% Roll control selection: 0 = Tuning, 1 = Custom, -1 = Baseline (SBA)
controllo_roll = 0; 
% Velocity control: 0 = Disabled, 1 = Enabled (P-controller)
controllo_velcit = 1; 

% Realistic simulation flags
tutto_realistico = 1; 
% Variable mass (propellant consumption)
massa_variabile = true; 
% Variable atmospheric environment
ambiente_variabile = true; 
% Variable engine thrust
spinta_variabile = true; 
% External disturbances (wind/gusts)
dist_agg = true; 


if(ambiente_variabile)
    % Enable variable environment model
    env_variabile = true; 
    % Aerodynamic coefficients vary with Mach number (M)
    aero_variabile_con_M = true; 
else
    env_variabile = false; 
    aero_variabile_con_M = false; 
end

% -------- INITIAL CONDITIONS --------
% Reference velocity [cite: 57]
V_r = 400; 
% Initial velocity in Body Frame axes [u, v, w] [cite: 64, 171]
v_0 = [V_r; 0; 0]; 
% Initial angular velocity in Body Frame [p, q, r] [cite: 72, 171]
w_0 = [0; 0; 0]; 
% Initial position relative to Earth (Inertial) [altitude, North, East] [cite: 86]
x_0 = [0; 0; 0]; 
% Initial attitude [roll (chi), pitch (theta), yaw (psi)] [cite: 70, 71, 171]
Attitude_0 = [0; 0.085; 0.085]; 

% -------- DISTURBANCES ---------
% Wind velocity in Inertial Frame [Vertical, North, East] [cite: 80, 171]
Env.WindVelocity = [0, 30, 30]; 
V_w = 0; 
Wind_direction = 0; 
% Gust amplitude settings
Gust_amplitude = [0 0 0]; 

% Reference Attitude and Velocity
Attitude_riferimento = [0; 0; 0]; 
VelocitB_riferimento = [V_r, 0, 0]; 

% -------- LAUNCH VEHICLE PARAMETERS --------
% -------- Mass and Inertia -------
% Propellant mass (approx. 90% of total)
m_p = 510.9453e3; % kg 
% Dry mass (structural, approx. 10% of total) [cite: 266]
m_s = 56.7717e3; % kg 
% Total mass [cite: 50, 266]
m_tot = m_p + m_s; 
% Engine Specific Impulse
I_sp = 237; % s 

% Total moment of inertia - X axis (Roll) [cite: 49, 266]
J_tot = 1.252179e5; % kg*m^2 
% Total moment of inertia - Y and Z axes (Pitch/Yaw) [cite: 48, 266]
I_tot = 2.963818e8; % kg*m^2 
% Total inertia matrix
inertia_tot = [ J_tot,  0,      0;
                0,      I_tot*90/100,  0; 
                0,      0,      I_tot*110/100 ]; 

% Estimated Dry and Propellant inertia distribution
inertia_s = inertia_tot / 6; 
inertia_p = inertia_tot * 5 / 6; 

% ---------- Geometry and Dimensions ---------
% All distances are measured relative to the nose tip [cite: 59]
% Nozzle X-coordinate
x_nozzle = -90; % m 
% Center of Gravity (CG) X-coordinate [cite: 59]
x_cm = -53.19; % m 
% Center of Pressure (CP) X-coordinate [cite: 60]
x_cp = -84; % m 
% Distance between CM and Nozzle [cite: 61]
r_nozzle_cm = [x_nozzle - x_cm; 0; 0]; 
% Distance between CM and CP [cite: 60]
r_cp_cm = [x_cp - x_cm; 0; 0]; 
% Vehicle radius and diameter
R = 1.855; % m 
b = 2 * R; 
% Reference Surface Area (Cross-section) [cite: 54]
S_rif = pi * R^2; 
% Reference length (rocket length)
l_rif = b; 
% Rocket length excluding motors
L = 90; % m 

% ---------- Aerodynamic Coefficients --------
% Normal force slope (C_N_alpha) and reference values [cite: 46, 51]
C_N_alpha_cost = 0.1465; 
C_N_alpha_Mach = 0.0267; 
% Axial force coefficient (Drag)
C_A_cost = 2; 
% Rolling moment coefficient
C_l = 0.001; 
% Pitching moment slope (C_m_alpha)
C_m_alpha_cost = 2.8; 
C_m_alpha_Mach = 0.6667; 

% Final coefficient assignment
C_m_alpha = C_m_alpha_cost; 
C_N_alpha = C_N_alpha_cost; 
C_A = C_A_cost; 

% -------- ENVIRONMENT CONSTANTS --------
Env.Temperature = 15; 
Env.SpeedOfSound = 340.2; 
Env.AirPressure = 79495; 
Env.AirDensity = 0.4597; 
Env.Gravity = 7.9552; % [cite: 47]

% Reference Dynamic Pressure (q) [cite: 52]
q_r = (1/2) * Env.AirDensity * V_r^2; 
V_w = 30; 
Wind_direction = 45; 
Gust_amplitude = [3.5 3.5 3.0]; 

% -------- THRUST AND VELOCITY CONTROL --------
% Control Thrust (Ares I engine capability) [cite: 55, 266]
T_c = 1.052247e7; % N 
% Reference Thrust required to maintain constant cruise velocity [cite: 56]
T_r = m_tot * Env.Gravity + q_r * S_rif * C_A; 

% Apply thrust based on variant selection
if (spinta_variabile)
    T = T_c; 
else
    T = T_r; 
end

% ------- LINEARIZED SYSTEM PARAMETERS --------
% Dimensional derivatives [cite: 221, 232, 237]
N_alpha = q_r * S_rif * C_N_alpha; 
M_alpha = q_r * S_rif * b * C_m_alpha; 
x_CP = x_cp - x_cm; 
x_N = x_nozzle - x_cm; 
g = Env.Gravity; 

% ------- PITCH DYNAMICS ---------
% State vector x = [w (vertical velocity), q (pitch rate), theta (pitch angle)] [cite: 237]
A_p = [- N_alpha/(V_r*m_tot),                       +V_r,                       -g;
        (x_CP *N_alpha + M_alpha)/(I_tot * V_r),    0,                          0;
        0,                                          1,                          0;
      ];
% Control input u = eta (pitch nozzle deflection) [cite: 79]
B_p = [-T/m_tot;  (x_N * T)/I_tot;    0];
% Disturbance z = v_wE (wind)
P_p = [N_alpha/(V_r*m_tot); -(x_CP * N_alpha + M_alpha)/(V_r * I_tot); 0];
B_pz = cat(2, B_p, P_p); 
C_p = [1, 0, 0]; 
D_pz = [0,0]; 

% ------- YAW DYNAMICS ---------
% State vector x = [v (lateral velocity), r (yaw rate), psi (yaw angle)] [cite: 232]
A_y = [N_alpha/(V_r*m_tot),                       -V_r,                       g;
        -(x_CP * N_alpha + M_alpha)/(I_tot * V_r),  0,                           0;
        0,                                          1,                           0;
      ];
% Control input u = zeta (yaw nozzle deflection) [cite: 78]
B_y = [T/m_tot;  (x_N * T)/I_tot;    0];
% Disturbance z = v_wN
P_y = [N_alpha/(V_r*m_tot); (x_CP * N_alpha + M_alpha)/(V_r * I_tot); 0];
B_yz = cat(2, B_y, P_y); 
C_y = [1, 0, 0]; 
D_yz = [0,0]; 

% ------- ROLL DYNAMICS ---------
% State vector x = [p (roll rate), chi (roll angle)] [cite: 225]
A_r = [0 0; 1 0]; 
B_r = [1/J_tot; 0]; 
C_r = [0 1]; 

% --- State Space and Transfer Function objects ---
sys_p = ss(A_p, B_p, C_p, 0); 
sys_y = ss(A_y, B_y, C_y, 0); 
sys_r = ss(A_r, B_r, C_r, 0); 
zpk_r = zpk(sys_r); 

s = tf('s'); 

% -------- CONTROLLER DESIGN --------

% --- Roll Control ---
% Baseline Roll controller [cite: 311]
G_roll_sba = zpk([-0.5, -0.1], [0, -10], 100); 
% Optimized Roll PID with derivative filter [cite: 380]
G_roll = 2.6e5 + 4.4e4/s + 3.8e5 * 15 / (1 + 15/s); 

% --- Attitude PID Control (Pitch/Yaw) ---
% Step 3: Final tuned PID gains [cite: 575]
PID_p = -(10 + 5*10/(1+10/s) + 0.1/s); 
PID_y = -(10 + 5*10/(1+10/s) + 0.1/s); 

% --- Frequency-based Control (Angles only) ---
% Step 3: Final frequency-shaped controller [cite: 756]
G_6_p = zpk([-0.8 -0.25], [-8 0], -20); 
G_6_y = zpk([-0.8 -0.25], [-8 0], -20); 

% --- Velocity Control (Inner/Outer Loop) ---
% Angle-to-velocity dynamics conversion
conv_den_y = tf([1 0],[1 -N_alpha/(m_tot*V_r) -(x_cp*N_alpha)/(I_tot)]); 
conv_cost_y = -g; 

% Final tuned Velocity PID [cite: 1040, 1090]
G_7_y = (0.1 + 0.1*100/(1+100/s) + 0.1/s); 
G_7_p = (0.1 + 0.1*100/(1+100/s) + 0.1/s); 

% --- Rotation Matrix (Earth to Body) ---
chi = Attitude_0(1); theta = Attitude_0(2); psi = Attitude_0(3); 
R_BE = [cos(psi)*cos(theta), -sin(psi)*cos(chi)+cos(psi)*sin(theta)*sin(chi), sin(psi)*sin(chi)+cos(psi)*sin(theta)*cos(chi);
        sin(psi)*cos(theta), cos(psi)*cos(chi)+sin(psi)*sin(theta)*sin(chi),  -cos(psi)*sin(chi)+sin(psi)*sin(theta)*cos(chi);
        -sin(theta),  cos(theta)*sin(chi), cos(theta)*cos(chi)]; 

% --- Advanced Control (Full State Feedback) ---
% Pole placement using Ackermann's formula
K_p = acker(A_p, B_p, [-1, -1, -1]); 
K_y = acker(A_y, B_y, [-1, -1, -1]); 

% State Observers (Luenberger)
G_xp = (place(A_p', eye(3), [-2,-2,-2]))'; 
G_xy = (place(A_y', eye(3), [-2,-2,-2]))'; 

% Disturbance Observers
G_wE = (place(0, P_p', -5))'; 
G_wN = (place(0, P_y', -5))'; 

% --- RUN SIMULATION ---
out = sim("sim_finale_1.slx"); 

% --- SAVE DATA ---
if stampa == 1
    FileName = ['Output_', filetxt, '.mat']; 
    save(fullfile('./simulation_data/', FileName), 'out', 'massa_variabile','ambiente_variabile', 'spinta_variabile'); 
end

% --- FILENAME CONVENTION LEGEND ---
% 1st digit: Controller type
% 2nd digit: Condition type (Realistic vs Ideal)
% 3rd digit: Yaw angle variation magnitude
% 4th digit: Pitch angle variation magnitude
% NOTE: Higher digits indicate progressively more complex flight conditions.
