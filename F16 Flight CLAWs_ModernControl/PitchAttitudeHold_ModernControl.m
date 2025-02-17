%Aircraft model data obtained from the reference [1]: 'Brian L. Stevens, Aircraft Control and Simulation'
%Script edited by Di Wang
%Created time: 13/Feb/2025
%Latest update: 14/Feb/2025 by Di Wang

%Pitch Attitude Hold Autopilot with Modern Control Theory
%Optimal LQR method is applied on controller and Kalman filter observer

%F-16 Longitudinal State Space Model with specific trim condition
%(VT=250ft/s, h=50ft, gamma and xcg, refer to the mentioned book P326)
A = [-3.8916e-2 1.8992e1 -3.2139e1 0;
     -1.0285e-3 -6.4537e-1 5.6129e-3 1;
     0 0 0 1;
     8.0847e-5 -7.7287e-1 -8.0979e-4 -5.29e-1]; %Four states (VT, AoA, Theta, q)
B = [0; 0; 0; -0.010992]; %one input elevator angle
C = [0 0 1 0]; %one output (Theta)
D = [0];
sysLong = ss(A,B,C,D);

%Check stability for original aircraft dynamic under cruise condition
%result: stable - all eigen values are at LHP
[T, D] = eig(A); %check eigen values position

%Check controllability of system with single input (elevator angle)
controllability = rank(ctrb(A,B)); %full rank indicates controllable

%Check observability of the pitch angle state (theta)
%full rank indicates full states can be estimated (observed) from the 'theta' state
observability = rank(obsv(A,C));

%% Control law design by applying LQR method
%optimal controller gain matrix design with LQR
%On going work: tuning the Q and R
Q = [1 0 0 0;
     0 10 0 0;
     0 0 200 0;
     0 0 0 50]; %states penalty matrix
R = 10; %input penalty scaler
K = lqr(A,B,Q,R); %Controller

%Kalman filter observer design with LQR
%insert Gaussian system input disturbance Vd and sensor noise Vn
Vd = 1 * eye(4);
Vn = 1;
Kf = (lqr(A',C',Vd,Vn))'; %Gain of Kalman filter observer
sysKf = ss(A - Kf*C, [B Kf], eye(4), 0*[B Kf]); %state space of kalman filter

%%Feedback control system modeling please refer to related Simulink file