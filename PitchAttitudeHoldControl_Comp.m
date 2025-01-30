%Reference [1]: 'Brian L. Stevens, Aircraft Control and Simulation'
%Script edited by Di Wang
%Created time: 23/Jan/2025
%Latest Updated: 24/Jan/2025

%Pitch Attitude Hold Autopilot with Dynamic Compensation
%F-16 Longitudinal State Space Model with specific trim condition
%(VT=250ft/s, h=50ft, gamma and xcg, refer to the mentioned book P326)
A = [-3.8916e-2 1.8992e1 -3.2139e1 0;
     -1.0285e-3 -6.4537e-1 5.6129e-3 1;
     0 0 0 1;
     8.0847e-5 -7.7287e-1 -8.0979e-4 -5.29e-1]; %Four states (VT, AoA, Theta, q)
B = [0; 0; 0; -0.010992]; %one input elevator angle
C = [0 0 57.296 0;
     0 0 0 57.296]; %two outputs (Theta, q)
D = [0;0];
states = {'VT' 'AoA' 'Theta' 'q'};
inputs = {'elevator'};
outputs = {'Theta' 'q'};
sysLong = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs); %obtain state space of the aircraft Longitudinal dynamic

actuator = tf([0 1],[0.05 1]); %TF of elevator angle actuator
gyro_q = tf([0 1],[0.05 1]); %TF of pitch rate sensor
gyro_theta = tf([0 1],[0.05 1]); %TF of pitch angle sensor
Comp = tf([1 1.6 0.28],[1 14 0]);
Kq = -1.09; %Pitch rate gain
Kp = -40; %pitch angle gain
P_I_D = pid(-11.24721, -4.89388, -6.09047); %PID controller (KP,KI,KD)

inner_cl_q = feedback(actuator*sysLong, Kq*gyro_q, 1, 2); %pitch rate inner closed loop
%choose one of the below as the outer closed loop
outer_cl_PID = feedback(P_I_D*inner_cl_q, gyro_theta, 1, 1); %outer c.l with PID controller
%outer_cl_comp = feedback(Comp*Kp*inner_cl_q, gyro_theta, 1, 1); %outer c.l with pitch angle gain and compensator

figure(1)
step(outer_cl_PID)
%step(outer_cl_comp)