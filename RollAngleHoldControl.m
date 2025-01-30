%Reference [1]: 'Brian L. Stevens, Aircraft Control and Simulation'
%Script edited by Di Wang
%Created time: 24/Jan/2025
%Latest updated: 25/Jan/2025

%Roll Angle Hold Autopilot
%F16 Lateral-Directional dynamic state space 
A = [-3.2201e-1 6.404e-2 3.6382e-2 -9.9167e-1;
     0 0 1 3.6928e-2;
     -30.649 0 -3.6784 6.6461e-1;
     8.5395 0 -2.5435e-2 -4.7637]; %Four states (Beta, Phi, p, r)
B = [2.9506e-4; 0; -7.3331e-1; -3.1865e-2]; % one input delta_a (aileron angle)
C = [0 57.296 0 0;
     0 0 57.296 0]; %Two outputs (Phi, p)
D = zeros(2,1);
states = {'Beta' 'Phi' 'p' 'r'}; %clarify the names of states
inputs = {'delta_a'}; %clarify the names of inputs
outputs = {'Phi' 'p'}; %clarify the names of outputs
sysLatDire = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs); %Obtain state space of the system

actuator = tf([0 -20.2],[1 20.2]); %aileron actuator
Kp = 0.05; %roll rate gain
gyro_p = tf([0 1],[0.05 1]); %TF of roll rate gyro
gyro_phi = tf([0 1],[0.05 1]); %TF of roll angle gyro
P_I = pid(5,1,0); %PI controller for roll angle

inner_p = feedback(Kp*actuator*sysLatDire, gyro_p, 1, 2); %roll rate p inner loop
outer_phi = feedback(P_I*inner_p, gyro_phi, 1, 1); %Pitch angle outer loop

figure(1)
step(outer_phi(1,:))
