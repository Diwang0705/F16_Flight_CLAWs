%Reference [1]: 'Brian L. Stevens, Aircraft Control and Simulation'
%Reference [2]: 'Ben D. Flight Control Fundamental'
%Script edited by Di Wang
%Created time: 21/Jan/2025


%Altitude Hold Autopilot
%Longitudinal F16 State Space Maxtrix (4 states and 1 input)
A = [-1.9311e-2 8.8157 -3.217e1 -5.7499e-1;
     -2.5389e-4 -1.0189 0 9.0506e-1;
     0 0 0 1;
     2.9465e-12 8.2225e-1 0 -1.0744];
B = [1.737e-1;
     -2.1499e-3;
     0;
     -1.7555e-1];
C = eye(4); %states = outputs
D = zeros(4,1); %no relation between outputs and inputs
states = {'VT' 'AoA' 'theta' 'q'}; %clarify the names of states
inputs = {'delta_e'}; %clarify the names of inputs
outputs = {'VT' 'AoA' 'theta' 'q'}; %clarify the names of outputs
sysLong = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs); %Obtain state space of the system
J = tf(sysLong); %display TF for each input and output

actuator = tf([0 4], [0.05 1]); %TF of the elevator angle actuator
Kq = -25; %pitch rate gain
inner_cl = feedback(actuator*sysLong, Kq, 1, 4); %inner feedback control loop
Ktheta = -50; %pitch angle gain
outer_cl = feedback(Ktheta*inner_cl, 1, 1, 3); %outer feedback control loop

%visulization
figure(1)
step(outer_cl)