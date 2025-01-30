%Reference [1]: 'Brian L. Stevens, Aircraft Control and Simulation'
%Script edited by Di Wang
%Created time: 24/Jan/2025

%Altitude Hold Autopilot
%F16 Longitudinal state space with altitude state
A = [-0.56761 0 1 2.2633e-6;
     0 0 1 0;
     -1.4847 0 -0.47599 -1.4947e-7;
     -500 500 0 0]; %Four states AoA, Theta, q, h
B = [0;0;-0.019781;0]; %one input delta_e elevator angle
C = [0 57.296 0 0;
     0 0 57.296 0;
     0 0 0 1]; %Three outputs theta, q, h
D = [0;0;0];
states = {'AoA' 'theta' 'q' 'h'};
inputs = {'aileron'};
outputs = {'theta' 'q' 'h'};
sysLong = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs); %obtain state space of the aircraft lateral system

actuator = tf([0 1],[0.05 1]); %Tf of elevator angle servo
lead_comp = tf([1 0.3],[3 7.2]); %TF of lead compensator
Kp = -3; %Pitch angle gain
Kq = -2.5; %Pitch rate gain
Sensor = tf([0 1],[0.05 1]); %Altitude sensor

inner_q = feedback(actuator*sysLong, Kq, 1, 2); %Pitch rate q inner loop
inner_theta = feedback(inner_q*Kp, 1, 1, 1); %Pitch angle theta inner loop
outer_cl = feedback(inner_theta*lead_comp, Sensor, 1, 3); %outer c.l

figure(1)
step(1000*outer_cl(3,:))