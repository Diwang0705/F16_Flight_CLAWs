%Reference [1]: 'Brian L. Stevens, Aircraft Control and Simulation'
%Script edited by Di Wang
%Created time: 22/Jan/2025

%Heading Hold Autopilot
%F16 Aircraft lateral-directional state space (4 states, 1 input)
A = [-0.1315 0.14858 0.32434 -0.93964;
     0 0 1 0.33976;
     -10.614 0 -1.1793 1.0023;
     0.99655 0 -0.00182 -0.25855];
B = [0.00012;
     0;
     -0.10316;
     -0.00213];
C = eye(4);
D = zeros(4,1);
states = {'beta' 'phi' 'roll_rate(p)' 'yaw_rate(r)'};
inputs = {'aileron'};
outputs = {'beta' 'phi' 'roll_rate(p)' 'yaw_rate(r)'};
sysLat = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs); %obtain state space of the aircraft lateral system

Actuator = tf([0 1],[0.005 1]); %Aileron angle servo
Sensor = tf([0 10],[1 10]); %Achieved Yaw angle sensor
Kpsi = 80; %Yaw angle gain
Kphi = -6469; %Roll angle gain
Kp = 0.2785; %Roll rate gain
Phi2Psi = tf([0 9.81], [637 0]); %roll to yaw angle transform in s-domain with airspeed 637ft/s

inner_cl_p = feedback(Kphi*Actuator*sysLat, Kp, 1, 3);
inner_cl_phi = feedback(inner_cl_p, 1, 1, 2);
%outer_cl = feedback(Kpsi*inner_cl_phi*Phi2Psi, Sensor); %Wrong code
roll_angle = inner_cl_phi(2, :); % Extract the roll angle output from inner_cl_phi
yaw_angle = Phi2Psi * roll_angle; % Transform roll angle to yaw angle
outer_cl = feedback(Kpsi * yaw_angle, Sensor); % Outer loop: yaw angle feedback

%visulizaiton
figure(1)
step(outer_cl)