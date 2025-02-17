%Aircraft model data obtained from the reference [1]: 'Brian L. Stevens, Aircraft Control and Simulation'
%Script edited by Di Wang
%Created time: 15/Feb/2025

%Altitude Hold with Loop Shaping Modern Control Theory

%F16 Aircraft longitudinal state space (4 states, 1 input, 3 measuerd outputs)
A = [-0.56761 0 1 2.2633e-6;
     0 0 1 0;
     -1.4847 0 -0.47599 -1.4947e-7;
     -500 500 0 0]; %Four states AoA, Theta, q, h
B = [0;0;-0.019781;0]; %one input delta_e elevator angle
C = eye(4); %Three outputs theta, q, h
D = [0;0;0;0];
states = {'AoA' 'theta' 'q' 'h'};
inputs = {'elevator'};
outputs = {'AoA' 'theta' 'q' 'h'};
sysLong = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);%longitudinal state space model
sysLong_tf = tf(sysLong);

%Check stability for original aircraft dynamic
%result: stable - all eigen values are at LHP
[T, D] = eig(A); %check eigen values position
%the phugoid period pole pair is unstable

%Check controllability of system with single input (elevator angle)
controllability = rank(ctrb(A,B)); %full rank indicates controllable

%% Control law Design
%check bode plot of ol TF from elevator angle to altitude
TF_e2h = sysLong_tf(4);
%output bode plot for openloop system
figure(1)
subplot(1,3,1)
bode(TF_e2h)
grid on
title('bodeplot_ol')
%output nyquist plot for openloop system
figure(2)
subplot(1,3,1)
nyquist(TF_e2h)
grid on
title('nyquistplot_ol')

%Pole placement control law
desired_eigs = [-1.5219 + 1.2177i; -1.5219 - 1.2177i; -0.2 + 0.313i; -0.2 - 0.313i];
K_pp = place(A, B, desired_eigs); %gain controller by pole placements method
sysLong_cl = feedback(sysLong, K_pp, -1);
sysLong_cl_tf = tf(sysLong_cl);
TF_e2h_cl = sysLong_cl_tf(4);
%output bode plot for closeloop system with gain controlelr by pole placement
figure(1)
subplot(1,3,2)
bode(TF_e2h_cl)
grid on
title('bodeplot_cl_PP')
%output nyquist plot for openloop system
figure(2)
subplot(1,3,2)
nyquist(TF_e2h_cl)
grid on
title('nyquistplot_cl_PP')

%Robust control law H-infinity
% s = tf('s');
% Tr = 10;
% Ws = 1/(1+Tr*s); %sensitivity weighting function (low pass filter)
% Wt = (Tr*s)/(1+Tr*s); %complementary sensitivity weighting function (high pass filter)
[Kh,CL,GAM] = hinfsyn(sysLong,4,1);
sysLong_cl_H = feedback(sysLong, Kh, 1);
sysLong_cl_H_tf = tf(sysLong_cl_H);
TF_e2h_cl_H = sysLong_cl_H_tf(4);
%output bode plot for closeloop system by H-Inifinity controller
figure(1)
subplot(1,3,3)
bode(TF_e2h_cl_H)
grid on
title('bodeplot_cl_H')
%output nyquist plot for openloop system
figure(2)
subplot(1,3,3)
nyquist(TF_e2h_cl_H)
grid on
title('nyquistplot_cl_H')