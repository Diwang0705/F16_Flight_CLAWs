%Aircraft model data obtained from the reference [1]: 'Brian L. Stevens, Aircraft Control and Simulation'
%Script edited by Di Wang
%Created time: 14/Feb/2025

%Roll Angle Hold with Pole Placement Modern Control Theory

%F16 Aircraft lateral-directional state space (4 states, 1 input)
A = [-3.2201e-1 6.404e-2 3.6382e-2 -9.9167e-1;
     0 0 1 3.6928e-2;
     -30.649 0 -3.6784 6.6461e-1;
     8.5395 0 -2.5435e-2 -4.7637]; %Four states (Beta, Phi, p, r)
B = [2.9506e-4; 0; -7.3331e-1; -3.1865e-2]; % one input delta_a (aileron angle)
C = eye(4); % Full states estimation
D = zeros(4,1);
sysLat = ss(A,B,C,D);

%Check stability for original aircraft dynamic
%result: stable - all eigen values are at LHP
[T, D] = eig(A); %check eigen values position

%Check controllability of system with single input (elevator angle)
controllability = rank(ctrb(A,B)); %full rank indicates controllable

%% Pole Placement Control Law Design
% According to the handling quality, only spiral mode is adjusted
%eigs = [-0.0667; -2.32+2.1i; -2.32-2.1i; -3.89]; %pole placement
eigs = [-0.0667; -2.32+2.1i; -2.32-2.1i; -2];
K = place(A, B, eigs); %controller gain by pole placement
sysLat_cl = feedback(sysLat, K, -1);

%% check pole positions
figure(1)
subplot(1,2,1)
pzmap(sysLat)
title('pole locations of original aircraft dynamic')
grid on
subplot(1,2,2)
pzmap(sysLat_cl)
title('pole locations of close-loop system ')
grid on