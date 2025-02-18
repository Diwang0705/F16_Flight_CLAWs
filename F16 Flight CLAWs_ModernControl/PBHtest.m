%PBH controllability test

A = [1 2 3; 2 1 0; 0 2 4]; %Original system dynamic matrix A

%obtain eigen values and reltaed eigen vectors
[V,D] = eig(A);

lambda1 = D(1,1);
lambda2 = D(2,2);
lambda3 = D(3,3);
V1 = V(:,1);
V2 = V(:,2);
V3 = V(:,3);

%Compute the range of (A-lambda*I)
% orth function calculates the number of independent vectors of the matrix
range1 = orth(A - lambda1*eye(3));
range2 = orth(A - lambda2*eye(3));
range3 = orth(A - lambda3*eye(3));

%% visulize range of range1
% Extract the basis vectors of the plane
v1 = range1(:,1); 
v2 = range1(:,2); 
% Define four corner points of the plane (for visualization)
p1 = -2*v1 - 2*v2;
p2 = -2*v1 + 2*v2;
p3 =  2*v1 + 2*v2;
p4 =  2*v1 - 2*v2;
% Create a figure
figure(1);
% Plot the transparent plane using fill3
fill3([p1(1), p2(1), p3(1), p4(1)], ...
      [p1(2), p2(2), p3(2), p4(2)], ...
      [p1(3), p2(3), p3(3), p4(3)], ...
      'c', 'FaceAlpha', 0.5, 'EdgeColor', 'none'); % Transparent cyan plane
hold on

% visulize range of range2
% Extract the basis vectors of the plane
v3 = range2(:,1); 
v4 = range2(:,2); 
% Define four corner points of the plane (for visualization)
p11 = -2*v3 - 2*v4;
p22 = -2*v3 + 2*v4;
p33 =  2*v3 + 2*v4;
p44 =  2*v3 - 2*v4;
% Plot the transparent plane using fill3
fill3([p11(1), p22(1), p33(1), p44(1)], ...
      [p11(2), p22(2), p33(2), p44(2)], ...
      [p11(3), p22(3), p33(3), p44(3)], ...
      'r', 'FaceAlpha', 0.5, 'EdgeColor', 'none'); % Transparent cyan plane
grid on
hold on

% visulize range of range3
% Extract the basis vectors of the plane
v5 = range3(:,1); 
v6 = range3(:,2); 
% Define four corner points of the plane (for visualization)
p111 = -2*v5 - 2*v6;
p222 = -2*v5 + 2*v6;
p333 =  2*v5 + 2*v6;
p444 =  2*v5 - 2*v6;
% Plot the transparent plane using fill3
fill3([p111(1), p222(1), p333(1), p444(1)], ...
      [p111(2), p222(2), p333(2), p444(2)], ...
      [p111(3), p222(3), p333(3), p444(3)], ...
      'g', 'FaceAlpha', 0.5, 'EdgeColor', 'none'); % Transparent cyan plane
grid on
hold on

%plot the actuator matrix B which makes the system controllable
b = [0.5;-1.25;5];
quiver3(0, 0, 0, b(1), b(2), b(3), 'b', 'LineWidth', 2);