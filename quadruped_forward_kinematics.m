clc; close all; clear all;
% syms theta1 theta2 theta3 theta4 L1 L2 e m n;
e= 12.2/2;
L1= 40;
L3= 39.8153;
L2= 40;
L4= 40;
E= 31.7750;
F= 30.8076;

theta1_deg = 60; % Joint angle 1 in degrees
theta2_deg = 135; % Joint angle 2 in degrees
% Convert degrees to radians
theta1 = deg2rad(theta1_deg);
theta2 = deg2rad(theta2_deg);

x_3 = e + L1 * cos(theta1);
y_3 = L1 * sin(theta1);
x_4 = -e + L1 * cos(theta2);
y_4 = L1 * sin(theta2);
d3_4 = sqrt((x_3 - x_4)^2 + (y_3 - y_4)^2);

% Compute theta3 and theta4
theta3_ = (pi) + atan2(y_3 - y_4, x_3 - x_4) - acos(d3_4 / (2 * L2));
theta3 = theta3_ - theta1;

theta4_ = atan2(y_3 - y_4, x_3 - x_4) + acos(d3_4 / (2 * L2));
theta4 = theta2 - theta4_;

disp(rad2deg(theta3_));
disp(rad2deg(theta4_));
disp(rad2deg(theta3));
disp(rad2deg(theta4));

% ------------------------------Forward Kinematics-----------------
%right
T0_2 = [1, 0, 0, -e;
    0, 1, 0, 0;
    0, 0, 1, 0;
    0, 0, 0, 1;];

T2_4 = [cos(theta2), -sin(theta2), 0, L2 * cos(theta2);
    sin(theta2), cos(theta2), 0, L2 * sin(theta2);
    0, 0, 1, 0;
    0, 0, 0, 1];

T4_6 = [cos(-theta4), -sin(-theta4), 0, (L2 + E) * cos(-theta4);
    sin(-theta4), cos(-theta4), 0, (L2+E) * sin(-theta4);
    0, 0, 1, 0;
    0, 0, 0, 1];
T6_7= [cos(1.5707), -sin(1.5707), 0, F * cos(1.5707);
    sin(1.5707), cos(1.5707), 0, F * sin(1.5707);
    0, 0, 1, 0;
    0, 0, 0, 1];
transformation_of_right_side_link= T0_2 * T2_4 * T4_6 * T6_7;
% -------

T0_1 = [1, 0, 0, e;
    0, 1, 0, 0;
    0, 0, 1, 0;
    0, 0, 0, 1;];

T1_3 = [cos(theta1), -sin(theta1), 0, L1 * cos(theta1);
    sin(theta1), cos(theta1), 0, L1 * sin(theta1);
    0, 0, 1, 0;
    0, 0, 0, 1];

T3_5 = [cos(theta3), -sin(theta3), 0, L1 * cos(theta3);
    sin(theta3), cos(theta3), 0, L1 * sin(theta3);
    0, 0, 1, 0;
    0, 0, 0, 1];



% Compute points for plotting
P0 = [0; 0; 0; 1];
P1 = T0_2 * [0; 0; 0; 1];
P2 = T0_2 * T2_4 * [0; 0; 0; 1];
P3 = T0_2 * T2_4 * T4_6 * [0; 0; 0; 1];
P4 = T0_2 * T2_4 * T4_6 * T6_7 * [0; 0; 0; 1];
P5 = T0_1 * [0; 0; 0; 1];
P6 = T0_1 * T1_3 * [0; 0; 0; 1];
P7 = T0_1 * T1_3 * T3_5 * [0; 0; 0; 1];

% Plot
figure;
hold on;
plot3([P0(1) P1(1)], [P0(2) P1(2)], [P0(3) P1(3)], 'b', 'LineWidth', 2);
plot3([P1(1) P2(1)], [P1(2) P2(2)], [P1(3) P2(3)], 'g', 'LineWidth', 2);
plot3([P2(1) P3(1)], [P2(2) P3(2)], [P2(3) P3(3)], 'r', 'LineWidth', 2);
plot3([P3(1) P4(1)], [P3(2) P4(2)], [P3(3) P4(3)], 'm', 'LineWidth', 2);
plot3([P0(1) P5(1)], [P0(2) P5(2)], [P0(3) P5(3)], 'c', 'LineWidth', 2);
plot3([P5(1) P6(1)], [P5(2) P6(2)], [P5(3) P6(3)], 'y', 'LineWidth', 2);
plot3([P6(1) P7(1)], [P6(2) P7(2)], [P6(3) P7(3)], 'k', 'LineWidth', 2);
scatter3(P0(1), P0(2), P0(3), 100, 'k', 'filled');
scatter3(P1(1), P1(2), P1(3), 100, 'b', 'filled');
scatter3(P2(1), P2(2), P2(3), 100, 'g', 'filled');
scatter3(P3(1), P3(2), P3(3), 100, 'r', 'filled');
scatter3(P4(1), P4(2), P4(3), 100, 'm', 'filled');
scatter3(P5(1), P5(2), P5(3), 100, 'c', 'filled');
scatter3(P6(1), P6(2), P6(3), 100, 'y', 'filled');
scatter3(P7(1), P7(2), P7(3), 100, 'k', 'filled');
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Transformation of the manipulator');
legend('Base to Joint 1', 'Joint 1 to Joint 2', 'Joint 2 to Joint 3', 'Joint 3 to End Effector',...
    'Base to Joint 4', 'Joint 4 to Joint 5', 'Joint 5 to End Effector');
view(3)
grid on;
axis equal;
hold off;