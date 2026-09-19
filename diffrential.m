clc;
clear;
close all;

%% =========================================================
%        DIFFERENTIAL DRIVE AMR - PATH FOLLOWING
% ==========================================================

%% ================= ROBOT PARAMETERS ======================

r = 0.05;       % Wheel radius (m)
L = 0.30;       % Distance between left and right wheels (m)

dt = 0.02;      % Simulation time step (s)
T  = 60;        % Maximum simulation time (s)

%% ================= INSTRUCTED PATH ========================

% Give the path that the AMR must follow.
%
% Each row = [X Y] waypoint in metres.

path = [
    0.0  0.0
    1.0  0.0
    2.0  0.0
    2.0  1.0
    2.0  2.0
    3.0  2.0
    4.0  2.0
    4.0  3.0
];

%% ================= INITIAL STATE ==========================

X = path(1,1);
Y = path(1,2);

theta = 0;      % Initial orientation (rad)

%% ================= CONTROLLER GAINS =======================

Kv = 0.8;       % Forward velocity gain
Kw = 2.5;       % Heading gain

%% ================= LIMITS ================================

max_v = 0.50;       % Maximum linear velocity (m/s)
max_omega = 2.0;    % Maximum angular velocity (rad/s)

%% ================= WAYPOINT SETTINGS =====================

waypoint_threshold = 0.12;

current_waypoint = 2;

%% ================= DATA STORAGE ===========================

time_data = [];
X_data = [];
Y_data = [];
theta_data = [];

v_data = [];
omega_data = [];

vL_data = [];
vR_data = [];

rpmL_data = [];
rpmR_data = [];

error_data = [];
heading_error_data = [];

%% ================= FIGURE ================================

figure( ...
    'Name','Differential Drive AMR - Path Following',...
    'NumberTitle','off',...
    'Color','w',...
    'Position',[100 80 1200 700]);

%% =========================================================
%                    MAIN SIMULATION
% ==========================================================

for t = 0:dt:T

    %% =====================================================
    %  CHECK FINAL DESTINATION
    % ======================================================

    if current_waypoint > size(path,1)

        disp(' ');
        disp('======================================');
        disp('       PATH COMPLETED SUCCESSFULLY');
        disp('======================================');

        break;
    end

    %% =====================================================
    %  CURRENT TARGET WAYPOINT
    % ======================================================

    targetX = path(current_waypoint,1);
    targetY = path(current_waypoint,2);

    %% =====================================================
    %  POSITION ERROR
    % ======================================================

    errorX = targetX - X;
    errorY = targetY - Y;

    distance_error = sqrt(errorX^2 + errorY^2);

    %% =====================================================
    %  CHECK WHETHER WAYPOINT IS REACHED
    % ======================================================

    if distance_error < waypoint_threshold

        current_waypoint = current_waypoint + 1;

        continue;
    end

    %% =====================================================
    %  DESIRED HEADING
    % ======================================================

    theta_desired = atan2(errorY,errorX);

    %% =====================================================
    %  HEADING ERROR
    % ======================================================

    heading_error = theta_desired - theta;

    % Normalize angle to [-pi, pi]

    heading_error = atan2( ...
        sin(heading_error),...
        cos(heading_error));

    %% =====================================================
    %  CONTROLLER
    % ======================================================

    v = Kv * distance_error;

    omega = Kw * heading_error;

    %% =====================================================
    %  VELOCITY LIMIT
    % ======================================================

    v = min(v,max_v);

    omega = max( ...
        min(omega,max_omega),...
        -max_omega);

    %% =====================================================
    %  DIFFERENTIAL DRIVE INVERSE KINEMATICS
    % ======================================================

    % Right wheel velocity

    vR = v + (L/2)*omega;

    % Left wheel velocity

    vL = v - (L/2)*omega;

    %% =====================================================
    %  WHEEL ANGULAR VELOCITIES
    % ======================================================

    omegaR = vR/r;
    omegaL = vL/r;

    %% =====================================================
    %  WHEEL RPM
    % ======================================================

    rpmR = omegaR * 60/(2*pi);
    rpmL = omegaL * 60/(2*pi);

    %% =====================================================
    %  DIFFERENTIAL DRIVE FORWARD KINEMATICS
    % ======================================================

    v_actual = (vR + vL)/2;

    omega_actual = (vR - vL)/L;

    %% =====================================================
    %  ROBOT KINEMATIC EQUATIONS
    % ======================================================

    X_dot = v_actual*cos(theta);

    Y_dot = v_actual*sin(theta);

    theta_dot = omega_actual;

    %% =====================================================
    %  STATE UPDATE
    % ======================================================

    X = X + X_dot*dt;

    Y = Y + Y_dot*dt;

    theta = theta + theta_dot*dt;

    %% =====================================================
    %  STORE DATA
    % ======================================================

    time_data(end+1) = t;

    X_data(end+1) = X;
    Y_data(end+1) = Y;
    theta_data(end+1) = theta;

    v_data(end+1) = v_actual;
    omega_data(end+1) = omega_actual;

    vL_data(end+1) = vL;
    vR_data(end+1) = vR;

    rpmL_data(end+1) = rpmL;
    rpmR_data(end+1) = rpmR;

    error_data(end+1) = distance_error;
    heading_error_data(end+1) = heading_error;

    %% =====================================================
    %  LIVE DISPLAY
    % ======================================================

    clf;

    %% -----------------------------------------------------
    % 1. PATH AND ROBOT
    % ------------------------------------------------------

    subplot(2,2,1);

    plot(path(:,1),path(:,2),'k--',...
        'LineWidth',2);

    hold on;

    plot(X_data,Y_data,'b',...
        'LineWidth',2);

    plot(path(:,1),path(:,2),'ro',...
        'MarkerSize',7,...
        'LineWidth',1.5);

    % Current target

    if current_waypoint <= size(path,1)

        plot( ...
            path(current_waypoint,1),...
            path(current_waypoint,2),...
            'gx',...
            'MarkerSize',14,...
            'LineWidth',3);
    end

    % Robot

    plot(X,Y,'bo',...
        'MarkerFaceColor','b',...
        'MarkerSize',10);

    % Robot heading arrow

    arrow_length = 0.30;

    quiver( ...
        X,Y,...
        arrow_length*cos(theta),...
        arrow_length*sin(theta),...
        0,...
        'r',...
        'LineWidth',2,...
        'MaxHeadSize',1);

    grid on;

    axis equal;

    xlabel('X (m)');
    ylabel('Y (m)');

    title('AMR Path Following');

    legend( ...
        'Instructed Path',...
        'Actual Path',...
        'Waypoints',...
        'Current Target',...
        'AMR',...
        'Location','best');

    %% -----------------------------------------------------
    % 2. ROBOT STATE
    % ------------------------------------------------------

    subplot(2,2,2);

    axis off;

    text(0.02,0.92,...
        'AMR STATE',...
        'FontSize',15,...
        'FontWeight','bold');

    text(0.02,0.80,...
        sprintf('X = %.3f m',X),...
        'FontSize',12);

    text(0.02,0.71,...
        sprintf('Y = %.3f m',Y),...
        'FontSize',12);

    text(0.02,0.62,...
        sprintf('\\theta = %.2f deg',rad2deg(theta)),...
        'FontSize',12);

    text(0.02,0.53,...
        sprintf('Linear velocity = %.3f m/s',v_actual),...
        'FontSize',12);

    text(0.02,0.44,...
        sprintf('Angular velocity = %.3f rad/s',omega_actual),...
        'FontSize',12);

    text(0.02,0.35,...
        sprintf('Path error = %.3f m',distance_error),...
        'FontSize',12);

    text(0.02,0.26,...
        sprintf('Heading error = %.2f deg',...
        rad2deg(heading_error)),...
        'FontSize',12);

    text(0.02,0.17,...
        sprintf('Waypoint = %d / %d',...
        min(current_waypoint,size(path,1)),...
        size(path,1)),...
        'FontSize',12);

    %% -----------------------------------------------------
    % 3. WHEEL INPUT / OUTPUT
    % ------------------------------------------------------

    subplot(2,2,3);

    bar([vL vR]);

    grid on;

    set(gca,...
        'XTick',[1 2],...
        'XTickLabel',...
        {'LEFT','RIGHT'});

    ylabel('Velocity (m/s)');

    title('Wheel Velocity Input');

    %% -----------------------------------------------------
    % 4. ERROR + STATUS
    % ------------------------------------------------------

    subplot(2,2,4);

    axis off;

    text(0.02,0.90,...
        'CONTROL STATUS',...
        'FontSize',15,...
        'FontWeight','bold');

    text(0.02,0.75,...
        sprintf('Distance Error : %.4f m',...
        distance_error),...
        'FontSize',12);

    text(0.02,0.62,...
        sprintf('Heading Error : %.2f deg',...
        rad2deg(heading_error)),...
        'FontSize',12);

    text(0.02,0.49,...
        sprintf('Left Wheel : %.2f RPM',rpmL),...
        'FontSize',12);

    text(0.02,0.36,...
        sprintf('Right Wheel : %.2f RPM',rpmR),...
        'FontSize',12);

    text(0.02,0.23,...
        sprintf('Time : %.2f s',t),...
        'FontSize',12);

    if distance_error < waypoint_threshold

        status = 'WAYPOINT REACHED';

    else

        status = 'FOLLOWING PATH';

    end

    text(0.02,0.10,...
        status,...
        'FontSize',13,...
        'FontWeight','bold');

    drawnow;

end

%% =========================================================
%                    FINAL RESULTS
% ==========================================================

figure('Name','AMR Performance Results',...
       'Color','w');

subplot(3,1,1);

plot(time_data,X_data,'LineWidth',1.5);
hold on;
plot(time_data,Y_data,'LineWidth',1.5);

grid on;

xlabel('Time (s)');
ylabel('Position (m)');

legend('X','Y');

title('AMR Position');

subplot(3,1,2);

plot(time_data,error_data,'LineWidth',1.5);

grid on;

xlabel('Time (s)');
ylabel('Error (m)');

title('Path Following Error');

subplot(3,1,3);

plot(time_data,...
    rad2deg(theta_data),...
    'LineWidth',1.5);

grid on;

xlabel('Time (s)');
ylabel('\theta (deg)');

title('AMR Orientation');