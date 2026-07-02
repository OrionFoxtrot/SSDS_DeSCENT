%C:\Users\lohat\OneDrive\Desktop\School\Cornell\SSDS\Descent V2 Testing\SSDS_DeSCENT
clc; clear; close all;

p.m = 0.040;          % kg

p.L = 0.060;          % m, 5cm
p.W = 0.050;          % m, 6cm
p.T = 0.004;          % m, 4mm = 0.4cm

Ixx = 1/12 * p.m*p.L^2;
Iyy = 1/12 * p.m*p.W^2;
Izz = 1/12 * p.m*(p.L^2+p.W^2);
p.I = diag([Ixx, Iyy, Izz]);

% Aero coefficient model
p.CdMax = 1.28;     % max drag coefficient when face-on
p.ClMax = 1.10;     % max lift coefficient near 45 deg angle of attack

% Simple fixed rotational damping
p.Cdamp = 1e-6;      % N*m*s, may be very strong for a tiny ChipSat


% COP relative to COM in body frame
% Body x = length, body y = width, body z = plate normal

p.rCOP_B = [5; 1; 0]/1000;   % m, 5 mm offset in body x

% Wind in inertial frame
p.wind_I = [0; 0; 0];

h0 = 100;           % m initial altitude

% Linear 
r0_I = [0; 0; h0];    % inertial position, z positive upward
v0_I = [0; 0; 0];     % inertial velocity

% Angular
roll0  = deg2rad(0);
pitch0 = deg2rad(0);
yaw0   = deg2rad(0);

q0 = eulerToQuat(roll0, pitch0, yaw0);   % body to inertial quaternion
omega0_B = deg2rad([0; 0; 0]);          % body rates [p q r], rad/s

% State:
% x = [r_I; v_I; q_IB; omega_B]
% x = [pos_x, pos_y, pos_z, quaternion, angular velocities]
x0 = [r0_I; v0_I; q0; omega0_B];

tspan = [0 2000];

opts = odeset( ...
    'RelTol', 1e-7, ...
    'AbsTol', 1e-9, ...
    'MaxStep', 0.02, ...
    'Events', @(t,x) groundEvent(t,x,p));

[t, x] = ode45(@(t,x) eomFlatPlate(t,x,p), tspan, x0, opts);


r_I = x(:,1:3);
v_I = x(:,4:6);
q   = x(:,7:10);
w_B = x(:,11:13);

altitude = r_I(:,3);
speed = vecnorm(v_I, 2, 2);
bodyRatesDeg = rad2deg(w_B);

fprintf("Impact time: %.2f s\n", t(end));
fprintf("Impact speed: %.2f m/s\n", speed(end));
fprintf("Final altitude: %.2f m\n", altitude(end));

% Plots
close all
figure;
plot(t, altitude, 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Altitude (m)');
grid on;

figure;
plot(t, speed, 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Speed (m/s)');
grid on;

figure;
hold on;
plot(t, bodyRatesDeg(:,1), 'LineWidth', 1.5); 
plot(t, bodyRatesDeg(:,2), 'LineWidth', 1.5);
plot(t, bodyRatesDeg(:,3), 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Body Rates (deg/s)');
legend('p', 'q', 'r');
grid on;

figure;
hold on
plot3(0,0,h0,'o','MarkerFaceColor', 'r', 'MarkerEdgeColor', 'r', ...
    'MarkerSize',8,'DisplayName','Origin')
plot3(r_I(:,1),r_I(:,2),r_I(:,3),'DisplayName','Path')

xlabel("Inertial X (m)")
ylabel("Inertial Y (m)")

b = floor_axis_lims(r_I(:,1));
xlim([b.min, b.max])
b = floor_axis_lims(r_I(:,2));
ylim([b.min, b.max])

zlabel("Inertial Z (m)")
zlim([0, h0*1.3])
grid on;
view(70,45)
legend()

%% Equations of motion

function xdot = eomFlatPlate(~, x, p)

% State unpacking
r_I = x(1:3);
v_I = x(4:6);
q_IB = x(7:10);
omega_B = x(11:13);

q_IB = q_IB / norm(q_IB); % Normalaize Quat

altitude = max(r_I(3), 0); % Make sure Alt is > 0

rho = atmosphereDensity(altitude); % NASA Atmo Model

R_IB = quatToDCM(q_IB);   % body to inertial
R_BI = R_IB'; 

% Relative velocity through air
vRel_I = v_I - p.wind_I;
vRel_B = R_BI * vRel_I;

% Aero force and moment in body frame
[F_aero_B, M_aero_B] = aeroFlatPlate(vRel_B, omega_B, rho, p);

% Gravity
g_I = [0; 0; -9.80665];

% Translational dynamics
rDot_I = v_I;
vDot_I = g_I + (R_IB * F_aero_B) / p.m;

% Attitude kinematics
qDot_IB = quatDerivative(q_IB, omega_B);

% Rotational dynamics
omegaDot_B = p.I \ (M_aero_B - cross(omega_B, p.I * omega_B));

% Pack derivative
xdot = [rDot_I; vDot_I; qDot_IB; omegaDot_B];

end

%% Flat plate aero model
function [F_B, M_B] = aeroFlatPlate(vRel_B, omega_B, rho, p)

% Inputs:
% vRel_B  = ChipSat velocity relative to air, expressed in body frame
% omega_B = ChipSat angular velocity, expressed in body frame
% rho     = local atmospheric density
% p       = parameter struct
%
% Outputs:
% F_B = aerodynamic force in body frame
% M_B = aerodynamic moment about COM in body frame

V = norm(vRel_B);

if V < 1e-6
    F_B = zeros(3,1);
    M_B = zeros(3,1);
    return;
end

% Dynamic pressure
qbar = 0.5 * rho * V^2;

% Incoming airflow direction
% vRel_B points in the direction the ChipSat is moving through the air.
% Incoming airflow points opposite that.
vAir_B = -vRel_B / V;

% Plate normal and angle of attack
% Body z-axis is the normal to the ChipSat flat plate.
n_B = [0; 0; 1];

% Face-on factor
% mu = 1 means face-on.
% mu = 0 means edge-on.
mu_raw = dot(n_B, vAir_B);
mu = abs(mu_raw);
mu = max(0, min(1, mu));

% Angle between plate normal and incoming flow
alpha = acos(mu);

% Lift and drag coefficients
% Simple flat-plate coefficient model.
% Cd is maximum when face-on.
% Cl is maximum around alpha = 45 deg.
Cd = p.CdMax * mu;
Cl = p.ClMax * sin(2 * alpha);

% Drag force
A = p.L * p.W;

% Drag acts with incoming airflow direction,
% meaning opposite the ChipSat motion through the air.
F_drag_B = qbar * Cd * A * vAir_B;

% Lift force
% Pick the plate face that is exposed to the incoming airflow.
if abs(mu_raw) < 1e-9
    nFace_B = n_B;
else
    nFace_B = sign(mu_raw) * n_B;
end

% Lift direction is the component of plate normal perpendicular to airflow.
l_B = nFace_B - dot(nFace_B, vAir_B) * vAir_B;

if norm(l_B) < 1e-9
    F_lift_B = zeros(3,1);
else
    lhat_B = l_B / norm(l_B);
    F_lift_B = qbar * Cl * A * lhat_B;
end

% Total aerodynamic force
F_B = F_drag_B + F_lift_B;

% Moment from COP / COM offset
% Aerodynamic force acts at COP.
% Moment is computed about the COM.
M_cp_B = cross(p.rCOP_B, F_B);

% Simple fixed rotational damping
% Boring linear damper:
% M_damp = -c_omega * omega
M_damp_B = -p.Cdamp * omega_B;

% Total moment
M_B = M_cp_B + M_damp_B;

end
%% Standard Utility


%% Plot nice function
function b = floor_axis_lims(a)
    if ( max(a)-min(a) < 1 )
        b.min = mean(a,'all')-1;
        b.max = b.min+2;
    else
        b.min = min(a);
        b.max = max(a);
    end
end

%% Atmosphere model
function rho = atmosphereDensity(h)

rho0 = 1.225;      % kg/m^3
H = 8500;          % m, scale height

rho = rho0 * exp(-h / H);

end

%% Quaternion Things

function q = eulerToQuat(roll, pitch, yaw)

cr = cos(roll/2);
sr = sin(roll/2);
cp = cos(pitch/2);
sp = sin(pitch/2);
cy = cos(yaw/2);
sy = sin(yaw/2);

qw = cr*cp*cy + sr*sp*sy;
qx = sr*cp*cy - cr*sp*sy;
qy = cr*sp*cy + sr*cp*sy;
qz = cr*cp*sy - sr*sp*cy;

q = [qw; qx; qy; qz];
q = q / norm(q);

end

function R = quatToDCM(q)

q = q / norm(q);

qw = q(1);
qx = q(2);
qy = q(3);
qz = q(4);

% body vectors into inertial vectors
R = [
    1 - 2*(qy^2 + qz^2),   2*(qx*qy - qw*qz),     2*(qx*qz + qw*qy);
    2*(qx*qy + qw*qz),     1 - 2*(qx^2 + qz^2),   2*(qy*qz - qw*qx);
    2*(qx*qz - qw*qy),     2*(qy*qz + qw*qx),     1 - 2*(qx^2 + qy^2)
    ];

end

function qDot = quatDerivative(q, omega_B)

q = q / norm(q);

qw = q(1);
qx = q(2);
qy = q(3);
qz = q(4);

p = omega_B(1);
qRate = omega_B(2);
r = omega_B(3);

qDot = 0.5 * [ % Hot cross buns operation
    -qx*p - qy*qRate - qz*r;
    qw*p + qy*r     - qz*qRate;
    qw*qRate - qx*r + qz*p;
    qw*r + qx*qRate - qy*p
    ];

end

%% Ground event
function [value, isterminal, direction] = groundEvent(~, x, ~)

altitude = x(3);

value = altitude;      % stop when altitude = 0
isterminal = 1;
direction = -1;

end
