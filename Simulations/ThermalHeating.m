% High-speed: Sutton–Graves reentry heating https://tfaws.nasa.gov/TFAWS12/Proceedings/Aerothermodynamics%20Course.pdf
% Low-speed: Newtonian convection https://en.wikipedia.org/wiki/Newton%27s_law_of_cooling

clear; clc; close all;

% Geometry / mass
A_cm = 5;
B_cm = 6;

m = 40; % g
m = m/1000; % kg

Cd = 1.28; % ceof of drag
g  = 9.81;

% Thermal / material parameters
cp    = 900; % thermal capacity J/(kg*K) aluminum/fr4(?)
eps   = 0.8; % emissivity
sigma = 5.670374419e-8; % stefan boltzmann constant
T0    = 300; % initial temperature [K]

% Heating model parameters
C_sg  = 1.83e-4; % Sutton–Graves constant (Earth)


V_switch = 1500; % speed threshold

% Initial conditions
h0 = 100000;    % altitude [m]
v0 = 0;         % downward positive [m/s]

tspan = [0 2000];

% Computed:
A  = (A_cm/100) * (B_cm/100);% area [m^2]


%Rn = sqrt(A/pi)/2; % effective nose radius [m]
t = 0.0016; % 1.6 mm PCB thickness
Rn = t/2; % effective nose radius

% pack params
params.m = m;
params.Cd = Cd;
params.A = A;
params.g = g;
params.cp = cp;
params.eps = eps;
params.sigma = sigma;
params.C_sg = C_sg;
params.Rn = Rn;
params.V_switch = V_switch;

% ODE 
y0 = [h0; v0; T0];

options = odeset('Events', @(t,y) groundEvent(t,y));

[t, y] = ode45(@(t,y) flatPlateEOM_thermal_hybrid(t, y, params), tspan, y0, options);

h = y(:,1);
v = y(:,2);
T = y(:,3);

% Plots
figure;
hold on;

yyaxis left;
plot(t, v, 'LineWidth', 1.5, 'DisplayName','Velocity vs time');
grid on;
xlabel('Time (s)');
ylabel('Downward velocity v (m/s)');

yyaxis right;
plot(t, T-273.15, 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('ChipSat temperature T (deg C)');
title('ChipSat Temperature / Velocity vs Time');
set(gca,'fontsize', 20) 

debug = 0;
if debug
    figure;
    subplot(1,2,1);
    plot(h, v, 'LineWidth', 1.5);
    grid on;
    xlabel('Altitude h (m)');
    ylabel('Downward velocity v (m/s)');
    title('ChipSat Drop: Velocity vs Altitude');
    set(gca, 'XDir', 'reverse');

    subplot(1,2,2);
    plot(t, h, 'LineWidth', 1.5);
    grid on;
    xlabel('Time (s)');
    ylabel('Altitude h (m)');
    title('ChipSat Drop: Altitude vs Time');
end

function dydt = flatPlateEOM_thermal_hybrid(t, y, params)
    % State
    h = y(1); % altitude
    v = y(2); % velocity
    T = y(3); % temperature

    % params
    m = params.m;
    Cd = params.Cd;
    A  = params.A;
    g = params.g;
    cp = params.cp;
    eps = params.eps;
    sigma = params.sigma;
    C_sg = params.C_sg;
    Rn = params.Rn;
    V_switch = params.V_switch;

    % Atmosphere details
    rho   = densityAtAlt(h);
    T_inf = tempAtAlt(h);

    % Drag 
    D = 0.5 * rho * Cd * A * v^2;

    % EOM
    dhdt = -v;
    dvdt = g - D/m;

    % Convective heating 
    Vmag = abs(v);
    if Vmag > V_switch
        % Sutton–Graves
        q_conv = C_sg * sqrt(rho / Rn) * Vmag^3;
    else
        % Newtonian convection
        q_conv = 15 * (T_inf - T);
    end

    % Radiative cooling
    q_rad = eps * sigma * (T^4 - T_inf^4);

    % Lumped thermal energy balance
    dTdt = (A * (q_conv - q_rad)) / (m * cp);

    dydt = [dhdt; dvdt; dTdt];
end

function rho = densityAtAlt(h)
    % https://www.grc.nasa.gov/www/k-12/airplane/atmosmet.html
    if h < 11000
        T = 15.04 - 0.00649*h;
        p = 101.29*((T+273.1)/288.08)^5.256;
    elseif h <= 25000
        T = -56.46;
        p = 22.65 * exp(1.73 - 0.000157*h);
    else
        T = -131.21 + 0.00299*h;
        p = 2.488*((T+273.1)/216.6)^(-11.388);
    end

    rho = p/(0.2869* (T+273.1));
end

function T_K = tempAtAlt(h)
    if h < 0
        h = 0;
    end

    if h < 11000
        T = 15.04 - 0.00649*h;
    elseif h <= 25000
        T = -56.46;
    else
        T = -131.21 + 0.00299*h;
    end

    T_K = T + 273.1;
end

function [value, isterminal, direction] = groundEvent(t, y)
    % Stop when altitude reaches ground
    h         = y(1);
    value      = h; % event when h = 0
    isterminal = 1; % stop integration
    direction  = -1; % only when descending through zero
end

%% 
tempAtAlt(100000)