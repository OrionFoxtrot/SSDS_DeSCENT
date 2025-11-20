% High-speed: Sutton–Graves reentry heating https://tfaws.nasa.gov/TFAWS12/Proceedings/Aerothermodynamics%20Course.pdf
% Low-speed: Newtonian convection https://en.wikipedia.org/wiki/Newton%27s_law_of_cooling
clear; clc; close all;


A_cm = 5;      
B_cm = 6;      

m = 40; % g
m = m/1000; % kg

Cd   = 1.28;
g    = 9.81;   

% Thermal / material parameters
cp   = 900;    % J/(kg*K), e.g. aluminum-like
eps  = 0.8;    % emissivity
sigma = 5.670374419e-8;   % Stefan-Boltzmann [W/m^2/K^4]
T0   = 300;    % initial plate temperature [K]

% Sutton–Graves constant (Earth, SI units: W/m^2)
C_sg = 1.83e-4;

% Low-speed convection parameters (very rough engineering values)
h_c0 = 15; % baseline convective coefficient [W/m^2/K]
rho_ref = 1.225; % reference density at sea level [kg/m^3]
V_ref   = 50; % reference speed for scaling [m/s]

% Speed threshold between regimes
V_switch = 500;  % [m/s] above this, use Sutton–Graves; below, Newtonian

% Initial conditions
h0   = 100000;  % initial altitude [m]
v0   = 0;       % starting from rest, downward positive [m/s]

tspan = [0 2000]; 

% Geometry
A = (A_cm/100) * (B_cm/100);         % area [m^2]
Rn = sqrt(A/pi)/2;                   % rough nose radius [m] for heating correlation

% PACK PARAMETERS
params.m      = m;
params.Cd     = Cd;
params.A      = A;
params.g      = g;
params.cp     = cp;
params.eps    = eps;
params.sigma  = sigma;
params.C_sg   = C_sg;
params.Rn     = Rn;
params.V_switch = V_switch;

% ODE SETUP
y0 = [h0; v0; T0]; 

options = odeset('Events', @(t,y) groundEvent(t,y));

% Integrate
[t, y] = ode45(@(t,y) flatPlateEOM_thermal_hybrid(t, y, params), tspan, y0, options);

h = y(:,1);  
v = y(:,2);   
T = y(:,3);  


% PLOTS

% Velocity vs time
figure;
hold on
yyaxis left
plot(t, v, 'LineWidth', 1.5, 'DisplayName','Velocity vs time');
grid on;
xlabel('Time (s)');
ylabel('Downward velocity v (m/s)');


yyaxis right
plot(t, T, 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('ChipSat temperature T (K)');
title('ChipSat Temperature/Vel vs Time');

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

    h = y(1); % altitude 
    v = y(2); % velocity (downward positive)
    T = y(3); % plate temperature [K]

    m      = params.m;
    Cd     = params.Cd;
    A      = params.A;
    g      = params.g;
    cp     = params.cp;

    %heating
    eps    = params.eps;
    sigma  = params.sigma;
    C_sg   = params.C_sg;
    Rn     = params.Rn;
    V_switch = params.V_switch;

    rho = densityAtAlt(h);
    T_inf = tempAtAlt(h);

    % Drag force magnitude
    D = 0.5 * rho * Cd * A * v^2;   

    % Equations of motion
    dhdt = -v;           
    dvdt = g - D/m;      

    % Convective flux (hybrid)
    Vmag = abs(v);
    
    if Vmag > V_switch
        % High-speed Sutton–Graves heating
        q_conv = C_sg * sqrt(rho / Rn) * Vmag^3;
        fprintf('%d %d %d \n', Vmag,t, V_switch)
    else
        % Low-speed Newtonian convection
        % 15 for forced convection at low speed
        q_conv = 15 * (T_inf - T);
    end

    % Radiative cooling
    q_rad = eps * sigma * (T^4 - T_inf^4);

    % Lumped thermal energy balance
    dTdt = (A * (q_conv - q_rad)) / (m * cp);

    dydt = [dhdt; dvdt; dTdt];
end


function [rho] = densityAtAlt(h)

    % https://www.grc.nasa.gov/www/k-12/airplane/atmosmet.html
    if h < 11000
        T = 15.04 - 0.00649*h;
        p = 101.29*((T+273.1)/288.08)^5.256;
    elseif (h >= 11000 && h <= 25000)
        T = -56.46;
        p = 22.65 * exp(1.73 - 0.000157*h);
    elseif h > 25000
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
    elseif (h >= 11000 && h <= 25000)
        T = -56.46;
    elseif h > 25000
        T = -131.21 + 0.00299*h;
    end
    T_K = T + 273.1;
end

function [value, isterminal, direction] = groundEvent(~, y)
   
    h = y(1);
    value      = h; % when value = 0, event triggers
    isterminal = 1; % stop the integration
    direction  = -1; % only when crossing from positive to negative
end
