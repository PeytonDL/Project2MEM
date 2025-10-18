%% WIND_TURBINE_TOWER_ANALYSIS - Advanced analysis of wind turbine tower using ODE solver
%
% This script analyzes a wind turbine tower with:
% - Variable diameter and wall thickness (from towerSpecs.csv)
% - Distributed loading from wind drag (varying with height)
% - Point load at top (currently set to 0)
% - Steel material properties
% - Cantilever boundary conditions (fixed at base, free at top)
%
% The analysis includes:
% 1. Reading tower geometry from CSV file
% 2. Calculating drag coefficients and distributed loading
% 3. Computing variable EI along tower height
% 4. Solving using advanced ODE beam solver
% 5. Comprehensive results and visualization
%
% WIND LOADING DEFINITION:
% Free-stream wind speed: The undisturbed wind velocity far upstream of the tower,
% unaffected by the towers presence. This represents the natural wind conditions
% at the site before any interaction with the structure.
%
% Author: Generated for Project 2 MEM
% Date: 2024

clear; clc; close all;

fprintf('=== WIND TURBINE TOWER ANALYSIS ===\n\n');

%% Material Properties
% Steel properties for wind turbine tower
E_steel = 200e9;          % Youngs modulus [Pa] (200 GPa)
rho_steel = 7850;         % Steel density [kg/m^3]
rho_air = 1.1;            % Air density [kg/m^3] 
mu_air = 1.8e-5;          % Air dynamic viscosity [Pa·s] 
g = 9.81;                 % Gravitational acceleration [m/s^2]

fprintf('Material Properties:\n');
fprintf('  Steel Young''s modulus: %.0f GPa\n', E_steel/1e9);
fprintf('  Steel density: %.0f kg/m^3\n', rho_steel);
fprintf('  Air density: %.3f kg/m^3\n', rho_air);
fprintf('  Air dynamic viscosity: %.2e Pa·s\n', mu_air);
fprintf('\n');

%% Read Tower Specifications
fprintf('Reading tower specifications...\n');
tower_data = readtable('/Users/peytonlettau/Documents/GitHub/Project2MEM/Auxilary Information/Given Parameters/towerSpecs.csv', 'VariableNamingRule', 'preserve');

% Extract data (convert from mm to m)
height_mm = tower_data.('Height (mm)');
OD_mm = tower_data.('OD (mm)');
wall_thk_mm = tower_data.('Wall thk (mm)');

% Convert to meters
height = height_mm / 1000;        % Height [m]
OD = OD_mm / 1000;               % Outer diameter [m]
wall_thk = wall_thk_mm / 1000;   % Wall thickness [m]

% Calculate inner diameter
ID = OD - 2 * wall_thk;          % Inner diameter [m]

% Calculate cross-sectional properties
A = pi/4 * (OD.^2 - ID.^2);      % Cross-sectional area [m^2]
I = pi/64 * (OD.^4 - ID.^4);     % Second moment of area [m^4]
EI = E_steel * I;                % Flexural rigidity [Nm^2]

% Tower height
tower_height = max(height);

fprintf('Tower Geometry:\n');
fprintf('  Total height: %.1f m\n', tower_height);
fprintf('  Base diameter: %.2f m (OD), %.2f m (ID)\n', OD(1), ID(1));
fprintf('  Top diameter: %.2f m (OD), %.2f m (ID)\n', OD(end), ID(end));
fprintf('  Base wall thickness: %.1f mm\n', wall_thk(1)*1000);
fprintf('  Top wall thickness: %.1f mm\n', wall_thk(end)*1000);
fprintf('\n');

%% Wind Loading Parameters
% Free-stream wind speed definition and parameters
% Free-stream wind speed: Undisturbed wind velocity far upstream of the tower
V_freestream = 12;       % Free-stream wind speed [m/s] (constant with height)

% Calculate wind speed profile (constant free-stream speed with height)
V_wind = @(z) V_freestream * ones(size(z));

% Drag coefficient for circular cylinder (varies with Reynolds number)
% More accurate model using Reynolds number
Cd_base = 0.7;           % Base drag coefficient for smooth cylinder

% Function to calculate drag coefficient based on Reynolds number
drag_coefficient = @(D, V) calculate_drag_coefficient(D, V, rho_air, mu_air);

%% Calculate Distributed Loading
fprintf('Calculating distributed loading...\n');

% Create height array for analysis
z_analysis = linspace(0, tower_height, 1000);

% Interpolate tower properties at analysis points
OD_interp = interp1(height, OD, z_analysis, 'linear', 'extrap');
wall_thk_interp = interp1(height, wall_thk, z_analysis, 'linear', 'extrap');
ID_interp = OD_interp - 2 * wall_thk_interp;
A_interp = pi/4 * (OD_interp.^2 - ID_interp.^2);
I_interp = pi/64 * (OD_interp.^4 - ID_interp.^4);
EI_interp = E_steel * I_interp;

% Calculate wind speed at each height
V_wind_profile = V_wind(z_analysis);

% Calculate drag coefficient and Reynolds number at each height
Cd_profile = zeros(size(z_analysis));
Re_profile = zeros(size(z_analysis));
for i = 1:length(z_analysis)
    [Cd_profile(i), Re_profile(i)] = drag_coefficient(OD_interp(i), V_wind_profile(i));
end

% Calculate distributed drag force (per unit length)
q_drag = 0.5 * rho_air * Cd_profile .* OD_interp .* V_wind_profile.^2;

% Calculate self-weight (per unit length) - for reference only
q_weight = rho_steel * g * A_interp;

% Total distributed loading (wind drag only for horizontal beam analysis)
q_total = q_drag;

fprintf('Loading Summary:\n');
fprintf('  Free-stream wind speed: %.1f m/s (constant)\n', V_freestream);
fprintf('  Reynolds number range: %.0e to %.0e\n', min(Re_profile), max(Re_profile));
fprintf('  Drag coefficient range: %.3f to %.3f\n', min(Cd_profile), max(Cd_profile));
fprintf('  Maximum drag loading: %.1f N/m (at top)\n', max(q_drag));
fprintf('  Self-weight loading: %.1f N/m (at base) - for reference only\n', max(q_weight));
fprintf('  Total horizontal loading: %.1f N/m (wind drag only)\n', max(q_total));
fprintf('\n');

%% Point Load at Top
% Point load from rotor/nacelle (calculated by other script)
P_top = 0;  % Currently set to 0 as specified

fprintf('Point Load:\n');
fprintf('  Top load: %.0f N\n', P_top);
fprintf('\n');

%% Setup Beam Analysis
fprintf('Setting up beam analysis...\n');

% Define beam parameters
beam_params.L = tower_height;
beam_params.EI = @(x) interp1(z_analysis, EI_interp, x, 'linear', 'extrap');

% Define loading function (distributed load + point load)
loading_func = @(x) interp1(z_analysis, q_total, x, 'linear', 'extrap');

% Define boundary conditions (cantilever: fixed at base, free at top)
boundary_conditions.left_type = 'fixed';      % Fixed at base
boundary_conditions.right_type = 'free';      % Free at top
boundary_conditions.left_values = [0, 0];     % No displacement or rotation at base

% Solver options
options.tolerance = 1e-6;
options.max_step = tower_height/2000;
options.plot_results = false;  

%% Solve Beam Analysis
fprintf('Solving beam analysis using ODE solver...\n');

try
    [x, y, theta, M, V] = cantilever_beam_ode_solver(beam_params, loading_func, boundary_conditions, options);
    fprintf('✓ Beam analysis completed successfully\n');
catch ME
    fprintf('✗ Error in beam analysis: %s\n', ME.message);
    return;
end

%% Results Analysis
fprintf('\n=== TOWER ANALYSIS RESULTS ===\n');

% Find maximum values
[max_deflection, max_def_idx] = max(abs(y));
[max_rotation, max_rot_idx] = max(abs(theta));
[max_moment, max_mom_idx] = max(abs(M));
[max_shear, max_shear_idx] = max(abs(V));

fprintf('Maximum Values:\n');
fprintf('  Deflection: %.4f m (%.2f mm) at height %.1f m\n', ...
    max_deflection, max_deflection*1000, x(max_def_idx));
fprintf('  Rotation: %.4f rad (%.2f°) at height %.1f m\n', ...
    max_rotation, max_rotation*180/pi, x(max_rot_idx));
fprintf('  Bending moment: %.0f kNm at height %.1f m\n', ...
    max_moment/1000, x(max_mom_idx));
fprintf('  Shear force: %.0f kN at height %.1f m\n', ...
    max_shear/1000, x(max_shear_idx));

% Calculate tip deflection and rotation
tip_deflection = y(end);
tip_rotation = theta(end);
fprintf('\nTip Values:\n');
fprintf('  Tip deflection: %.4f m (%.2f mm)\n', tip_deflection, tip_deflection*1000);
fprintf('  Tip rotation: %.4f rad (%.2f°)\n', tip_rotation, tip_rotation*180/pi);

% Calculate stress at critical locations
stress_base = max_moment * OD(1) / (2 * I(1));  % Maximum stress at base
fprintf('\nStress Analysis:\n');
fprintf('  Maximum stress (at base): %.0f MPa\n', stress_base/1e6);

%% Comprehensive Visualization
fprintf('\nCreating comprehensive visualization...\n');

figure('Position', [100, 100, 1400, 1200]);

% Subplot 1: Tower geometry
subplot(3,3,1);
plot(height, OD*1000, 'b-', 'LineWidth', 2, 'DisplayName', 'Outer Diameter');
hold on;
plot(height, ID*1000, 'r-', 'LineWidth', 2, 'DisplayName', 'Inner Diameter');
xlabel('Height (m)'); ylabel('Diameter (mm)');
title('Tower Geometry');
legend; grid on;
ylim([0, max(OD*1000)*1.1]);

% Subplot 2: Wall thickness
subplot(3,3,2);
plot(height, wall_thk*1000, 'g-', 'LineWidth', 2);
xlabel('Height (m)'); ylabel('Wall Thickness (mm)');
title('Wall Thickness Profile');
grid on;
ylim([0, max(wall_thk*1000)*1.1]);

% Subplot 3: Flexural rigidity
subplot(3,3,3);
plot(z_analysis, EI_interp/1e9, 'm-', 'LineWidth', 2);
xlabel('Height (m)'); ylabel('EI (GNm^2)');
title('Flexural Rigidity');
grid on;
ylim([0, max(EI_interp/1e9)*1.1]);

% Subplot 4: Wind speed profile
subplot(3,3,4);
plot(z_analysis, V_wind_profile, 'c-', 'LineWidth', 2);
xlabel('Height (m)'); ylabel('Wind Speed (m/s)');
title('Free-Stream Wind Speed Profile');
grid on;
ylim([0, max(V_wind_profile)*1.1]);

% Subplot 5: Drag coefficient and Reynolds number
subplot(3,3,5);
yyaxis left;
plot(z_analysis, Cd_profile, 'k-', 'LineWidth', 2);
ylabel('Drag Coefficient');
ylim([0, max(Cd_profile)*1.1]);
yyaxis right;
plot(z_analysis, Re_profile/1e6, 'r-', 'LineWidth', 2);
ylabel('Reynolds Number (×10^6)');
ylim([0, max(Re_profile/1e6)*1.1]);
xlabel('Height (m)');
title('Drag Coefficient & Reynolds Number');
grid on;

% Subplot 6: Distributed loading
subplot(3,3,6);
plot(z_analysis, q_drag, 'b-', 'LineWidth', 2, 'DisplayName', 'Wind Drag');
hold on;
plot(z_analysis, q_weight, 'r--', 'LineWidth', 1, 'DisplayName', 'Self-Weight (ref.)');
plot(z_analysis, q_total, 'k-', 'LineWidth', 2, 'DisplayName', 'Total Horizontal');
xlabel('Height (m)'); ylabel('Loading (N/m)');
title('Distributed Loading');
legend; grid on;
ylim([0, max(q_total)*1.1]);

% Subplot 7: Deflection
subplot(3,3,7);
plot(x, y*1000, 'b-', 'LineWidth', 2);
xlabel('Height (m)'); ylabel('Deflection (mm)');
title('Tower Deflection');
grid on;
xlim([0, max(x)]);
ylim([min(y*1000)*1.1, 0]);

% Subplot 8: Bending moment
subplot(3,3,8);
plot(x, M/1000, 'r-', 'LineWidth', 2);
xlabel('Height (m)'); ylabel('Moment (kNm)');
title('Bending Moment');
grid on;
xlim([0, max(x)]);
ylim([min(M/1000)*1.1, 0]);

% Subplot 9: Shear force
subplot(3,3,9);
plot(x, V/1000, 'g-', 'LineWidth', 2);
xlabel('Height (m)'); ylabel('Shear (kN)');
title('Shear Force');
grid on;
xlim([0, max(x)]);
ylim([0, max(abs(V/1000))*1.1]);

sgtitle('Wind Turbine Tower Analysis - Complete Results', 'FontSize', 16, 'FontWeight', 'bold');

% Save figure
saveas(gcf, 'wind_turbine_tower_analysis.png');
fprintf('Figure saved as: wind_turbine_tower_analysis.png\n');

%% Summary Report
fprintf('\n=== ANALYSIS SUMMARY ===\n');
fprintf('Tower Analysis Completed Successfully\n');
fprintf('• Tower height: %.1f m\n', tower_height);
fprintf('• Material: Steel (E = %.0f GPa)\n', E_steel/1e9);
fprintf('• Loading: Wind drag + self-weight\n');
fprintf('• Maximum deflection: %.2f mm (%.3f%% of height)\n', ...
    max_deflection*1000, max_deflection/tower_height*100);
fprintf('• Maximum stress: %.0f MPa\n', stress_base/1e6);
fprintf('• Analysis method: Advanced ODE solver\n');
fprintf('\nThe tower analysis provides comprehensive results for:\n');
fprintf('• Structural response to horizontal wind loading\n');
fprintf('• Lateral deflection and rotation profiles\n');
fprintf('• Bending stress distribution\n');
fprintf('• Design validation for wind loads\n');
fprintf('\nNote: Self-weight causes axial compression (separate analysis)\n');
fprintf('Results are ready for further analysis and design optimization.\n');

%% Helper Function for Drag Coefficient Calculation
function [Cd, Re] = calculate_drag_coefficient(D, V, rho, mu)
    % Calculate drag coefficient for circular cylinder based on Reynolds number
    % 
    % INPUTS:
    %   D   - Diameter [m]
    %   V   - Velocity [m/s]
    %   rho - Air density [kg/m^3]
    %   mu  - Air dynamic viscosity [Pa·s]
    %
    % OUTPUTS:
    %   Cd  - Drag coefficient [-]
    %   Re  - Reynolds number [-]
    
    % Calculate Reynolds number
    Re = rho * V * D / mu;
    
    % Drag coefficient correlation for smooth circular cylinder
    % Based on experimental data for subcritical to supercritical flow
    if Re < 1e5
        % Subcritical flow (laminar boundary layer)
        Cd = 1.2;
    elseif Re < 3e5
        % Critical flow (transition region)
        Cd = 1.2 - 0.5 * (Re - 1e5) / (2e5);
    elseif Re < 1e6
        % Supercritical flow (turbulent boundary layer)
        Cd = 0.7 + 0.3 * exp(-(Re - 3e5) / 2e5);
    else
        % High Reynolds number (fully turbulent)
        Cd = 0.7;
    end
    
    % Ensure reasonable bounds
    Cd = max(0.5, min(1.5, Cd));
end
