function [CP_max, optimal_conditions] = Deliverable2(V_wind, lambda, pitch_min, pitch_max, pitch_step)
% DELIVERABLE2 Wind Turbine Pitch Angle Optimization
% Finds maximum power coefficient by varying pitch angle at fixed wind speed and tip speed ratio
% Usage: [CP_max, optimal_conditions] = Deliverable2(8, 6.91, -15, 15, 1);

    clc; close all;
    fprintf('=== Wind Turbine Pitch Angle Optimization ===\n\n');
    
    fprintf('Extracting wind turbine parameters...\n');
    data = ParameterExtraction();

    % Defaults from predefined deliverables when inputs are omitted
    if nargin < 1 || isempty(V_wind), V_wind = data.deliverables.part2.V_wind; end
    if nargin < 2 || isempty(lambda), lambda = data.deliverables.part2.lambda; end
    if nargin < 3 || isempty(pitch_min), pitch_min = -15; end
    if nargin < 4 || isempty(pitch_max), pitch_max = 15; end
    if nargin < 5 || isempty(pitch_step), pitch_step = 1; end

    fprintf('Optimizing CP for:\n');
    fprintf('  Wind velocity: %.1f m/s\n', V_wind);
    fprintf('  Tip speed ratio: %.2f\n', lambda);
    fprintf('  Pitch angle range: %.1f to %.1f degrees\n', pitch_min, pitch_max);
    fprintf('  Step size: %.1f degrees\n\n', pitch_step);
    
    R = data.turbine.performance.rotorRadius;
    A = data.turbine.calculated.rotorArea;
    rho = data.materials.air.density;
    
    pitch_range = pitch_min:pitch_step:pitch_max;
    n_points = length(pitch_range);
    CP_values = zeros(n_points, 1);
    CT_values = zeros(n_points, 1);
    P_values = zeros(n_points, 1);
    T_values = zeros(n_points, 1);
    
    fprintf('Performing pitch angle optimization...\n');
    blade_profile = data.blade.profile;
    % Precompute station-wise invariants for clarity and speed
    n_stations = height(blade_profile); % number of spanwise blade stations (rows in profile)
    r_stations = blade_profile.DistanceFromCenterOfRotation / 1000; % station radii [m]
    chord_m = blade_profile.ChordLength / 1000; % chord lengths [m]
    twist_deg_vec = blade_profile.BladeTwist; % twist [deg]
    airfoil_raw = blade_profile.Airfoil; % raw airfoil labels
    lambda_r_vec = lambda * r_stations / R; % local TSR at each station
    B = 3;
    
    % Calculate rotational velocity from tip speed ratio
    omega_rad = (lambda * V_wind) / R;
    omega_rpm = omega_rad * 60 / (2 * pi);
    
    fprintf('  Rotational speed: %.1f rpm\n', omega_rpm);
    
    for j = 1:n_points
        pitch_angle = pitch_range(j);
        
        dT = zeros(n_stations, 1);
        dQ = zeros(n_stations, 1);
        dP = zeros(n_stations, 1);
        
        pitch_rad = deg2rad(pitch_angle);
        for i = 1:n_stations
            r = r_stations(i);
            c = chord_m(i);
            twist = twist_deg_vec(i);
            airfoil = airfoil_raw{i};
            
            lambda_r = lambda_r_vec(i); % local tip-speed ratio at radius r
            
            % Section aerodynamics via helper (handles circle vs airfoil polars)
            [a, a_prime, CL, CD, Cn, Ct, V_rel] = getSectionCoefficients(airfoil, twist, r, c, ...
                lambda_r, V_wind, omega_rad, pitch_rad, data, rho, data.materials.air.viscosity);
            
            dT(i) = 0.5 * rho * V_rel^2 * c * Cn;
            dQ(i) = 0.5 * rho * V_rel^2 * c * Ct * r;
            dP(i) = dQ(i) * omega_rad;
        end
        
        T_total = B * trapz(r_stations, dT);
        Q_total = B * trapz(r_stations, dQ);
        P_total = B * trapz(r_stations, dP);
        
        CP_values(j) = P_total / (0.5 * rho * A * V_wind^3);
        CT_values(j) = T_total / (0.5 * rho * A * V_wind^2);
        P_values(j) = P_total;
        T_values(j) = T_total;
        
        if mod(j, 5) == 0 || j == n_points
            fprintf('  Progress: %.1f%% (pitch=%.1f°)\n', j/n_points*100, pitch_angle);
        end
    end
    
    [CP_max, max_idx] = max(CP_values);
    optimal_pitch = pitch_range(max_idx);
    
    optimal_conditions = struct();
    optimal_conditions.pitch_angle = optimal_pitch;
    optimal_conditions.lambda = lambda;
    optimal_conditions.omega_rad = omega_rad;
    optimal_conditions.omega_rpm = omega_rpm;
    optimal_conditions.CP = CP_max;
    optimal_conditions.CT = CT_values(max_idx);
    optimal_conditions.P = P_values(max_idx);
    optimal_conditions.T = T_values(max_idx);
    optimal_conditions.V_wind = V_wind;
    
    fprintf('\n=== OPTIMIZATION RESULTS ===\n');
    fprintf('Maximum CP: %.4f\n', CP_max);
    fprintf('Optimal pitch angle: %.1f degrees\n', optimal_pitch);
    fprintf('Optimal thrust coefficient: %.4f\n', optimal_conditions.CT);
    fprintf('Optimal power: %.1f kW\n', optimal_conditions.P/1000);
    fprintf('Optimal thrust: %.1f kN\n', optimal_conditions.T/1000);
    
    createPitchOptimizationPlot(pitch_range, CP_values, CT_values, optimal_pitch, CP_max, V_wind, lambda);
    fprintf('\nPitch optimization complete!\n');
end

function [a, a_prime, CL, CD, Cn, Ct, V_rel] = solveBEMSection(r, c, twist_rad, perf_data, lambda_r, V_wind, omega_rad, pitch_rad)
% Closed-form induction factors and sectional loads (no iteration)
% Inputs:
%   r, c           - local radius [m] and chord [m]
%   twist_rad      - local geometric twist [rad]
%   perf_data      - airfoil polars with .AoA, .CL, .CD
%   lambda_r       - local tip-speed ratio (λ * r / R)
%   V_wind         - freestream wind speed [m/s]
%   omega_rad      - rotor speed [rad/s]
%   pitch_rad      - blade pitch angle [rad]
% Outputs:
%   a, a_prime     - axial/tangential induction factors
%   CL, CD         - section lift/drag coefficients at α
%   Cn, Ct         - normal/tangential force coefficients
%   V_rel          - relative velocity magnitude at section [m/s]

    a = 1/3;
    a_prime = -0.5 + 0.5 * sqrt(1 + (4/(lambda_r^2)) * a * (1 - a));

    phi = atan((1 - a) / ((1 + a_prime) * lambda_r));
    alpha = phi - (twist_rad + pitch_rad);
    alpha_deg = rad2deg(alpha);

    CL = interp1(perf_data.AoA, perf_data.CL, alpha_deg, 'linear', 'extrap');
    CD = interp1(perf_data.AoA, perf_data.CD, alpha_deg, 'linear', 'extrap');

    s = sin(phi); c = cos(phi);
    Cn = CL * c + CD * s;
    Ct = CL * s - CD * c;

    V_rel = sqrt((V_wind*(1-a))^2 + (omega_rad*r*(1+a_prime))^2);
end

function [a, a_prime, CL, CD, Cn, Ct, V_rel] = solveCircleSection(r, c, twist_rad, lambda_r, V_wind, omega_rad, pitch_rad, rho, mu)
    a = 1/3;
    a_prime = -0.5 + 0.5 * sqrt(1 + (4/(lambda_r^2)) * a * (1 - a));
    phi = atan((1 - a) / ((1 + a_prime) * lambda_r));
    alpha = phi - (twist_rad + pitch_rad); %#ok<NASGU>
    V_rel = sqrt((V_wind*(1-a))^2 + (omega_rad*r*(1+a_prime))^2);
    Re = max(1, rho * V_rel * c / mu);
    CD = cylinderCDlocal(Re);
    CL = 0;
    s = sin(phi); cphi = cos(phi);
    Cn = CL * cphi + CD * s;
    Ct = CL * s - CD * cphi;
end

function C_D = cylinderCDlocal(Re)
    if Re < 2e5
        C_D = 11 * Re.^(-0.75) + 0.9 * (1.0 - exp(-1000./Re)) + 1.2 * (1.0 - exp(-(Re./4500).^0.7));
    elseif Re <= 5e5
        C_D = 10.^(0.32*tanh(44.4504 - 8 * log10(Re)) - 0.238793158);
    else
        C_D = 0.1 * log10(Re) - 0.2533429;
    end
end

function createPitchOptimizationPlot(pitch_range, CP_values, CT_values, optimal_pitch, CP_max, V_wind, lambda)
% Create visualization of CP and CT vs Pitch Angle
    
    figure('Position', [100, 100, 1000, 700]);
    
    yyaxis left;
    plot(pitch_range, CP_values, 'b-o', 'LineWidth', 2, 'MarkerSize', 4);
    ylabel('Coefficient of Power (C_P)');
    ylim_left = ylim;
    
    yyaxis right;
    plot(pitch_range, CT_values, 'r-x', 'LineWidth', 2, 'MarkerSize', 4);
    ylabel('Coefficient of Thrust (C_T)');
    ylim_right = ylim;
    
    % Set both y-axes to the same range
    ylim_combined = [min(ylim_left(1), ylim_right(1)), max(ylim_left(2), ylim_right(2))];
    yyaxis left;
    ylim(ylim_combined);
    yyaxis right;
    ylim(ylim_combined);
    
    hold on;
    plot(optimal_pitch, CP_max, 'go', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', sprintf('Max C_P = %.4f at \\theta = %.1f°', CP_max, optimal_pitch));
    hold off;
    
    xlabel('Pitch Angle (degrees)');
    title(sprintf('Wind Turbine Pitch Optimization (V = %.1f m/s, \\lambda = %.1f)', V_wind, lambda));
    legend('C_P', 'C_T', 'Optimal C_P', 'Location', 'best');
    grid on;
    
    saveas(gcf, 'Pitch_Optimization_Results.png');
    fprintf('Pitch optimization results visualization saved as Pitch_Optimization_Results.png\n');
end

function [a, a_prime, CL, CD, Cn, Ct, V_rel] = getSectionCoefficients(airfoil, twist_deg, r, c, lambda_r, V_wind, omega_rad, pitch_rad, data, rho, mu)
    airfoil_name = regexprep(airfoil, '\s+', '');
    airfoil_name = regexprep(airfoil_name, '[-–—]', '_');
    if strcmpi(airfoil_name, 'circle')
        twist_rad = deg2rad(twist_deg);
        [a, a_prime, CL, CD, Cn, Ct, V_rel] = solveCircleSection(r, c, twist_rad, ...
            lambda_r, V_wind, omega_rad, pitch_rad, rho, mu);
    else
        perf_data = data.airfoilPerformance.(airfoil_name);
        twist_rad = deg2rad(twist_deg);
        [a, a_prime, CL, CD, Cn, Ct, V_rel] = solveBEMSection(r, c, twist_rad, perf_data, ...
            lambda_r, V_wind, omega_rad, pitch_rad);
    end
end
