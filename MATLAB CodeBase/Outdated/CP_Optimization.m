function [CP_max, optimal_conditions] = CP_Optimization(V_wind, pitch_angle, lambda_min, lambda_max, lambda_step)
% CP_OPTIMIZATION Find maximum power coefficient for given conditions
% 
% This function uses the BEM analysis from Deliverable1 to find the maximum
% power coefficient (CP) by varying the tip speed ratio while keeping wind
% velocity and pitch angle constant.
%
% INPUTS:
%   V_wind - Wind velocity [m/s]
%   pitch_angle - Pitch angle [degrees]
%   lambda_min - Minimum tip speed ratio
%   lambda_max - Maximum tip speed ratio
%   lambda_step - Tip speed ratio step size
%
% OUTPUTS:
%   CP_max - Maximum power coefficient achieved
%   optimal_conditions - Structure containing optimal operating conditions
%
% Usage: [CP_max, optimal_conditions] = CP_Optimization(10, 0, 3, 12, 0.5);

    % Clear workspace and close figures
    clc; close all;
    
    fprintf('=== Wind Turbine CP Optimization ===\n\n');
    
    % Display optimization parameters
    fprintf('Optimizing CP for:\n');
    fprintf('  Wind velocity: %.1f m/s\n', V_wind);
    fprintf('  Pitch angle: %.1f degrees\n', pitch_angle);
    fprintf('  Tip speed ratio range: %.1f to %.1f\n', lambda_min, lambda_max);
    fprintf('  Step size: %.1f\n\n', lambda_step);
    
    % Extract parameter data
    fprintf('Extracting wind turbine parameters...\n');
    data = ParameterExtraction();
    
    % Get turbine specifications
    R = data.turbine.performance.rotorRadius;  % Rotor radius [m]
    A = data.turbine.calculated.rotorArea;     % Rotor swept area [m²]
    rho = data.materials.air.density;          % Air density [kg/m³]
    
    % Initialize optimization arrays
    lambda_range = lambda_min:lambda_step:lambda_max;
    n_points = length(lambda_range);
    CP_values = zeros(n_points, 1);
    CT_values = zeros(n_points, 1);
    P_values = zeros(n_points, 1);
    T_values = zeros(n_points, 1);
    
    fprintf('Performing CP optimization...\n');
    fprintf('Progress: ');
    
    % Loop through tip speed ratios
    for i = 1:n_points
        lambda = lambda_range(i);
        
        % Calculate rotational velocity from tip speed ratio
        omega_rad = lambda * V_wind / R;
        omega_rpm = omega_rad * 60 / (2 * pi);
        
        % Perform BEM analysis for this tip speed ratio
        [CP, CT, P, T] = performBEMAnalysis(V_wind, omega_rpm, pitch_angle, data);
        
        % Store results
        CP_values(i) = CP;
        CT_values(i) = CT;
        P_values(i) = P;
        T_values(i) = T;
        
        % Progress indicator
        if mod(i, max(1, floor(n_points/10))) == 0
            fprintf('%.0f%% ', (i/n_points)*100);
        end
    end
    
    fprintf('100%%\n\n');
    
    % Find maximum CP
    [CP_max, max_idx] = max(CP_values);
    optimal_lambda = lambda_range(max_idx);
    optimal_omega = optimal_lambda * V_wind / R;
    optimal_omega_rpm = optimal_omega * 60 / (2 * pi);
    
    % Create results structure
    optimal_conditions = struct();
    optimal_conditions.lambda = optimal_lambda;
    optimal_conditions.omega_rpm = optimal_omega_rpm;
    optimal_conditions.omega_rads = optimal_omega;
    optimal_conditions.CP = CP_max;
    optimal_conditions.CT = CT_values(max_idx);
    optimal_conditions.P = P_values(max_idx);
    optimal_conditions.T = T_values(max_idx);
    optimal_conditions.V_wind = V_wind;
    optimal_conditions.pitch_angle = pitch_angle;
    
    % Display results
    fprintf('\n=== OPTIMIZATION RESULTS ===\n');
    fprintf('Maximum CP: %.4f\n', CP_max);
    fprintf('Optimal tip speed ratio: %.3f\n', optimal_lambda);
    fprintf('Optimal rotational speed: %.1f rpm\n', optimal_omega_rpm);
    fprintf('Optimal thrust coefficient: %.4f\n', optimal_conditions.CT);
    fprintf('Optimal power: %.1f kW\n', optimal_conditions.P/1000);
    fprintf('Optimal thrust: %.1f kN\n', optimal_conditions.T/1000);
    
    % Create visualization
    createOptimizationPlot(lambda_range, CP_values, CT_values, optimal_lambda, CP_max);
    
    fprintf('\nOptimization complete!\n');
end

function [CP, CT, P, T] = performBEMAnalysis(V_wind, omega_rpm, pitch_angle, data)
% Perform BEM analysis for given conditions
% This function extracts the BEM analysis from Deliverable1

    % Convert rotational velocity to rad/s
    omega_rad = omega_rpm * 2 * pi / 60;
    
    % Get turbine specifications
    R = data.turbine.performance.rotorRadius;
    A = data.turbine.calculated.rotorArea;
    rho = data.materials.air.density;
    
    % Get blade profile data
    blade_profile = data.blade.profile;
    n_stations = height(blade_profile);
    r_stations = blade_profile.DistanceFromCenterOfRotation / 1000;  % Convert mm to m
    
    % Initialize arrays
    dT = zeros(n_stations, 1);
    dQ = zeros(n_stations, 1);
    dP = zeros(n_stations, 1);
    B = 3;  % Number of blades
    
    % Perform BEM analysis for each station
    for i = 1:n_stations
        r = r_stations(i);
        c = blade_profile.ChordLength(i) / 1000;  % Chord length [m]
        twist = blade_profile.BladeTwist(i);   % Twist angle [degrees]
        airfoil = blade_profile.Airfoil{i};   % Airfoil type
        
        % Local tip speed ratio
        lambda_r = omega_rad * r / V_wind;
        
        % Perform BEM analysis for this station
        [a, a_prime, CL, CD, Cn, Ct] = solveBEMIteration(r, c, twist, airfoil, ...
                                                         lambda_r, V_wind, omega_rad, ...
                                                         data.airfoilPerformance, rho, ...
                                                         data.materials.air.viscosity, pitch_angle);
        
        % Calculate flow angle and relative velocity
        phi = atan((1-a)*V_wind / ((1+a_prime)*omega_rad*r));
        V_rel = sqrt((V_wind*(1-a))^2 + (omega_rad*r*(1+a_prime))^2);
        
        % Calculate differential quantities
        dT(i) = 0.5 * rho * V_rel^2 * c * Cn;
        dQ(i) = 0.5 * rho * V_rel^2 * c * Ct * r;
        dP(i) = dQ(i) * omega_rad;
    end
    
    % Calculate total quantities
    T = B * trapz(r_stations, dT);
    Q = B * trapz(r_stations, dQ);
    P = B * trapz(r_stations, dP);
    
    % Calculate coefficients
    CP = P / (0.5 * rho * A * V_wind^3);
    CT = T / (0.5 * rho * A * V_wind^2);
end

function [a, a_prime, CL, CD, Cn, Ct] = solveBEMIteration(r, c, twist, airfoil, lambda_r, V_wind, omega_rad, airfoilData, rho, mu, pitch_angle)
% Robust BEM iteration solver (copied from Deliverable1)
    
    % Get airfoil performance data
    airfoil_name = strrep(airfoil, '-', '_');
    if isfield(airfoilData, airfoil_name)
        perf_data = airfoilData.(airfoil_name);
    else
        perf_data = airfoilData.DU96_W_180;
    end
    
    % Initialize induction factors
    a = 0.1;      % Start with small positive value
    a_prime = 0.01; % Start with small positive value
    
    % Convergence parameters
    max_iter = 50;
    tolerance = 1e-4;
    relaxation = 0.2;  % Conservative relaxation factor
    
    % Iterative BEM analysis
    for iter = 1:max_iter
        a_old = a;
        a_prime_old = a_prime;
        
        % Calculate flow angle
        phi = atan((1-a)*V_wind / ((1+a_prime)*omega_rad*r));
        
        % Calculate angle of attack
        alpha = phi - deg2rad(twist) - deg2rad(pitch_angle);
        alpha_deg = rad2deg(alpha);
        
        % Interpolate airfoil coefficients
        CL = interp1(perf_data.AoA, perf_data.CL, alpha_deg, 'linear', 'extrap');
        CD = interp1(perf_data.AoA, perf_data.CD, alpha_deg, 'linear', 'extrap');
        
        % Calculate force coefficients
        Cn = CL * cos(phi) + CD * sin(phi);
        Ct = CL * sin(phi) - CD * cos(phi);
        
        % Calculate local solidity
        sigma = c / (2 * pi * r);
        
        % Calculate new induction factors using momentum theory
        % Axial induction factor
        if Cn > 0 && sigma > 0
            a_new = 1 / (1 + 4*sin(phi)^2 / (sigma * Cn));
        else
            a_new = 0;
        end
        
        % Tangential induction factor
        if Ct > 0 && sigma > 0
            a_prime_new = 1 / (4*sin(phi)*cos(phi) / (sigma * Ct) - 1);
        else
            a_prime_new = 0;
        end
        
        % Apply bounds to prevent unrealistic values
        a_new = max(0, min(a_new, 0.5));
        a_prime_new = max(0, min(a_prime_new, 1.0));
        
        % Apply relaxation for stability
        a = a + relaxation * (a_new - a);
        a_prime = a_prime + relaxation * (a_prime_new - a_prime);
        
        % Convergence check
        if abs(a - a_old) < tolerance && abs(a_prime - a_prime_old) < tolerance
            break;
        end
        
        % Additional stability check - prevent oscillations
        if iter > 10 && (abs(a - a_old) > 0.1 || abs(a_prime - a_prime_old) > 0.1)
            % Reduce relaxation factor if oscillating
            relaxation = relaxation * 0.9;
        end
    end
    
    % Final calculation of coefficients
    phi = atan((1-a)*V_wind / ((1+a_prime)*omega_rad*r));
    alpha = phi - deg2rad(twist) - deg2rad(pitch_angle);
    alpha_deg = rad2deg(alpha);
    
    CL = interp1(perf_data.AoA, perf_data.CL, alpha_deg, 'linear', 'extrap');
    CD = interp1(perf_data.AoA, perf_data.CD, alpha_deg, 'linear', 'extrap');
    
    Cn = CL * cos(phi) + CD * sin(phi);
    Ct = CL * sin(phi) - CD * cos(phi);
    
    % Check for convergence issues
    if iter == max_iter
        fprintf('Warning: BEM did not converge at r=%.1f: a=%.3f, a''=%.3f\n', r, a, a_prime);
    end
end

function createOptimizationPlot(lambda_range, CP_values, CT_values, optimal_lambda, CP_max)
% Create optimization visualization
    
    figure('Position', [100, 100, 1200, 600]);
    
    % Subplot 1: CP vs Tip Speed Ratio
    subplot(1, 2, 1);
    plot(lambda_range, CP_values, 'b-', 'LineWidth', 2);
    hold on;
    plot(optimal_lambda, CP_max, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    xlabel('Tip Speed Ratio (λ)');
    ylabel('Power Coefficient (C_P)');
    title('Power Coefficient vs Tip Speed Ratio');
    grid on;
    legend('C_P', 'Optimal Point', 'Location', 'best');
    
    % Subplot 2: CT vs Tip Speed Ratio
    subplot(1, 2, 2);
    plot(lambda_range, CT_values, 'r-', 'LineWidth', 2);
    xlabel('Tip Speed Ratio (λ)');
    ylabel('Thrust Coefficient (C_T)');
    title('Thrust Coefficient vs Tip Speed Ratio');
    grid on;
    
    sgtitle('Wind Turbine CP Optimization Results', 'FontSize', 16, 'FontWeight', 'bold');
    
    % Save the figure
    saveas(gcf, 'CP_Optimization_Results.png');
    fprintf('Optimization results saved as CP_Optimization_Results.png\n');
end
