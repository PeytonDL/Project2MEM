function [CP, CT] = Deliverable1()
% DELIVERABLE1 Wind Turbine Analysis using ParameterExtraction data
% 
% This function calculates the coefficient of power (C_P) and coefficient 
% of thrust (C_T) for a wind turbine under specified operating conditions.
%
% Given Conditions:
%   - Wind velocity: 10 m/s
%   - Rotational velocity: 14 rpm
%   - Pitch angle: 0 degrees
%
% OUTPUTS:
%   CP - Coefficient of power
%   CT - Coefficient of thrust
%
% Usage: [CP, CT] = Deliverable1();

    % Clear workspace and close figures
    clc; close all;
    
    fprintf('=== Wind Turbine Analysis - Deliverable 1 ===\n\n');
    
    % Extract all parameter data
    fprintf('Extracting wind turbine parameters...\n');
    data = ParameterExtraction();
    
    % Given operating conditions
    V_wind = 10;        % Wind velocity [m/s]
    omega = 14;         % Rotational velocity [rpm]
    pitch_angle = 0;    % Pitch angle [degrees]
    
    fprintf('\nOperating Conditions:\n');
    fprintf('  Wind velocity: %.1f m/s\n', V_wind);
    fprintf('  Rotational velocity: %.1f rpm\n', omega);
    fprintf('  Pitch angle: %.1f degrees\n', pitch_angle);
    
    % Convert rotational velocity to rad/s
    omega_rad = omega * 2 * pi / 60;  % [rad/s]
    
    % Get turbine specifications
    R = data.turbine.performance.rotorRadius;  % Rotor radius [m]
    A = data.turbine.calculated.rotorArea;     % Rotor swept area [m²]
    rho = data.materials.air.density;          % Air density [kg/m³]
    
    fprintf('\nTurbine Specifications:\n');
    fprintf('  Rotor radius: %.1f m\n', R);
    fprintf('  Swept area: %.1f m²\n', A);
    fprintf('  Air density: %.2f kg/m³\n', rho);
    
    % Calculate tip speed ratio
    lambda = omega_rad * R / V_wind;
    fprintf('\nCalculated Parameters:\n');
    fprintf('  Tip speed ratio (λ): %.3f\n', lambda);
    
    % Perform blade element momentum analysis
    fprintf('\nPerforming blade element momentum analysis...\n');
    
    % Get blade profile data
    blade_profile = data.blade.profile;
    n_stations = height(blade_profile);
    
    fprintf('  Analyzing %d blade stations...\n', n_stations);
    
    % Initialize arrays for integration
    dCP = zeros(n_stations, 1);
    dCT = zeros(n_stations, 1);
    r_stations = blade_profile.DistanceFromCenterOfRotation;
    
    % Perform BEM analysis for each station
    for i = 1:n_stations
        r = r_stations(i);  % Local radius [m]
        c = blade_profile.ChordLength(i) / 1000;  % Chord length [m]
        twist = blade_profile.BladeTwist(i);   % Twist angle [degrees]
        airfoil = blade_profile.Airfoil{i};   % Airfoil type
        
        % Local tip speed ratio
        lambda_r = lambda * r / R;
        
        % Calculate local coefficients using BEM theory
        [dCP(i), dCT(i)] = calculateLocalCoefficients(r, c, twist, airfoil, ...
                                                     lambda_r, V_wind, omega_rad, ...
                                                     data.airfoilPerformance, rho, ...
                                                     data.materials.air.viscosity);
    end
    
    % Integrate to get total coefficients
    % Note: dCP and dCT are already per unit radius, so we integrate directly
    CP = trapz(r_stations, dCP);
    CT = trapz(r_stations, dCT);
    
    % Display results
    fprintf('\n=== RESULTS ===\n');
    fprintf('Coefficient of Power (C_P): %.4f\n', CP);
    fprintf('Coefficient of Thrust (C_T): %.4f\n', CT);
    
    % Calculate power and thrust
    P = 0.5 * rho * A * V_wind^3 * CP;  % Power [W]
    T = 0.5 * rho * A * V_wind^2 * CT;  % Thrust [N]
    
    fprintf('\nCalculated Values:\n');
    fprintf('Power: %.1f kW\n', P/1000);
    fprintf('Thrust: %.1f kN\n', T/1000);
    
    % Create visualization
    createVisualization(r_stations, dCP, dCT, CP, CT, lambda);
    
    fprintf('\nAnalysis complete!\n');
end

function [dCP, dCT] = calculateLocalCoefficients(r, c, twist, airfoil, lambda_r, V_wind, omega_rad, airfoilData, rho, mu)
% Calculate local power and thrust coefficients using BEM theory
    
    % Get airfoil performance data
    airfoil_name = strrep(airfoil, '-', '_');
    if isfield(airfoilData, airfoil_name)
        perf_data = airfoilData.(airfoil_name);
    else
        % Use default airfoil if not found
        perf_data = airfoilData.DU96_W_180;
    end
    
    % Calculate local flow angle
    phi = atan(1/lambda_r);  % Flow angle [rad]
    
    % Calculate angle of attack
    alpha = phi - deg2rad(twist);  % AoA [rad]
    alpha_deg = rad2deg(alpha);
    
    % Calculate relative velocity
    V_rel = V_wind * sqrt(1 + lambda_r^2);
    
    % Calculate Reynolds number
    Re = rho * V_rel * c / mu;
    
    % Interpolate airfoil coefficients (assuming data is at standard Re)
    CL = interp1(perf_data.AoA, perf_data.CL, alpha_deg, 'linear', 'extrap');
    CD = interp1(perf_data.AoA, perf_data.CD, alpha_deg, 'linear', 'extrap');
    
    % Calculate local solidity
    sigma = c / (2 * pi * r);
    
    % Calculate local coefficients using BEM theory
    % Power coefficient contribution (corrected BEM equation)
    dCP = 8 * lambda_r^2 * sigma * CL * sin(phi)^2 * cos(phi);
    
    % Thrust coefficient contribution (corrected BEM equation)
    dCT = 4 * sigma * (CL * cos(phi) + CD * sin(phi)) * sin(phi)^2;
end

function createVisualization(r_stations, dCP, dCT, CP, CT, lambda)
% Create visualization of the analysis results
    
    figure('Position', [100, 100, 1200, 800]);
    
    % Subplot 1: Local coefficient distributions
    subplot(2, 2, 1);
    plot(r_stations, dCP, 'b-', 'LineWidth', 2);
    xlabel('Radius [m]');
    ylabel('Local C_P');
    title('Local Power Coefficient Distribution');
    grid on;
    
    subplot(2, 2, 2);
    plot(r_stations, dCT, 'r-', 'LineWidth', 2);
    xlabel('Radius [m]');
    ylabel('Local C_T');
    title('Local Thrust Coefficient Distribution');
    grid on;
    
    % Subplot 3: Combined plot
    subplot(2, 2, 3);
    yyaxis left;
    plot(r_stations, dCP, 'b-', 'LineWidth', 2);
    ylabel('Local C_P');
    yyaxis right;
    plot(r_stations, dCT, 'r-', 'LineWidth', 2);
    ylabel('Local C_T');
    xlabel('Radius [m]');
    title('Local Coefficients vs Radius');
    legend('C_P', 'C_T', 'Location', 'best');
    grid on;
    
    % Subplot 4: Results summary
    subplot(2, 2, 4);
    text(0.1, 0.8, sprintf('Tip Speed Ratio: %.3f', lambda), 'FontSize', 14, 'FontWeight', 'bold');
    text(0.1, 0.6, sprintf('C_P = %.4f', CP), 'FontSize', 14, 'FontWeight', 'bold', 'Color', 'blue');
    text(0.1, 0.4, sprintf('C_T = %.4f', CT), 'FontSize', 14, 'FontWeight', 'bold', 'Color', 'red');
    text(0.1, 0.2, 'Wind Velocity: 10 m/s', 'FontSize', 12);
    text(0.1, 0.1, 'Rotational Speed: 14 rpm', 'FontSize', 12);
    axis off;
    title('Analysis Results Summary');
    
    sgtitle('Wind Turbine Analysis - Deliverable 1', 'FontSize', 16, 'FontWeight', 'bold');
    
    % Save the figure
    saveas(gcf, 'Deliverable1_Results.png');
    fprintf('Results visualization saved as Deliverable1_Results.png\n');
end
