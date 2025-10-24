function [CP, CT] = Deliverable1()
% DELIVERABLE1 Wind Turbine BEM Analysis
% Calculates power and thrust coefficients for specified operating conditions
% Usage: [CP, CT] = Deliverable1();

    clc; close all;
    fprintf('=== Wind Turbine Analysis - Deliverable 1 ===\n\n');
    
    fprintf('Extracting wind turbine parameters...\n');
    data = ParameterExtraction();
    
    % Use predefined defaults if available
    if isfield(data, 'deliverables') && isfield(data.deliverables, 'part1')
        V_wind = data.deliverables.part1.V_wind;
        omega = data.deliverables.part1.omega_rpm;
        pitch_angle = data.deliverables.part1.pitch_deg;
    else
        V_wind = 10;
        omega = 14;
        pitch_angle = 0;
    end
    
    fprintf('\nOperating Conditions:\n');
    fprintf('  Wind velocity: %.1f m/s\n', V_wind);
    fprintf('  Rotational velocity: %.1f rpm\n', omega);
    fprintf('  Pitch angle: %.1f degrees\n', pitch_angle);
    
    omega_rad = omega * 2 * pi / 60;
    R = data.turbine.performance.rotorRadius;
    A = data.turbine.calculated.rotorArea;
    rho = data.materials.air.density;
    
    fprintf('\nTurbine Specifications:\n');
    fprintf('  Rotor radius: %.1f m\n', R);
    fprintf('  Swept area: %.1f m²\n', A);
    fprintf('  Air density: %.2f kg/m³\n', rho);
    
    lambda = omega_rad * R / V_wind;
    fprintf('\nCalculated Parameters:\n');
    fprintf('  Tip speed ratio (λ): %.3f\n', lambda);
    
    fprintf('\nPerforming blade element momentum analysis...\n');
    blade_profile = data.blade.profile;
    n_stations = height(blade_profile);
    fprintf('  Analyzing %d blade stations...\n', n_stations);
    
    dCP = zeros(n_stations, 1);
    dCT = zeros(n_stations, 1);
    r_stations = blade_profile.DistanceFromCenterOfRotation / 1000;
    dT = zeros(n_stations, 1);
    dQ = zeros(n_stations, 1);
    dP = zeros(n_stations, 1);
    B = 3;
    
    % Constant inputs precomputed once
    pitch_rad = deg2rad(pitch_angle);
    
    % Iterate over blade stations to accumulate sectional loads and power
    for i = 1:n_stations
        r = r_stations(i);
        c = blade_profile.ChordLength(i) / 1000; % converts to meters
        twist = blade_profile.BladeTwist(i); % degrees (geometric twist)
        airfoil = blade_profile.Airfoil{i};
        
        lambda_r = lambda * r / R; % local tip-speed ratio at radius r
        
        % Section aerodynamics (dispatches circle vs airfoil polars)
        [a, a_prime, CL, CD, Cn, Ct, V_rel] = getSectionCoefficients(airfoil, twist, r, c, ...
            lambda_r, V_wind, omega_rad, pitch_rad, data, rho, data.materials.air.viscosity);
        
        
        
        % Elemental thrust, torque, and power contributions
        dT(i) = 0.5 * rho * V_rel^2 * c * Cn; % thrust per unit length (66)
        dQ(i) = 0.5 * rho * V_rel^2 * c * Ct * r; % torque per unit length (68)
        dP(i) = dQ(i) * omega_rad; % power per unit length (torque * rotational speed) (69)
        
        dCP(i) = dP(i) / (0.5 * rho * V_wind^3); % power coefficient per unit length
        dCT(i) = dT(i) / (0.5 * rho * V_wind^2); % thrust coefficient per unit length
    end
    
    T = B * trapz(r_stations, dT); % total thrust
    Q = B * trapz(r_stations, dQ); % total torque
    P = B * trapz(r_stations, dP); % total power
    
    CP = P / (0.5 * rho * A * V_wind^3);
    CT = T / (0.5 * rho * A * V_wind^2);
    
    fprintf('\n=== RESULTS ===\n');
    fprintf('Coefficient of Power (C_P): %.4f\n', CP);
    fprintf('Coefficient of Thrust (C_T): %.4f\n', CT);
    fprintf('\nCalculated Values:\n');
    fprintf('Power: %.1f kW\n', P/1000);
    fprintf('Thrust: %.1f kN\n', T/1000);
    fprintf('Torque: %.1f kN·m\n', Q/1000);
    
    createVisualization(r_stations, dCP, dCT, CP, CT, lambda, V_wind, omega);
    fprintf('\nAnalysis complete!\n');
end

function [a, a_prime, CL, CD, Cn, Ct, V_rel] = solveBEMSection(r, c, twist_rad, perf_data, lambda_r, V_wind, omega_rad, pitch_rad)
% Closed-form induction factors and sectional loads (no iteration)
% Inputs:
%   r           - radial position [m]
%   c           - local chord [m]
%   twist_rad   - local geometric twist [rad]
%   perf_data   - airfoil polar struct with fields .AoA, .CL, .CD
%   lambda_r    - local tip-speed ratio (λ * r / R)
%   V_wind      - freestream wind speed [m/s]
%   omega_rad   - rotor speed [rad/s]
%   pitch_rad   - blade pitch angle [rad]
% Outputs:
%   a, a_prime  - axial and tangential induction factors
%   CL, CD      - section lift and drag coefficients at computed α
%   Cn, Ct      - normal and tangential force coefficients
%   V_rel       - relative velocity magnitude at section [m/s]

    % Assumed momentum-limit induction with closed-form a_prime from λ_r
    a = 1/3;
    a_prime = -0.5 + 0.5 * sqrt(1 + (4/(lambda_r^2)) * a * (1 - a));

    % Inflow angle and angle of attack
    phi = atan((1 - a) / ((1 + a_prime) * lambda_r));
    alpha = phi - (twist_rad + pitch_rad); % angle of attack
    alpha_deg = rad2deg(alpha); % angle of attack in degrees

    % Interpolate section polars (for circle, CD is flat and CL=0 by construction)
    CL = interp1(perf_data.AoA, perf_data.CL, alpha_deg, 'linear', 'extrap');
    CD = interp1(perf_data.AoA, perf_data.CD, alpha_deg, 'linear', 'extrap');

    % Resolve to normal and tangential force coefficients
    s = sin(phi); c = cos(phi);
    Cn = CL * c + CD * s;
    Ct = CL * s - CD * c;

    % Relative velocity at the section
    V_rel = sqrt((V_wind*(1-a))^2 + (omega_rad*r*(1+a_prime))^2);
end

function [a, a_prime, CL, CD, Cn, Ct, V_rel] = solveCircleSection(r, c, twist_rad, lambda_r, V_wind, omega_rad, pitch_rad, rho, mu)
% Circle (cylinder) section: CL=0, CD from Re via empirical fit. Uses same BEM kinematics.
    a = 1/3;
    a_prime = -0.5 + 0.5 * sqrt(1 + (4/(lambda_r^2)) * a * (1 - a));
    
    phi = atan((1 - a) / ((1 + a_prime) * lambda_r));
    alpha = phi - (twist_rad + pitch_rad); %#ok<NASGU>
    
    % Reynolds number at this station
    V_rel = sqrt((V_wind*(1-a))^2 + (omega_rad*r*(1+a_prime))^2);
    Re = max(1, rho * V_rel * c / mu);
    CD = cylinderCDlocal(Re);
    CL = 0;
    
    s = sin(phi); cphi = cos(phi);
    Cn = CL * cphi + CD * s;
    Ct = CL * s - CD * cphi;
end

function C_D = cylinderCDlocal(Re)
% Drag coefficient for a smooth circular cylinder in cross-flow (empirical fit)
    if Re < 2e5
        C_D = 11 * Re.^(-0.75) + 0.9 * (1.0 - exp(-1000./Re)) + 1.2 * (1.0 - exp(-(Re./4500).^0.7));
    elseif Re <= 5e5
        C_D = 10.^(0.32*tanh(44.4504 - 8 * log10(Re)) - 0.238793158);
    else
        C_D = 0.1 * log10(Re) - 0.2533429;
    end
end

function [a, a_prime, CL, CD, Cn, Ct, V_rel] = getSectionCoefficients(airfoil, twist_deg, r, c, lambda_r, V_wind, omega_rad, pitch_rad, data, rho, mu)
% Normalize airfoil name and dispatch to appropriate section solver
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

function createVisualization(r_stations, dCP, dCT, CP, CT, lambda, V_wind, omega)
% Create visualization of the analysis results
    
    figure('Position', [100, 100, 1200, 800]);
    
    if ~isempty(dCP) && ~isempty(dCT)
        subplot(2, 2, 1);
        area(r_stations, dCP, 'FaceColor', 'blue', 'FaceAlpha', 0.3, 'EdgeColor', 'blue', 'LineWidth', 2);
        xlabel('Radius [m]');
        ylabel('Local C_P');
        title('Local Power Coefficient Distribution');
        grid on;
        
        subplot(2, 2, 2);
        area(r_stations, dCT, 'FaceColor', 'red', 'FaceAlpha', 0.3, 'EdgeColor', 'red', 'LineWidth', 2);
        xlabel('Radius [m]');
        ylabel('Local C_T');
        title('Local Thrust Coefficient Distribution');
        grid on;
        
        subplot(2, 2, 3);
        yyaxis left;
        plot(r_stations, dCP, 'b-', 'LineWidth', 2);
        ylabel('Local C_P');
        ylim_left = ylim;
        yyaxis right;
        plot(r_stations, dCT, 'r-', 'LineWidth', 2);
        ylabel('Local C_T');
        ylim_right = ylim;
        
        % Set both y-axes to the same range
        ylim_combined = [min(ylim_left(1), ylim_right(1)), max(ylim_left(2), ylim_right(2))];
        yyaxis left;
        ylim(ylim_combined);
        yyaxis right;
        ylim(ylim_combined);
        
        xlabel('Radius [m]');
        title('Local Coefficients vs Radius');
        legend('C_P', 'C_T', 'Location', 'best');
        grid on;
    else
        subplot(2, 2, 1);
        a_range = 0:0.01:0.5;
        CP_theory = 4*a_range.*(1-a_range).^2;
        plot(a_range, CP_theory, 'b-', 'LineWidth', 2);
        xlabel('Axial Induction Factor (a)');
        ylabel('Power Coefficient (C_P)');
        title('Theoretical C_P vs Axial Induction Factor');
        grid on;
        
        subplot(2, 2, 2);
        CT_theory = 4*a_range.*(1-a_range);
        plot(a_range, CT_theory, 'r-', 'LineWidth', 2);
        xlabel('Axial Induction Factor (a)');
        ylabel('Thrust Coefficient (C_T)');
        title('Theoretical C_T vs Axial Induction Factor');
        grid on;
        
        subplot(2, 2, 3);
        yyaxis left;
        plot(a_range, CP_theory, 'b-', 'LineWidth', 2);
        ylabel('C_P');
        yyaxis right;
        plot(a_range, CT_theory, 'r-', 'LineWidth', 2);
        ylabel('C_T');
        xlabel('Axial Induction Factor (a)');
        title('Theoretical Coefficients vs Axial Induction Factor');
        legend('C_P', 'C_T', 'Location', 'best');
        grid on;
    end
    
    subplot(2, 2, 4);
    text(0.1, 0.8, sprintf('Tip Speed Ratio: %.3f', lambda), 'FontSize', 14, 'FontWeight', 'bold');
    text(0.1, 0.6, sprintf('C_P = %.4f', CP), 'FontSize', 14, 'FontWeight', 'bold', 'Color', 'blue');
    text(0.1, 0.4, sprintf('C_T = %.4f', CT), 'FontSize', 14, 'FontWeight', 'bold', 'Color', 'red');
    text(0.1, 0.2, sprintf('Wind Velocity: %.1f m/s', V_wind), 'FontSize', 12);
    text(0.1, 0.1, sprintf('Rotational Speed: %.1f rpm', omega), 'FontSize', 12);
    axis off;
    title('Analysis Results Summary');
    
    sgtitle('Wind Turbine Analysis - Deliverable 1', 'FontSize', 16, 'FontWeight', 'bold');
    saveas(gcf, 'Deliverable1_Results.png');
    fprintf('Results visualization saved as Deliverable1_Results.png\n');
end