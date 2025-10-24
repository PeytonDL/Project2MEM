function result = Deliverable5()
% DELIVERABLE5 Tower Deflection and Stress Analysis
% Computes tower deflection and static/fatigue overloads considering:
% - Distributed wind force along tower (atmospheric boundary layer)
% - Thrust force from turbine (calculated via BEM analysis)
% - Cyclic loading for fatigue analysis
% - Mohrs circle and Goodman diagram analysis
%
% Usage: result = Deliverable5();
% Returns struct with deflection, stress, and fatigue analysis results

    clc; close all;
    fprintf('=== Deliverable 5: Tower Deflection and Stress Analysis ===\n\n');

    % Extract parameters
    data = ParameterExtraction();
    
    % Tower geometry and material properties
    tower_specs = data.tower.specs;
    E = data.materials.steel.youngsModulus;           % Pa
    S_ut = data.materials.steel.tensileStrength;     % Pa
    S_y = data.materials.steel.yieldStrength;        % Pa
    rho_air = data.materials.air.density;           % kg/m³
    
    % Tower dimensions (convert from mm to m)
    tower_height = tower_specs.Height_mm_(end) / 1000;  % m
    hub_height = data.turbine.performance.hubHeight;    % m
    
    % Wind conditions
    V_wind = data.deliverables.part4.V_wind;  % m/s (rated wind speed)
    epsilon = 1/7;  % Power law exponent for atmospheric boundary layer
    
    fprintf('Tower height: %.1f m, Hub height: %.1f m\n', tower_height, hub_height);
    fprintf('Wind speed: %.1f m/s, E = %.0f GPa\n', V_wind, E/1e9);
    
    % Calculate thrust force from BEM analysis
    fprintf('\nCalculating rotor thrust force...\n');
    thrust_force = calculateRotorThrust(data, V_wind);
    fprintf('Total thrust force: %.1f kN\n', thrust_force/1000);
    
    % Define loading scenarios
    load_cases = defineLoadCases(V_wind, thrust_force);
    
    % Calculate tower section properties
    fprintf('\nComputing tower section properties...\n');
    section_props = computeTowerSectionProperties(tower_specs);
    
    % Solve deflection for both load cases
    fprintf('Solving tower deflection...\n');
    deflection_results = solveTowerDeflection(section_props, load_cases, E, data);
    
    % Calculate stresses at tower base
    fprintf('Computing stress analysis at tower base...\n');
    stress_results = computeStressAnalysis(section_props, load_cases, deflection_results, E, data);
    
    % Create visualizations
    fprintf('Creating visualizations...\n');
    createTowerDeflectionPlot(deflection_results, section_props);
    createMohrCirclePlots(stress_results);
    createGoodmanDiagram(stress_results, S_ut, S_y, data);
    
    % Compile results
    result = struct();
    result.deflection = deflection_results;
    result.stress = stress_results;
    result.thrust_force = thrust_force;
    result.tower_height = tower_height;
    result.hub_height = hub_height;
    result.material_properties = struct('E', E, 'S_ut', S_ut, 'S_y', S_y);
    
    fprintf('\n=== ANALYSIS COMPLETE ===\n');
    fprintf('Maximum tip deflection: %.3f m\n', max(abs(deflection_results.deflection)));
    fprintf('Maximum stress at base: %.1f MPa\n', max(stress_results.max_stress)/1e6);
    fprintf('Fatigue safety factor: %.2f\n', stress_results.fatigue_safety_factor);
end

function thrust_force = calculateRotorThrust(data, V_wind)
% Calculate rotor thrust force using BEM analysis (similar to Deliverable4)
    
    % Turbine parameters
    R = data.turbine.performance.rotorRadius;
    A = data.turbine.calculated.rotorArea;
    rho = data.materials.air.density;
    B = data.turbine.characteristics.blades;
    
    % Use rated conditions for thrust calculation
    lambda = data.deliverables.part5.lambda;  % Tip speed ratio at rated conditions
    pitch_deg = data.deliverables.part5.pitch_deg;  % Pitch angle for thrust calculation
    
    % Calculate thrust using BEM analysis
    blade_profile = data.blade.profile;
    r_stations = blade_profile.DistanceFromCenterOfRotation / 1000; % m
    chord_m = blade_profile.ChordLength / 1000; % m
    twist_deg_vec = blade_profile.BladeTwist;   % deg
    airfoil_raw = blade_profile.Airfoil;        % labels
    
    % Exclude hub region
    hub_radius = data.turbine.performance.hubRadius;
    use_idx = r_stations >= hub_radius;
    r_stations = r_stations(use_idx);
    chord_m = chord_m(use_idx);
    twist_deg_vec = twist_deg_vec(use_idx);
    airfoil_raw = airfoil_raw(use_idx);
    n_stations = numel(r_stations);
    
    % Calculate thrust
    omega_rad = lambda * V_wind / R;
    pitch_rad = deg2rad(pitch_deg);
    
    dT = zeros(n_stations, 1);
    for i = 1:n_stations
        r = r_stations(i);
        c = chord_m(i);
        twist_deg = twist_deg_vec(i);
        airfoil = airfoil_raw{i};
        lambda_r = lambda * r / R;
        
        [a, a_prime, CL, CD, Cn, Ct, V_rel] = getSectionCoefficients(airfoil, twist_deg, r, c, ...
            lambda_r, V_wind, omega_rad, pitch_rad, data, rho, data.materials.air.viscosity, B, R);
        
        dT(i) = 0.5 * rho * V_rel^2 * c * Cn;
    end
    
    thrust_force = B * trapz(r_stations, dT);
end

function load_cases = defineLoadCases(V_wind, thrust_force)
% Define loading scenarios for fatigue analysis
    
    % Load Case 1: Wind from 315° at magnitude 1.0 (maximum)
    load_cases.case1 = struct();
    load_cases.case1.wind_speed = V_wind;
    load_cases.case1.wind_direction = 315; % degrees
    load_cases.case1.thrust_force = thrust_force;
    load_cases.case1.description = 'Maximum loading - wind from 315°';
    
    % Load Case 2: Wind from 157° at magnitude 0.5 (minimum)
    load_cases.case2 = struct();
    load_cases.case2.wind_speed = V_wind * 0.5;
    load_cases.case2.wind_direction = 157; % degrees
    load_cases.case2.thrust_force = thrust_force * 0.5; % Assume thrust scales with wind speed
    load_cases.case2.description = 'Minimum loading - wind from 157°';
    
    fprintf('Load Case 1: V=%.1f m/s, θ=%.0f°, T=%.1f kN\n', ...
        load_cases.case1.wind_speed, load_cases.case1.wind_direction, load_cases.case1.thrust_force/1000);
    fprintf('Load Case 2: V=%.1f m/s, θ=%.0f°, T=%.1f kN\n', ...
        load_cases.case2.wind_speed, load_cases.case2.wind_direction, load_cases.case2.thrust_force/1000);
end

function section_props = computeTowerSectionProperties(tower_specs)
% Calculate section properties for each tower segment
    
    n_sections = height(tower_specs);
    section_props = struct();
    
    % Initialize arrays
    section_props.height = tower_specs.Height_mm_ / 1000;  % m
    section_props.OD = tower_specs.OD_mm_ / 1000;          % m
    section_props.wall_thickness = tower_specs{:,3} / 1000;  % m (3rd column is wall thickness)
    section_props.ID = section_props.OD - 2 * section_props.wall_thickness;  % m
    
    % Calculate section properties
    section_props.area = pi/4 * (section_props.OD.^2 - section_props.ID.^2);  % m²
    section_props.I = pi/64 * (section_props.OD.^4 - section_props.ID.^4);    % m⁴
    section_props.c = section_props.OD / 2;  % m (outer radius)
    section_props.section_modulus = section_props.I ./ section_props.c;  % m³
    
    fprintf('Tower sections: %d segments\n', n_sections);
    fprintf('Base diameter: %.2f m, Top diameter: %.2f m\n', section_props.OD(1), section_props.OD(end));
end

function deflection_results = solveTowerDeflection(section_props, load_cases, E, data)
% Solve beam deflection using numerical integration
    
    % Discretize tower into elements
    n_elements = 50;
    z = linspace(0, section_props.height(end), n_elements+1);
    dz = z(2) - z(1);
    
    % Initialize deflection arrays
    deflection_results.z = z;
    deflection_results.deflection = zeros(size(z));
    deflection_results.moment = zeros(size(z));
    
    % Calculate deflection for Load Case 1 (maximum)
    [deflection_case1, moment_case1] = calculateDeflectionCase(section_props, load_cases.case1, z, E, data);
    deflection_results.deflection = deflection_case1;
    deflection_results.moment = moment_case1;
    
    % Store case-specific results
    deflection_results.case1 = struct('deflection', deflection_case1, 'moment', moment_case1);
    [deflection_case2, moment_case2] = calculateDeflectionCase(section_props, load_cases.case2, z, E, data);
    deflection_results.case2 = struct('deflection', deflection_case2, 'moment', moment_case2);
    
    fprintf('Maximum deflection: %.3f m at tip\n', max(abs(deflection_case1)));
end

function [deflection, moment] = calculateDeflectionCase(section_props, load_case, z, E, data)
% Calculate deflection for a specific load case
    
    n_points = length(z);
    deflection = zeros(size(z));
    moment = zeros(size(z));
    
    % Calculate wind profile and distributed loading
    [wind_profile, drag_force] = computeWindLoading(section_props, load_case, z, data);
    
    % Calculate moment distribution (simplified approach)
    for i = 1:n_points
        % Thrust moment at hub height
        if z(i) <= 80.4  % Hub height
            thrust_moment = load_case.thrust_force * (80.4 - z(i));
        else
            thrust_moment = 0;
        end
        
        % Wind moment (integrate from current point to tip)
        wind_moment = 0;
        for j = i:n_points-1
            if j < n_points
                wind_moment = wind_moment + drag_force(j) * (z(j+1) - z(j)) * (z(j) - z(i));
            end
        end
        
        moment(i) = thrust_moment + wind_moment;
    end
    
    % Calculate deflection using moment-area method (simplified)
    for i = 1:n_points
        % Get section properties at current height
        I_current = interp1(section_props.height, section_props.I, z(i), 'linear', 'extrap');
        
        % Simple integration for deflection
        deflection(i) = sum(moment(1:i) .* (z(2) - z(1))) / (E * I_current);
    end
end

function [wind_profile, drag_force] = computeWindLoading(section_props, load_case, z, data)
% Compute wind profile and distributed drag force
    
    % Atmospheric boundary layer wind profile
    z_ref = section_props.height(end);  % Reference height (tower top)
    U_ref = load_case.wind_speed;
    epsilon = 1/7;
    
    wind_profile = U_ref * (z / z_ref).^epsilon;
    
    % Get tower diameter at each height
    D = interp1(section_props.height, section_props.OD, z, 'linear', 'extrap');
    
    % Calculate drag force per unit length
    rho_air = data.materials.air.density;  % kg/m³
    
    % Calculate drag coefficient using cylinder CD function
    % Use representative wind speed and tower diameter for Reynolds number
    U_avg = mean(wind_profile);
    D_avg = mean(D);
    mu_air = data.materials.air.viscosity;
    Re = rho_air * U_avg * D_avg / mu_air;
    C_d = cylinderCDlocal(Re);
    
    % Distributed drag force per unit length
    drag_force = 0.5 * rho_air * wind_profile.^2 .* D * C_d;
end

function stress_results = computeStressAnalysis(section_props, load_cases, deflection_results, E, data)
% Calculate stress analysis at tower base
    
    % Get base section properties
    base_I = section_props.I(1);
    base_c = section_props.c(1);
    base_area = section_props.area(1);
    
    % Calculate stresses for both load cases
    stress_results = struct();
    
    % Load Case 1 (maximum)
    moment_1 = deflection_results.case1.moment(1);
    thrust_1 = load_cases.case1.thrust_force;
    
    sigma_bending_1 = moment_1 * base_c / base_I;  % Bending stress
    sigma_axial_1 = thrust_1 / base_area;          % Axial stress
    sigma_max_1 = sigma_bending_1 + sigma_axial_1; % Combined stress
    
    % Load Case 2 (minimum)
    moment_2 = deflection_results.case2.moment(1);
    thrust_2 = load_cases.case2.thrust_force;
    
    sigma_bending_2 = moment_2 * base_c / base_I;
    sigma_axial_2 = thrust_2 / base_area;
    sigma_max_2 = sigma_bending_2 + sigma_axial_2;
    
    % Store results
    stress_results.case1 = struct('sigma_bending', sigma_bending_1, 'sigma_axial', sigma_axial_1, ...
        'sigma_max', sigma_max_1, 'moment', moment_1);
    stress_results.case2 = struct('sigma_bending', sigma_bending_2, 'sigma_axial', sigma_axial_2, ...
        'sigma_max', sigma_max_2, 'moment', moment_2);
    
    % Calculate mean and alternating stresses for fatigue
    stress_results.sigma_mean = (sigma_max_1 + sigma_max_2) / 2;
    stress_results.sigma_alt = (sigma_max_1 - sigma_max_2) / 2;
    stress_results.max_stress = [sigma_max_1, sigma_max_2];
    
    % Fatigue analysis
    S_ut = 450e6;  % Pa (from function parameter)
    S_e = data.materials.steel.enduranceLimitFactor * S_ut;  % Endurance limit
    n_fatigue = 1 / (stress_results.sigma_alt/S_e + stress_results.sigma_mean/S_ut);
    stress_results.fatigue_safety_factor = n_fatigue;
    
    fprintf('Base stress - Case 1: %.1f MPa, Case 2: %.1f MPa\n', ...
        sigma_max_1/1e6, sigma_max_2/1e6);
    fprintf('Mean stress: %.1f MPa, Alternating stress: %.1f MPa\n', ...
        stress_results.sigma_mean/1e6, stress_results.sigma_alt/1e6);
end

function createTowerDeflectionPlot(deflection_results, section_props)
% Create tower deflection visualization
    
    figure('Position', [100, 100, 800, 600]);
    
    subplot(2,1,1);
    plot(deflection_results.z, deflection_results.case1.deflection, 'b-', 'LineWidth', 2);
    hold on;
    plot(deflection_results.z, deflection_results.case2.deflection, 'r--', 'LineWidth', 2);
    xlabel('Height (m)');
    ylabel('Deflection (m)');
    title('Tower Deflection Profile');
    legend('Load Case 1 (Max)', 'Load Case 2 (Min)', 'Location', 'best');
    grid on;
    
    subplot(2,1,2);
    plot(section_props.height, section_props.OD, 'k-', 'LineWidth', 2);
    xlabel('Height (m)');
    ylabel('Tower Diameter (m)');
    title('Tower Geometry');
    grid on;
    
    saveas(gcf, 'Tower_Deflection_Analysis.png');
    fprintf('Tower deflection plot saved as Tower_Deflection_Analysis.png\n');
end

function createMohrCirclePlots(stress_results)
% Create Mohrs circle visualizations
    
    figure('Position', [200, 200, 1200, 500]);
    
    % Load Case 1
    subplot(1,2,1);
    sigma_x = stress_results.case1.sigma_max;
    sigma_y = 0;  % Assume no stress in y-direction
    tau_xy = 0;   % Assume no shear stress
    
    plotMohrCircle(sigma_x, sigma_y, tau_xy, 'Load Case 1 (Maximum)');
    
    % Load Case 2
    subplot(1,2,2);
    sigma_x = stress_results.case2.sigma_max;
    sigma_y = 0;
    tau_xy = 0;
    
    plotMohrCircle(sigma_x, sigma_y, tau_xy, 'Load Case 2 (Minimum)');
    
    saveas(gcf, 'Mohr_Circle_Analysis.png');
    fprintf('Mohr circle plots saved as Mohr_Circle_Analysis.png\n');
end

function plotMohrCircle(sigma_x, sigma_y, tau_xy, title_str)
% Plot Mohrs circle for given stress state
    
    % Calculate center and radius
    sigma_center = (sigma_x + sigma_y) / 2;
    R = sqrt(((sigma_x - sigma_y) / 2)^2 + tau_xy^2);
    
    % Principal stresses
    sigma_1 = sigma_center + R;
    sigma_2 = sigma_center - R;
    tau_max = R;
    
    % Create circle
    theta = linspace(0, 2*pi, 100);
    x_circle = sigma_center + R * cos(theta);
    y_circle = R * sin(theta);
    
    plot(x_circle/1e6, y_circle/1e6, 'b-', 'LineWidth', 2);
    hold on;
    plot([sigma_1/1e6, sigma_2/1e6], [0, 0], 'ro', 'MarkerSize', 8, 'LineWidth', 2);
    plot(sigma_center/1e6, 0, 'ko', 'MarkerSize', 6);
    
    xlabel('Normal Stress (MPa)');
    ylabel('Shear Stress (MPa)');
    title(title_str);
    grid on;
    axis equal;
    
    % Add annotations
    text(sigma_1/1e6, 0, sprintf('σ₁=%.1f MPa', sigma_1/1e6), 'VerticalAlignment', 'bottom');
    text(sigma_2/1e6, 0, sprintf('σ₂=%.1f MPa', sigma_2/1e6), 'VerticalAlignment', 'top');
    text(sigma_center/1e6, R/1e6, sprintf('τₘₐₓ=%.1f MPa', tau_max/1e6), 'HorizontalAlignment', 'center');
end

function createGoodmanDiagram(stress_results, S_ut, S_y, data)
% Create Goodman diagram for fatigue analysis
    
    figure('Position', [300, 300, 800, 600]);
    
    % Material properties
    S_e = data.materials.steel.enduranceLimitFactor * S_ut;  % Endurance limit
    
    % Operating point
    sigma_m = stress_results.sigma_mean;
    sigma_a = stress_results.sigma_alt;
    
    % Goodman line
    sigma_m_range = linspace(0, S_ut, 100);
    sigma_a_goodman = S_e * (1 - sigma_m_range / S_ut);
    
    % Plot Goodman line
    plot(sigma_m_range/1e6, sigma_a_goodman/1e6, 'r-', 'LineWidth', 2, 'DisplayName', 'Goodman Line');
    hold on;
    
    % Plot operating point
    plot(sigma_m/1e6, sigma_a/1e6, 'bo', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Operating Point');
    
    % Add yield line
    sigma_m_yield = linspace(0, S_y, 50);
    sigma_a_yield = S_y - sigma_m_yield;
    plot(sigma_m_yield/1e6, sigma_a_yield/1e6, 'g--', 'LineWidth', 2, 'DisplayName', 'Yield Line');
    
    xlabel('Mean Stress (MPa)');
    ylabel('Alternating Stress (MPa)');
    title('Goodman Diagram for Fatigue Analysis');
    legend('Location', 'best');
    grid on;
    
    % Add safety factor annotation
    n_fatigue = stress_results.fatigue_safety_factor;
    text(0.1, 0.9, sprintf('Safety Factor: %.2f', n_fatigue), 'Units', 'normalized', ...
        'BackgroundColor', 'white', 'EdgeColor', 'black');
    
    saveas(gcf, 'Goodman_Diagram_Analysis.png');
    fprintf('Goodman diagram saved as Goodman_Diagram_Analysis.png\n');
end

% Helper functions from Deliverable4 (reused)
function [a, a_prime, CL, CD, Cn, Ct, V_rel] = getSectionCoefficients(airfoil, twist_deg, r, c, lambda_r, V_wind, omega_rad, pitch_rad, data, rho, mu, B, R)
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

function [a, a_prime, CL, CD, Cn, Ct, V_rel] = solveBEMSection(r, c, twist_rad, perf_data, lambda_r, V_wind, omega_rad, pitch_rad)
    a = 1/3;
    a_prime = -0.5 + 0.5 * sqrt(1 + (4/(lambda_r^2)) * a * (1 - a));
    
    phi = atan((1 - a) / ((1 + a_prime) * lambda_r));
    alpha = phi - (twist_rad + pitch_rad);
    alpha_deg = rad2deg(alpha);
    
    % Interpolate CL and CD at the calculated angle of attack
    CL = interp1(perf_data.AoA, perf_data.CL, alpha_deg, 'linear', 'extrap');
    CD = interp1(perf_data.AoA, perf_data.CD, alpha_deg, 'linear', 'extrap');
    
    s = sin(phi); cphi = cos(phi);
    Cn = CL * cphi + CD * s;
    Ct = CL * s - CD * cphi;
    
    V_rel = sqrt((V_wind*(1-a))^2 + (omega_rad*r*(1+a_prime))^2);
end

function [a, a_prime, CL, CD, Cn, Ct, V_rel] = solveCircleSection(r, c, twist_rad, lambda_r, V_wind, omega_rad, pitch_rad, rho, mu)
    a = 1/3; a_prime = -0.5 + 0.5 * sqrt(1 + (4/(lambda_r^2)) * a * (1 - a));
    phi = atan((1 - a) / ((1 + a_prime) * lambda_r));
    V_rel = sqrt((V_wind*(1-a))^2 + (omega_rad*r*(1+a_prime))^2);
    Re = max(1, rho * V_rel * c / mu);
    CD = cylinderCDlocal(Re); CL = 0;
    s = sin(phi); cphi = cos(phi);
    Cn = CL * cphi + CD * s; Ct = CL * s - CD * cphi;
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

function F = prandtlLossFactor(B, r, R, lambda_r, a, a_prime)
    phi = atan((1 - a) / ((1 + a_prime) * lambda_r + eps));
    sphi = sin(abs(phi)) + eps;
    mu = r / R;
    f_tip = (B/2) * (1 - mu) / sphi;
    f_root = (B/2) * (mu) / sphi;
    F_tip = (2/pi) * acos(exp(-max(0, f_tip)));
    F_root = (2/pi) * acos(exp(-max(0, f_root)));
    F = max(1e-3, F_tip * F_root);
end
