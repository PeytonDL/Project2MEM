% ============================================================================
% MASTER WIND TURBINE ANALYSIS SCRIPT
% ============================================================================
% 
% Comprehensive wind turbine analysis tool consolidating all deliverables into a
% single executable script with simplified boolean configuration system.
% 
% DELIVERABLES:
%   Deliverable1() - Basic BEM Analysis (CP, CT calculation)
%   Deliverable2() - Pitch Optimization (find optimal pitch angle)
%   Deliverable3() - 2D CP Optimization (lambda and pitch optimization)
%   Deliverable4() - Rated Power Pitch Control (pitch for rated power)
%   Deliverable5() - Tower Deflection and Stress Analysis (structural analysis)
%
% MASTER CONTROL:
%   runAllDeliverables() - Executes all deliverables in sequence with error handling
%
% CONFIGURATION:
%   Edit boolean variables in runAllDeliverables() function (lines 102-117)
%   to control which deliverables and plots are executed.
%
% DATA FLOW:
%   D1 → D2 → D3 → D4 → D5 (results passed between deliverables)
%   D4 provides lambda and pitch_deg to D5 for thrust calculation
%
% DEPENDENCIES:
%   - Auxilary Information/Given Parameters/ folder with CSV files
%   - BladeProfile.csv, towerSpecs.csv, airfoil .dat files
%   - Airfoil performance data files
%
% USAGE:
%   1. Edit configuration variables in runAllDeliverables() (lines 102-117)
%   2. Hit Run button or press F5 to execute
%   3. Or run individual deliverables: Deliverable1(), Deliverable2(), etc.
%
% VERSION: 2.0
% LAST UPDATED: 2025
% ============================================================================

%% ============================================================================
%% SCRIPT EXECUTION
%% ============================================================================
% 
% TO RUN THE ANALYSIS:
% 1. Edit the boolean variables inside runAllDeliverables() function (lines 102-117)
% 2. Hit the Run button or press F5 to execute
% 3. Or run individual deliverables: Deliverable1(), Deliverable2(), etc.
%
% Clear workspace and close figures
clc; close all;

% Disable all warnings
warning('off', 'all');

% Execute the analysis when script is run
runAllDeliverables();

function runAllDeliverables()
% RUNALLDELIVERABLES Master control function to run all deliverables in sequence
% Usage: runAllDeliverables()
% 
% Configuration is controlled by boolean variables at the top of the script:
%   RUN_DELIVERABLE_1, RUN_DELIVERABLE_2, etc. - Enable/disable deliverables
%   PLOT_D1_RESULTS, PLOT_D2_OPTIMIZATION, etc. - Enable/disable plots
%   SAVE_PLOTS - Save plots to files
%   VERBOSE_OUTPUT - Show detailed progress information
%
% NOTE: Variables and figures are preserved across all deliverables to allow
% for comparison and analysis of results from different deliverables.

    % Make configuration variables global so they can be accessed from functions
    global RUN_DELIVERABLE_1 RUN_DELIVERABLE_2 RUN_DELIVERABLE_3 RUN_DELIVERABLE_4 RUN_DELIVERABLE_5
    global PLOT_D1_RESULTS PLOT_D2_OPTIMIZATION PLOT_D3_OPTIMIZATION PLOT_D5_DEFLECTION PLOT_D5_MOHR PLOT_D5_GOODMAN
    global SAVE_PLOTS VERBOSE_OUTPUT PARAMETERS_PATH

    % Define configuration variables (these are the defaults - edit these values to change behavior)
    RUN_DELIVERABLE_1 = true;    % Basic BEM Analysis
    RUN_DELIVERABLE_2 = true;    % Pitch Optimization
    RUN_DELIVERABLE_3 = true;    % 2D CP Optimization
    RUN_DELIVERABLE_4 = false;    % Rated Power Pitch Control
    RUN_DELIVERABLE_5 = false;    % Tower Deflection Analysis
    
    PLOT_D1_RESULTS = false;      % D1: BEM Analysis Results
    PLOT_D2_OPTIMIZATION = false; % D2: Pitch Optimization Plot
    PLOT_D3_OPTIMIZATION = true; % D3: 2D Optimization Plot
    PLOT_D5_DEFLECTION = false;   % D5: Tower Deflection Plot
    PLOT_D5_MOHR = false;         % D5: Mohr Circle Plot
    PLOT_D5_GOODMAN = false;       % D5: Goodman Diagram
    
    SAVE_PLOTS = false;          % Save plots to files
    VERBOSE_OUTPUT = false;       % Show detailed progress information
    PARAMETERS_PATH = 'Auxilary Information/Given Parameters/';  % Path to data files

    fprintf('=== MASTER WIND TURBINE ANALYSIS ===\n');
    if VERBOSE_OUTPUT
        fprintf('Configuration: Deliverables [%s], Save: %s\n', ...
            mat2str([RUN_DELIVERABLE_1, RUN_DELIVERABLE_2, RUN_DELIVERABLE_3, RUN_DELIVERABLE_4, RUN_DELIVERABLE_5]), ...
            mat2str(SAVE_PLOTS));
        fprintf('Plots: D1=%s, D2=%s, D3=%s, D5_deflection=%s, D5_mohr=%s, D5_goodman=%s\n', ...
            mat2str(PLOT_D1_RESULTS), mat2str(PLOT_D2_OPTIMIZATION), mat2str(PLOT_D3_OPTIMIZATION), ...
            mat2str(PLOT_D5_DEFLECTION), mat2str(PLOT_D5_MOHR), mat2str(PLOT_D5_GOODMAN));
    end
    fprintf('\n');
    
    try
        results = struct();
        
        % Deliverable 1: Basic BEM Analysis
        if RUN_DELIVERABLE_1
            fprintf('Running Deliverable 1...\n');
            [CP1, CT1] = Deliverable1();
            results.D1 = struct('CP', CP1, 'CT', CT1);
            fprintf('D1 Complete: CP=%.4f, CT=%.4f\n\n', CP1, CT1);
        else
            fprintf('Skipping Deliverable 1 (disabled)\n\n');
        end
        
        % Deliverable 2: Pitch Optimization
        if RUN_DELIVERABLE_2
            fprintf('Running Deliverable 2...\n');
            [CP2, optimal_conditions] = Deliverable2();
            results.D2 = struct('CP', CP2, 'optimal_conditions', optimal_conditions);
            fprintf('D2 Complete: Max CP=%.4f at pitch=%.1f°\n\n', CP2, optimal_conditions.pitch_angle);
        else
            fprintf('Skipping Deliverable 2 (disabled)\n\n');
        end
        
        % Deliverable 3: 2D Optimization
        if RUN_DELIVERABLE_3
            fprintf('Running Deliverable 3...\n');
            [CP3, optimal_conditions_3D] = Deliverable3();
            results.D3 = struct('CP', CP3, 'optimal_conditions', optimal_conditions_3D);
            fprintf('D3 Complete: Max CP=%.4f at λ=%.2f, pitch=%.1f°\n\n', CP3, optimal_conditions_3D.lambda, optimal_conditions_3D.pitch_angle);
        else
            fprintf('Skipping Deliverable 3 (disabled)\n\n');
        end
        
        % Deliverable 4: Rated Power Pitch Control
        if RUN_DELIVERABLE_4
            fprintf('Running Deliverable 4...\n');
            result4 = Deliverable4();
            results.D4 = result4;
            fprintf('D4 Complete: Required pitch=%.1f° at λ=%.2f\n\n', result4.pitch_deg, result4.lambda);
        else
            fprintf('Skipping Deliverable 4 (disabled)\n\n');
            % Create dummy result for D5 if needed
            result4 = struct('lambda', 5.0, 'pitch_deg', 0.0);
        end
        
        % Deliverable 5: Tower Analysis (using D4 results)
        if RUN_DELIVERABLE_5
            fprintf('Running Deliverable 5...\n');
            result5 = Deliverable5(result4.lambda, result4.pitch_deg);
            results.D5 = result5;
            fprintf('D5 Complete: Max deflection=%.3f m, Safety factor=%.2f\n\n', max(abs(result5.deflection.deflection)), result5.stress.fatigue_safety_factor);
        else
            fprintf('Skipping Deliverable 5 (disabled)\n\n');
        end
        
        % Summary
        fprintf('=== ALL DELIVERABLES COMPLETE ===\n');
        if RUN_DELIVERABLE_1
            fprintf('D1: CP=%.4f, CT=%.4f\n', results.D1.CP, results.D1.CT);
        end
        if RUN_DELIVERABLE_2
            fprintf('D2: Max CP=%.4f at pitch=%.1f°\n', results.D2.CP, results.D2.optimal_conditions.pitch_angle);
        end
        if RUN_DELIVERABLE_3
            fprintf('D3: Max CP=%.4f at λ=%.2f, pitch=%.1f°\n', results.D3.CP, results.D3.optimal_conditions.lambda, results.D3.optimal_conditions.pitch_angle);
        end
        if RUN_DELIVERABLE_4
            fprintf('D4: Required pitch=%.1f° at λ=%.2f\n', results.D4.pitch_deg, results.D4.lambda);
        end
        if RUN_DELIVERABLE_5
            fprintf('D5: Max deflection=%.3f m, Safety factor=%.2f\n', max(abs(results.D5.deflection.deflection)), results.D5.stress.fatigue_safety_factor);
        end
        
    catch ME
        fprintf('Error in deliverable execution: %s\n', ME.message);
        fprintf('Stack trace:\n');
        for i = 1:length(ME.stack)
            fprintf('  %s (line %d)\n', ME.stack(i).name, ME.stack(i).line);
        end
    end
end

% ============================================================================
% DELIVERABLE 1: BASIC BEM ANALYSIS
% ============================================================================
% Calculates power and thrust coefficients for specified operating conditions
% using Blade Element Momentum (BEM) theory with airfoil performance data.

function [CP, CT] = Deliverable1()
% DELIVERABLE1 Wind Turbine BEM Analysis
% Calculates power and thrust coefficients for specified operating conditions
% Usage: [CP, CT] = Deliverable1();

    % Access global configuration variables
    global PLOT_D1_RESULTS SAVE_PLOTS PARAMETERS_PATH

    fprintf('=== Wind Turbine Analysis - Deliverable 1 ===\n\n');
    
    data = ParameterExtraction();
    
    V_wind = data.deliverables.part1.V_wind;
    omega = data.deliverables.part1.omega_rpm;
    pitch_angle = data.deliverables.part1.pitch_deg;
    
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
    
    % Exclude hub region to avoid non-lifting root effects
    r_stations = blade_profile.DistanceFromCenterOfRotation / 1000;
    hub_radius = data.turbine.performance.hubRadius;
    use_idx = r_stations >= hub_radius;
    r_stations = r_stations(use_idx);
    chord_m = blade_profile.ChordLength(use_idx) / 1000;
    twist_deg_vec = blade_profile.BladeTwist(use_idx);
    airfoil_raw = blade_profile.Airfoil(use_idx);
    n_stations = numel(r_stations);
    
    fprintf('  Analyzing %d blade stations (hub region excluded)...\n', n_stations);
    
    dCP = zeros(n_stations, 1);
    dCT = zeros(n_stations, 1);
    dT = zeros(n_stations, 1);
    dQ = zeros(n_stations, 1);
    dP = zeros(n_stations, 1);
    B = data.turbine.characteristics.blades;
    
    % Constant inputs precomputed once
    pitch_rad = deg2rad(pitch_angle);
    
    % Iterate over blade stations to accumulate sectional loads and power
    for i = 1:n_stations
        r = r_stations(i);
        c = chord_m(i);
        twist = twist_deg_vec(i);
        airfoil = airfoil_raw{i};
        
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
    
    % Create visualization if enabled
    if PLOT_D1_RESULTS
    createVisualization(r_stations, dCP, dCT, CP, CT, lambda, V_wind, omega);
    end
    fprintf('\nAnalysis complete!\n');
end

% ============================================================================
% DELIVERABLE 2: PITCH OPTIMIZATION
% ============================================================================
% Finds optimal pitch angle for maximum power coefficient at fixed wind speed
% and tip speed ratio using iterative BEM analysis.

function [CP_max, optimal_conditions] = Deliverable2(V_wind, lambda, pitch_min, pitch_max, pitch_step)
% DELIVERABLE2 Wind Turbine Pitch Angle Optimization
% Finds maximum power coefficient by varying pitch angle at fixed wind speed and tip speed ratio
% Usage: [CP_max, optimal_conditions] = Deliverable2();
%        [CP_max, optimal_conditions] = Deliverable2(V_wind, lambda, pitch_min, pitch_max, pitch_step);

    % Access global configuration variables
    global PLOT_D2_OPTIMIZATION SAVE_PLOTS PARAMETERS_PATH

    fprintf('=== Wind Turbine Pitch Angle Optimization ===\n\n');
    
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
    % Exclude hub region to avoid non-lifting root effects
    r_stations = blade_profile.DistanceFromCenterOfRotation / 1000; % station radii [m]
    hub_radius = data.turbine.performance.hubRadius;
    use_idx = r_stations >= hub_radius;
    r_stations = r_stations(use_idx);
    chord_m = blade_profile.ChordLength(use_idx) / 1000; % chord lengths [m]
    twist_deg_vec = blade_profile.BladeTwist(use_idx); % twist [deg]
    airfoil_raw = blade_profile.Airfoil(use_idx); % raw airfoil labels
    n_stations = numel(r_stations);
    lambda_r_vec = lambda * r_stations / R; % local TSR at each station
    B = data.turbine.characteristics.blades;
    
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
    
    % Create visualization if enabled
    if PLOT_D2_OPTIMIZATION
    createPitchOptimizationPlot(pitch_range, CP_values, CT_values, optimal_pitch, CP_max, V_wind, lambda);
    end
    fprintf('\nPitch optimization complete!\n');
end

% ============================================================================
% DELIVERABLE 3: 2D CP OPTIMIZATION
% ============================================================================
% Performs 2D optimization over tip speed ratio and pitch angle to find
% maximum power coefficient using nested optimization loops.

function [CP_max, optimal_conditions] = Deliverable3(V_wind, lambda_min, lambda_max, lambda_step, pitch_min, pitch_max, pitch_step)
% DELIVERABLE3 Wind Turbine 2D CP Optimization
% Finds maximum power coefficient by varying tip speed ratio and pitch angle
% Usage: [CP_max, optimal_conditions] = Deliverable3();
%        [CP_max, optimal_conditions] = Deliverable3(V_wind, lambda_min, lambda_max, lambda_step, pitch_min, pitch_max, pitch_step);

    % Access global configuration variables
    global PLOT_D3_OPTIMIZATION SAVE_PLOTS PARAMETERS_PATH

    fprintf('=== Wind Turbine 2D CP Optimization ===\n\n');
    
    data = ParameterExtraction();

    % Defaults from predefined deliverables when inputs are omitted
    if nargin < 1 || isempty(V_wind), V_wind = data.deliverables.part3.V_wind; end
    if nargin < 2 || isempty(lambda_min), lambda_min = 3; end
    if nargin < 3 || isempty(lambda_max), lambda_max = 12; end
    if nargin < 4 || isempty(lambda_step), lambda_step = 0.5; end
    if nargin < 5 || isempty(pitch_min), pitch_min = -15; end
    if nargin < 6 || isempty(pitch_max), pitch_max = 15; end
    if nargin < 7 || isempty(pitch_step), pitch_step = 1; end

    fprintf('Optimizing CP for:\n');
    fprintf('  Wind velocity: %.1f m/s\n', V_wind);
    fprintf('  Tip speed ratio range: %.1f to %.1f (step: %.1f)\n', lambda_min, lambda_max, lambda_step);
    fprintf('  Pitch angle range: %.1f to %.1f degrees (step: %.1f)\n', pitch_min, pitch_max, pitch_step);
    
    R = data.turbine.performance.rotorRadius;
    A = data.turbine.calculated.rotorArea;
    rho = data.materials.air.density;
    
    lambda_range = lambda_min:lambda_step:lambda_max;
    pitch_range = pitch_min:pitch_step:pitch_max;
    n_lambda = length(lambda_range);
    n_pitch = length(pitch_range);
    
    CP_matrix = zeros(n_pitch, n_lambda);
    CT_matrix = zeros(n_pitch, n_lambda);
    P_matrix = zeros(n_pitch, n_lambda);
    T_matrix = zeros(n_pitch, n_lambda);
    
    fprintf('Performing 2D CP optimization...\n');
    
    blade_profile = data.blade.profile;
    % Exclude hub region to avoid non-lifting root effects
    r_stations = blade_profile.DistanceFromCenterOfRotation / 1000; % station radii [m]
    hub_radius = data.turbine.performance.hubRadius;
    use_idx = r_stations >= hub_radius;
    r_stations = r_stations(use_idx);
    chord_m = blade_profile.ChordLength(use_idx) / 1000; % chord lengths [m]
    twist_deg_vec = blade_profile.BladeTwist(use_idx); % twist [deg]
    airfoil_raw = blade_profile.Airfoil(use_idx); % raw airfoil labels
    n_stations = numel(r_stations);
    B = data.turbine.characteristics.blades;
    
    total_iterations = n_lambda * n_pitch;
    current_iteration = 0;
    
    for j = 1:n_lambda
        lambda = lambda_range(j);
        omega_rad = (lambda * V_wind) / R;
        
        for k = 1:n_pitch
            pitch_angle = pitch_range(k);
            current_iteration = current_iteration + 1;
            
            pitch_rad = deg2rad(pitch_angle);
            dT = zeros(n_stations, 1);
            dQ = zeros(n_stations, 1);
            dP = zeros(n_stations, 1);
            
            for i = 1:n_stations
                r = r_stations(i);
                c = chord_m(i);
                twist = twist_deg_vec(i);
                airfoil = airfoil_raw{i};
                
                lambda_r = lambda * r / R; % local tip-speed ratio at radius r
                
                [a, a_prime, CL, CD, Cn, Ct, V_rel] = getSectionCoefficients(airfoil, twist, r, c, ...
                    lambda_r, V_wind, omega_rad, pitch_rad, data, rho, data.materials.air.viscosity);
                
                dT(i) = 0.5 * rho * V_rel^2 * c * Cn;
                dQ(i) = 0.5 * rho * V_rel^2 * c * Ct * r;
                dP(i) = dQ(i) * omega_rad;
            end
            
            T_total = B * trapz(r_stations, dT);
            Q_total = B * trapz(r_stations, dQ);
            P_total = B * trapz(r_stations, dP);
            
            CP_matrix(k, j) = P_total / (0.5 * rho * A * V_wind^3);
            CT_matrix(k, j) = T_total / (0.5 * rho * A * V_wind^2);
            P_matrix(k, j) = P_total;
            T_matrix(k, j) = T_total;
            
        end
    end
    
    [CP_max, max_idx] = max(CP_matrix(:));
    [optimal_pitch_idx, optimal_lambda_idx] = ind2sub(size(CP_matrix), max_idx);
    
    optimal_lambda = lambda_range(optimal_lambda_idx);
    optimal_pitch = pitch_range(optimal_pitch_idx);
    optimal_omega_rad = (optimal_lambda * V_wind) / R;
    optimal_omega_rpm = optimal_omega_rad * 60 / (2 * pi);
    
    optimal_conditions = struct();
    optimal_conditions.lambda = optimal_lambda;
    optimal_conditions.pitch_angle = optimal_pitch;
    optimal_conditions.omega_rad = optimal_omega_rad;
    optimal_conditions.omega_rpm = optimal_omega_rpm;
    optimal_conditions.CP = CP_max;
    optimal_conditions.CT = CT_matrix(optimal_pitch_idx, optimal_lambda_idx);
    optimal_conditions.P = P_matrix(optimal_pitch_idx, optimal_lambda_idx);
    optimal_conditions.T = T_matrix(optimal_pitch_idx, optimal_lambda_idx);
    optimal_conditions.V_wind = V_wind;
    
    fprintf('\n=== OPTIMIZATION RESULTS ===\n');
    fprintf('Maximum CP: %.4f\n', CP_max);
    fprintf('Optimal tip speed ratio: %.3f\n', optimal_lambda);
    fprintf('Optimal pitch angle: %.1f degrees\n', optimal_pitch);
    fprintf('Optimal rotational speed: %.1f rpm\n', optimal_omega_rpm);
    fprintf('Optimal thrust coefficient: %.4f\n', optimal_conditions.CT);
    fprintf('Optimal power: %.1f kW\n', optimal_conditions.P/1000);
    fprintf('Optimal thrust: %.1f kN\n', optimal_conditions.T/1000);
    
    % Create visualization if enabled
    if PLOT_D3_OPTIMIZATION
    create2DOptimizationPlot(lambda_range, pitch_range, CP_matrix, CT_matrix, optimal_lambda, optimal_pitch, CP_max, V_wind);
    end
    fprintf('\n2D optimization complete!\n');
end

% ============================================================================
% DELIVERABLE 4: RATED POWER PITCH CONTROL
% ============================================================================
% Determines required pitch angle to maintain rated power at high wind speeds
% using iterative BEM analysis with power constraint.

function result = Deliverable4()
% DELIVERABLE4 Rated Power Pitch Control
% Determine the blade pitch angle required to keep turbine power <= rated
% at the predefined high wind speed in ParameterExtraction (part4).
%
% Usage: result = Deliverable4();
% Returns struct with fields: pitch_deg, lambda, omega_rad, omega_rpm, P, CP

    fprintf('=== Deliverable 4: Rated Power Pitch Determination ===\n\n');

    data = ParameterExtraction();

    % Inputs and constants
    V_wind = data.deliverables.part4.V_wind;           % m/s
    ratedPower = data.turbine.performance.ratedPower;  % W
    R = data.turbine.performance.rotorRadius;          % m
    A = data.turbine.calculated.rotorArea;             % m^2
    rho = data.materials.air.density;                  % kg/m^3
    rpm_range = data.turbine.performance.rotorSpeedRange; % [min,max] RPM
    omega_range = rpm_range * 2*pi/60;                 % rad/s

    % Lambda range constrained by speed limits
    lambda_min = max(2.0, omega_range(1) * R / V_wind);
    lambda_max = omega_range(2) * R / V_wind;
    lambda_step = 0.25;
    lambda_range = lambda_min:lambda_step:lambda_max;

    % Pitch sweep (deg) - conservative range for feathering
    pitch_min = 0; pitch_max = 30; pitch_step = 0.5;
    pitch_range = pitch_min:pitch_step:pitch_max;

    fprintf('Wind = %.2f m/s, Rated Power = %.1f kW, λ in [%.2f, %.2f]\n', V_wind, ratedPower/1e3, lambda_min, lambda_max);

    % Precompute station data
    blade_profile = data.blade.profile;
    r_stations = blade_profile.DistanceFromCenterOfRotation / 1000; % m
    chord_m = blade_profile.ChordLength / 1000; % m
    twist_deg_vec = blade_profile.BladeTwist;   % deg
    airfoil_raw = blade_profile.Airfoil;        % labels

    % Exclude hub region to avoid non-lifting root effects
    HUB_EXCLUDE = true;
   
    hub_radius = data.turbine.performance.hubRadius; % m
    

    if HUB_EXCLUDE
        use_idx = r_stations >= hub_radius;
        r_stations = r_stations(use_idx);
        chord_m = chord_m(use_idx);
        twist_deg_vec = twist_deg_vec(use_idx);
        airfoil_raw = airfoil_raw(use_idx);
    end
    n_stations = numel(r_stations);

    % Preallocate accumulators
    B = data.turbine.characteristics.blades;
    CP_pitch = zeros(numel(pitch_range), 1);
    lambda_at_CP = zeros(numel(pitch_range), 1);

    % Outer loop over pitch: for each, find worst-case (max) CP across allowed λ
    for j = 1:numel(pitch_range)
        pitch_deg = pitch_range(j);
        pitch_rad = deg2rad(pitch_deg);
        best_CP = -inf; best_lambda = lambda_range(1); best_P = 0; best_omega = 0;

        for lam = lambda_range
            omega_rad = lam * V_wind / R;

            % Per-station sums
            dT = zeros(n_stations, 1);
            dQ = zeros(n_stations, 1);
            dP = zeros(n_stations, 1);

            for i = 1:n_stations
                r = r_stations(i);
                c = chord_m(i);
                twist_deg = twist_deg_vec(i);
                airfoil = airfoil_raw{i};
                lambda_r = lam * r / R;

                [a, a_prime, CL, CD, Cn, Ct, V_rel] = getSectionCoefficients(airfoil, twist_deg, r, c, ...
                    lambda_r, V_wind, omega_rad, pitch_rad, data, rho, data.materials.air.viscosity, B, R);


                dT(i) = 0.5 * rho * V_rel^2 * c * Cn;
                dQ(i) = 0.5 * rho * V_rel^2 * c * Ct * r;
                dP(i) = dQ(i) * omega_rad;
            end

            T_total = B * trapz(r_stations, dT);
            Q_total = B * trapz(r_stations, dQ);
            P_total = B * trapz(r_stations, dP);

            CP_val = P_total / (0.5 * rho * A * V_wind^3);
            if CP_val > best_CP
                best_CP = CP_val; best_lambda = lam; best_P = P_total; best_omega = omega_rad;
            end
        end

        CP_pitch(j) = best_CP;
        lambda_at_CP(j) = best_lambda;

        fprintf('Pitch %5.1f deg: max CP=%.4f at λ=%.2f -> P=%.1f kW\n', pitch_deg, best_CP, best_lambda, (best_CP*0.5*rho*A*V_wind^3)/1e3);
    end

    P_pitch = CP_pitch * (0.5 * rho * A * V_wind^3);

    % Find minimal pitch whose worst-case power does not exceed rated
    idx_ok = find(P_pitch <= ratedPower, 1, 'first');
    if isempty(idx_ok)
        fprintf('\nWarning: Max pitch %.1f deg still exceeds rated power.\n', pitch_range(end));
        idx_ok = numel(pitch_range);
    end

    pitch_req = pitch_range(idx_ok);
    lambda_req = lambda_at_CP(idx_ok);
    omega_req = lambda_req * V_wind / R;
    CP_req = CP_pitch(idx_ok);
    P_req = P_pitch(idx_ok);

    fprintf('\n=== Recommended Pitch ===\n');
    fprintf('Pitch angle: %.2f deg\n', pitch_req);
    fprintf('At λ=%.2f (ω=%.1f rpm), predicted P=%.1f kW (rated=%.1f kW)\n', lambda_req, omega_req*60/(2*pi), P_req/1e3, ratedPower/1e3);

    % Output struct
    result = struct('pitch_deg', pitch_req, 'lambda', lambda_req, 'omega_rad', omega_req, ...
                    'omega_rpm', omega_req*60/(2*pi), 'P', P_req, 'CP', CP_req, ...
                    'ratedPower', ratedPower, 'V_wind', V_wind);

    % Simple visualization
    figure(4); set(gcf, 'Position', [100, 100, 600, 500]);
    plot(pitch_range, P_pitch/1e3, 'b-', 'LineWidth', 2); hold on;
    yline(ratedPower/1e3, 'r--', 'Rated');
    plot(pitch_req, P_req/1e3, 'ko', 'MarkerSize', 8, 'LineWidth', 2);
    xlabel('Pitch (deg)'); ylabel('Power (kW)'); title(sprintf('Power vs Pitch (V = %.1f m/s)', V_wind)); grid on;
end

% ============================================================================
% DELIVERABLE 5: TOWER DEFLECTION AND STRESS ANALYSIS
% ============================================================================
% Computes tower deflection and stress analysis considering distributed wind
% loading, thrust forces, and fatigue analysis with Mohrs circle and Goodman diagram.

function result = Deliverable5(lambda, pitch_deg)
% DELIVERABLE5 Tower Deflection and Stress Analysis
% Computes tower deflection and static/fatigue overloads considering:
% - Distributed wind force along tower (atmospheric boundary layer)
% - Thrust force from turbine (calculated via BEM analysis)
% - Cyclic loading for fatigue analysis
% - Mohrs circle and Goodman diagram analysis
%
% Usage: result = Deliverable5(lambda, pitch_deg);
% Inputs: lambda - tip speed ratio from Deliverable 4
%         pitch_deg - pitch angle from Deliverable 4
% Returns struct with deflection, stress, and fatigue analysis results

    % Access global configuration variables
    global PLOT_D5_DEFLECTION PLOT_D5_MOHR PLOT_D5_GOODMAN SAVE_PLOTS PARAMETERS_PATH

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
    thrust_force = calculateRotorThrust(data, V_wind, lambda, pitch_deg);
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
    % Create visualizations if enabled
    if PLOT_D5_DEFLECTION
    createTowerDeflectionPlot(deflection_results, section_props);
    end
    if PLOT_D5_MOHR
    createMohrCirclePlots(stress_results);
    end
    if PLOT_D5_GOODMAN
    createGoodmanDiagram(stress_results, S_ut, S_y, data);
    end
    
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

function thrust_force = calculateRotorThrust(data, V_wind, lambda, pitch_deg)
% CALCULATEROTORTHRUST Calculate rotor thrust force using BEM analysis
% Computes total rotor thrust force by integrating sectional thrust forces
% across all blade stations using Blade Element Momentum theory.
%
% Inputs:
%   data        - Wind turbine data structure
%   V_wind      - Wind speed [m/s]
%   lambda      - Tip speed ratio
%   pitch_deg   - Blade pitch angle [degrees]
%
% Outputs:
%   thrust_force - Total rotor thrust force [N]
    
    % Turbine parameters
    R = data.turbine.performance.rotorRadius;
    A = data.turbine.calculated.rotorArea;
    rho = data.materials.air.density;
    B = data.turbine.characteristics.blades;
    
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
% DEFINELOADCASES Define loading scenarios for fatigue analysis
% Creates two load cases for fatigue analysis: maximum and minimum loading
% scenarios with different wind speeds and directions.
%
% Inputs:
%   V_wind      - Reference wind speed [m/s]
%   thrust_force - Thrust force from rotor [N]
%
% Outputs:
%   load_cases  - Structure containing case1 and case2 load scenarios
    
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
% COMPUTETOWERSECTIONPROPERTIES Calculate section properties for each tower segment
% Computes geometric and structural properties for each tower section including
% area, moment of inertia, and section modulus.
%
% Inputs:
%   tower_specs - Tower specifications table with height, OD, and wall thickness
%
% Outputs:
%   section_props - Structure containing geometric properties for each section
    
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
% SOLVETOWERDEFLECTION Solve beam deflection using numerical integration
% Calculates tower deflection for multiple load cases using beam theory
% with proper boundary conditions and distributed loading.
%
% Inputs:
%   section_props - Tower section properties structure
%   load_cases    - Loading scenarios structure
%   E            - Youngs modulus [Pa]
%   data         - Wind turbine data structure
%
% Outputs:
%   deflection_results - Structure containing deflection and moment results
    
    % Discretize tower into elements
    n_elements = 50;
    hub_height = data.turbine.performance.hubHeight;
    z = linspace(0, hub_height, n_elements+1);  % Extend to hub height
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
% CALCULATEDEFLECTIONCASE Calculate deflection for a specific load case
% Computes beam deflection using proper beam theory with double integration
% of curvature, handling both wind loading and nacelle distributed loads.
%
% Inputs:
%   section_props - Tower section properties structure
%   load_case     - Specific loading scenario
%   z            - Height discretization array [m]
%   E            - Youngs modulus [Pa]
%   data         - Wind turbine data structure
%
% Outputs:
%   deflection   - Deflection at each height [m]
%   moment       - Bending moment at each height [N·m]
    
    n_points = length(z);
    deflection = zeros(size(z));
    moment = zeros(size(z));
    
    % Calculate wind profile and distributed loading
    [wind_profile, drag_force, nacelle_load] = computeWindLoading(section_props, load_case, z, data);
    
    % Calculate moment distribution using proper beam theory
    for i = 1:n_points
        % Wind moment (integrate from current point to tip)
        wind_moment = 0;
        for j = i:n_points-1
            if j < n_points
                wind_moment = wind_moment + drag_force(j) * (z(j+1) - z(j)) * (z(j) - z(i));
            end
        end
        
        % Nacelle distributed load moment (integrate from current point to tip)
        nacelle_moment = 0;
        for j = i:n_points-1
            if j < n_points
                nacelle_moment = nacelle_moment + nacelle_load(j) * (z(j+1) - z(j)) * (z(j) - z(i));
            end
        end
        
        moment(i) = wind_moment + nacelle_moment;
    end
    
    % Calculate deflection using proper beam theory (double integration)
    % Get moment of inertia at each point
    I = interp1(section_props.height, section_props.I, z, 'linear', 'extrap');
    
    % Calculate curvature: M/(EI)
    curvature = moment ./ (E * I);
    
    % Set curvature to zero in nacelle region (rigid, EI = infinity)
    tower_height = section_props.height(end);  % 77.7m
    curvature(z > tower_height) = 0;
    
    % Double integration to get deflection
    % First integration: slope
    slope = cumtrapz(z, curvature);
    
    % Second integration: deflection
    deflection = cumtrapz(z, slope);
    
    % Apply boundary conditions: deflection = 0 at base (z=0)
    deflection = deflection - deflection(1);
end

function [wind_profile, drag_force, nacelle_load] = computeWindLoading(section_props, load_case, z, data)
% COMPUTEWINDLOADING Compute wind profile and distributed drag force
% Calculates atmospheric boundary layer wind profile and distributed loading
% including wind drag on tower and nacelle distributed loads.
%
% Inputs:
%   section_props - Tower section properties structure
%   load_case     - Loading scenario with wind speed and thrust
%   z            - Height discretization array [m]
%   data         - Wind turbine data structure
%
% Outputs:
%   wind_profile  - Wind speed at each height [m/s]
%   drag_force    - Distributed wind drag force [N/m]
%   nacelle_load  - Distributed nacelle load [N/m]
    
    % Atmospheric boundary layer wind profile
    % Use hub height as reference for wind speed (more realistic)
    z_ref = data.turbine.performance.hubHeight;  % Hub height from data (m)
    U_ref = load_case.wind_speed;
    epsilon = 1/7;
    
    % Wind speed increases with height (realistic atmospheric boundary layer)
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
    
    % Only apply wind drag to tower region (0 to tower top)
    tower_height = section_props.height(end);  % 77.7m
    drag_force(z > tower_height) = 0;
    
    % Calculate nacelle distributed load (77.7m to hub height)
    hub_height = data.turbine.performance.hubHeight;
    nacelle_height = hub_height - tower_height;  % Height of nacelle region
    
    % Uniformly distributed load in nacelle region
    nacelle_load = zeros(size(z));
    nacelle_region = (z > tower_height) & (z <= hub_height);
    if any(nacelle_region) && nacelle_height > 0
        nacelle_load(nacelle_region) = load_case.thrust_force / nacelle_height;
    end
end

function stress_results = computeStressAnalysis(section_props, load_cases, deflection_results, E, data)
% COMPUTESTRESSANALYSIS Calculate stress analysis at tower base
% Performs stress analysis including bending, axial, and combined stresses
% with fatigue analysis using Goodman diagram methodology.
%
% Inputs:
%   section_props    - Tower section properties structure
%   load_cases       - Loading scenarios structure
%   deflection_results - Deflection analysis results
%   E               - Youngs modulus [Pa]
%   data            - Wind turbine data structure
%
% Outputs:
%   stress_results  - Structure containing stress analysis results
    
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
% CREATETOWERDEFLECTIONPLOT Create tower deflection visualization
% Generates plots showing tower deflection profiles for different load cases
% with proper scaling and formatting.
%
% Inputs:
%   deflection_results - Deflection analysis results structure
%   section_props      - Tower section properties structure
    
    % Access global configuration variables
    global SAVE_PLOTS
    
    figure(5); set(gcf, 'Position', [100, 100, 800, 400]);
    
    plot(deflection_results.z, deflection_results.case1.deflection, 'b-', 'LineWidth', 2);
    hold on;
    plot(deflection_results.z, deflection_results.case2.deflection, 'r--', 'LineWidth', 2);
    xlabel('Height (m)');
    ylabel('Deflection (m)');
    title('Tower Deflection Profile');
    legend('Load Case 1 (Max)', 'Load Case 2 (Min)', 'Location', 'best');
    grid on;
    
    % Save plot if enabled
    if SAVE_PLOTS
    saveas(gcf, 'Tower_Deflection_Analysis.png');
    fprintf('Tower deflection plot saved as Tower_Deflection_Analysis.png\n');
    end
end

function createMohrCirclePlots(stress_results)
% CREATEMOHRCIRCLEPLOTS Create Mohrs circle visualizations
% Generates Mohrs circle plots for stress state analysis showing principal
% stresses and maximum shear stress for different load cases.
%
% Inputs:
%   stress_results - Stress analysis results structure
    
    % Access global configuration variables
    global SAVE_PLOTS
    
    figure(6); set(gcf, 'Position', [200, 200, 1200, 500]);
    
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
    
    % Save plot if enabled
    if SAVE_PLOTS
    saveas(gcf, 'Mohr_Circle_Analysis.png');
    fprintf('Mohr circle plots saved as Mohr_Circle_Analysis.png\n');
    end
end

function plotMohrCircle(sigma_x, sigma_y, tau_xy, title_str)
% PLOTMOHRCIRCLE Plot Mohrs circle for given stress state
% Creates Mohrs circle visualization for a specific stress state showing
% principal stresses, maximum shear stress, and stress components.
%
% Inputs:
%   sigma_x     - Normal stress in x-direction [Pa]
%   sigma_y     - Normal stress in y-direction [Pa]
%   tau_xy      - Shear stress [Pa]
%   title_str   - Title string for the plot
    
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
% CREATEGOODMANDIAGRAM Create Goodman diagram for fatigue analysis
% Generates Goodman diagram showing fatigue safety factors, endurance limits,
% and operating points for fatigue analysis of the tower structure.
%
% Inputs:
%   stress_results - Stress analysis results structure
%   S_ut          - Ultimate tensile strength [Pa]
%   S_y           - Yield strength [Pa]
%   data          - Wind turbine data structure
    
    % Access global configuration variables
    global SAVE_PLOTS
    
    figure(7); set(gcf, 'Position', [300, 300, 800, 600]);
    
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
    
    % Save plot if enabled
    if SAVE_PLOTS
    saveas(gcf, 'Goodman_Diagram_Analysis.png');
    fprintf('Goodman diagram saved as Goodman_Diagram_Analysis.png\n');
    end
end



function data = ParameterExtraction()
% PARAMETEREXTRACTION Extracts all data from the Given Parameters folder
% and organizes it into a structured format for wind turbine analysis.
%
% OUTPUTS:
%   data - Structure containing all extracted parameters organized by category
%
% The function extracts:
%   - Blade profile data (geometry, twist, airfoil assignments)
%   - Tower specifications (height, diameter, wall thickness)
%   - Airfoil coordinate data (for different airfoil types)
%   - Airfoil performance data (CL, CD, CM vs AoA)
%   - Material properties (air and steel)
%   - Wind turbine specifications (Clipper Liberty C96)
%
% Usage: data = ParameterExtraction();

    % Access global configuration variables
    global PARAMETERS_PATH

    % Define the path to the Given Parameters folder
    basePath = PARAMETERS_PATH;
    
    % Initialize the main data structure
    data = struct();
    
    try
        % Extract blade profile data
        data.blade = extractBladeProfile(fullfile(basePath, 'BladeProfile.csv'));
        
        % Extract tower specifications
        data.tower = extractTowerSpecs(fullfile(basePath, 'towerSpecs.csv'));
        
        % Extract airfoil coordinate data
        data.airfoils = extractAirfoilCoordinates(basePath);
        
        % Extract airfoil performance data
        data.airfoilPerformance = extractAirfoilPerformance(basePath);
        
        % Add material properties
        data.materials = extractMaterialProperties();
        
        % Add wind turbine specifications
        data.turbine = extractTurbineSpecifications();

        % Add predefined parameters for deliverables
        data.deliverables = struct();
        data.deliverables.part1 = struct();
        data.deliverables.part1.V_wind = 10; % m/s
        data.deliverables.part1.omega_rpm = 14; % rpm
        data.deliverables.part1.pitch_deg = 0; % degrees
        
        data.deliverables.part2 = struct();
        data.deliverables.part2.V_wind = 8; % m/s
        data.deliverables.part2.lambda = 6.91; % tip speed ratio
        
        data.deliverables.part3 = struct();
        data.deliverables.part3.V_wind = 6; % m/s
        
        data.deliverables.part4 = struct();
        data.deliverables.part4.V_wind = 14.6; % m/s
        
        
        % Add metadata
        data.metadata = struct();
        data.metadata.extractionDate = datetime('now');
        data.metadata.sourceFolder = basePath;
        data.metadata.description = 'Wind turbine parameters extracted from Given Parameters folder';

        
        
    catch ME
        error('Parameter extraction failed: %s', ME.message); %
    end
end



function bladeData = extractBladeProfile(filePath)
% EXTRACTBLADEPROFILE Extract blade profile data from CSV file
% Reads blade geometry data including chord length, twist angle, and airfoil
% assignments from CSV file and organizes into structured format.
%
% Inputs:
%   filePath    - Path to blade profile CSV file
%
% Outputs:
%   bladeData   - Structure containing blade profile data and metadata
    
    % Read the CSV file
    opts = detectImportOptions(filePath);
    bladeTable = readtable(filePath, opts);
    
    % Store the data
    bladeData = struct();
    bladeData.profile = bladeTable;
    bladeData.description = 'Blade profile data including geometry, twist, and airfoil assignments';
    
    % Extract unique airfoil types
    bladeData.airfoilTypes = unique(bladeTable.Airfoil);
    
    % Calculate additional parameters
    bladeData.totalLength = max(bladeTable.DistanceFromCenterOfRotation);
    bladeData.stations = height(bladeTable);
    
end

function towerData = extractTowerSpecs(filePath)
% EXTRACTTOWERSPECS Extract tower specification data from CSV file
% Reads tower geometry data including height, outer diameter, and wall thickness
% from CSV file and calculates additional structural parameters.
%
% Inputs:
%   filePath    - Path to tower specifications CSV file
%
% Outputs:
%   towerData   - Structure containing tower specifications and calculated parameters
    
    % Read the CSV file
    opts = detectImportOptions(filePath);
    towerTable = readtable(filePath, opts);
    
    % Store the data
    towerData = struct();
    towerData.specs = towerTable;
    towerData.description = 'Tower specifications including height, diameter, and wall thickness';
    towerData.dragCoefficient = 0.7; % Drag coefficient for cylindrical tower
    
    % Calculate additional parameters
    towerData.totalHeight = max(towerTable.Height_mm_) / 1000; % Convert to meters
    towerData.sections = height(towerTable);
    towerData.baseDiameter = towerTable.OD_mm_(1) / 1000; % Convert to meters
    towerData.topDiameter = towerTable.OD_mm_(end) / 1000; % Convert to meters
    
end

function airfoilData = extractAirfoilCoordinates(basePath)
% EXTRACTAIRFOILCOORDINATES Extract airfoil coordinate data from .dat files
% Reads airfoil coordinate data from .dat files and organizes into structured
% format for use in aerodynamic calculations and visualizations.
%
% Inputs:
%   basePath    - Base directory path containing airfoil .dat files
%
% Outputs:
%   airfoilData - Structure containing coordinate data for each airfoil
    
    airfoilData = struct();
    
    % List of airfoil coordinate files
    airfoilFiles = {'180.dat', '210.dat', '250.dat', '300.dat'};
    airfoilNames = {'DU96_W_180', 'DU93_W_210', 'DU91_W2_250', 'DU97_W_300'};
    
    for i = 1:length(airfoilFiles)
        filePath = fullfile(basePath, airfoilFiles{i});
        
        if exist(filePath, 'file')
            try
                % Read the .dat file (skip header lines starting with %)
                fid = fopen(filePath, 'r');
                lines = {};
                line = fgetl(fid);
                while ischar(line)
                    if ~startsWith(strtrim(line), '%')
                        lines{end+1} = line;
                    end
                    line = fgetl(fid);
                end
                fclose(fid);
                
                % Parse the coordinate data
                coords = zeros(length(lines), 2);
                for j = 1:length(lines)
                    coords(j, :) = str2num(lines{j});
                end
                
                % Store in structure
                airfoilData.(airfoilNames{i}) = struct();
                airfoilData.(airfoilNames{i}).x = coords(:, 1);
                airfoilData.(airfoilNames{i}).y = coords(:, 2);
                airfoilData.(airfoilNames{i}).description = sprintf('Airfoil coordinates for %s', airfoilNames{i});
                airfoilData.(airfoilNames{i}).numPoints = length(coords);
                
            catch ME
                warning('Failed to read airfoil file %s: %s', airfoilFiles{i}, ME.message);
            end
        end
    end
    
end

function performanceData = extractAirfoilPerformance(basePath)
% EXTRACTAIRFOILPERFORMANCE Extract airfoil performance data from CSV files
% Reads airfoil performance data including lift, drag, and moment coefficients
% versus angle of attack from CSV files for BEM analysis.
%
% Inputs:
%   basePath        - Base directory path containing airfoil performance CSV files
%
% Outputs:
%   performanceData - Structure containing performance data for each airfoil
    
    performanceData = struct();
    
    % List of airfoil performance files
    perfFiles = {'DU96-W-180.csv', 'DU93-W-210.csv', 'DU91-W2-250.csv', 'DU97-W-300.csv'};
    perfNames = {'DU96_W_180', 'DU93_W_210', 'DU91_W2_250', 'DU97_W_300'};
    
    for i = 1:length(perfFiles)
        filePath = fullfile(basePath, perfFiles{i});
        
        if exist(filePath, 'file')
            try
                % Read the CSV file
                opts = detectImportOptions(filePath);
                perfTable = readtable(filePath, opts);
                
                % Store in structure
                performanceData.(perfNames{i}) = struct();
                performanceData.(perfNames{i}).data = perfTable;
                performanceData.(perfNames{i}).AoA = perfTable.AoA;
                performanceData.(perfNames{i}).CL = perfTable.CL;
                performanceData.(perfNames{i}).CD = perfTable.CD;
                performanceData.(perfNames{i}).CM = perfTable.CM;
                performanceData.(perfNames{i}).description = sprintf('Performance data for %s', perfNames{i});
                performanceData.(perfNames{i}).numPoints = height(perfTable);
                
                % Calculate additional parameters
                performanceData.(perfNames{i}).maxCL = max(perfTable.CL);
                performanceData.(perfNames{i}).minCD = min(perfTable.CD);
                performanceData.(perfNames{i}).AoARange = [min(perfTable.AoA), max(perfTable.AoA)];
                
            catch ME
                warning('Failed to read performance file %s: %s', perfFiles{i}, ME.message);
            end
        end
    end
    

end

function materialData = extractMaterialProperties()
% EXTRACTMATERIALPROPERTIES Extract material properties for air and steel
% Defines material properties for air (aerodynamic calculations) and steel
% (structural calculations) with comprehensive property sets and units.
%
% Outputs:
%   materialData - Structure containing air and steel material properties
    
    materialData = struct();
    
    % Air properties
    materialData.air = struct();
    materialData.air.density = 1.1; % kg/m³
    materialData.air.viscosity = 1.8e-5; % Ns/m²
    materialData.air.description = 'Air properties for aerodynamic calculations';
    materialData.air.units = struct('density', 'kg/m³', 'viscosity', 'Ns/m²');
    
    % Steel properties (ASTM A572, Grade 50)
    materialData.steel = struct();
    materialData.steel.type = 'ASTM A572, Grade 50';
    materialData.steel.density = 7850; % kg/m³
    materialData.steel.tensileStrength = 450e6; % Pa (450 MPa)
    materialData.steel.yieldStrength = 345e6; % Pa (345 MPa)
    materialData.steel.youngsModulus = 200e9; % Pa (200 GPa)
    materialData.steel.description = 'Steel properties for structural calculations';
    materialData.steel.units = struct('density', 'kg/m³', 'tensileStrength', 'Pa', 'yieldStrength', 'Pa', 'youngsModulus', 'Pa');
    
    % Calculate additional steel properties
    materialData.steel.safetyFactor = 1.5; % Typical safety factor
    materialData.steel.allowableStress = materialData.steel.yieldStrength / materialData.steel.safetyFactor;
    materialData.steel.enduranceLimitFactor = 0.5; % Factor for endurance limit calculation
end

function turbineData = extractTurbineSpecifications()
% EXTRACTTURBINESPECIFICATIONS Extract wind turbine specifications
% Defines comprehensive wind turbine specifications for Clipper Liberty C96
% including performance characteristics, dimensions, and calculated parameters.
%
% Outputs:
%   turbineData - Structure containing complete turbine specifications
    
    turbineData = struct();
    
    % Basic turbine information
    turbineData.model = 'Clipper Liberty C96 2.5 MW';
    turbineData.location = 'Rosemount, MN';
    turbineData.description = 'UMN Clipper C96 Wind Turbine specifications';
    
    % Turbine characteristics
    turbineData.characteristics = struct();
    turbineData.characteristics.blades = 3;
    turbineData.characteristics.orientation = 'upwind';
    turbineData.characteristics.control = 'pitch controlled, variable speed';
    turbineData.characteristics.yawControl = true;
    turbineData.characteristics.yawAngle = 0; % degrees (for analysis)
    
    % Performance specifications
    turbineData.performance = struct();
    turbineData.performance.ratedPower = 2.5e6; % W (2.5 MW)
    turbineData.performance.bladeRadius = 48; % m
    turbineData.performance.rotorRadius = 48; % m (same as blade radius)
    turbineData.performance.hubHeight = 80.4; % m
    turbineData.performance.hubRadius = 1.3; % m
    turbineData.performance.cutInSpeed = 4; % m/s
    turbineData.performance.ratedSpeed = 11; % m/s
    turbineData.performance.cutOutSpeed = 25; % m/s
    turbineData.performance.rotorSpeedRange = [9.6, 15.5]; % RPM [min, max]
    
    % Calculated parameters
    turbineData.calculated = struct();
    turbineData.calculated.rotorArea = pi * turbineData.performance.rotorRadius^2; % m²
    turbineData.calculated.sweptArea = turbineData.calculated.rotorArea; % m²
    turbineData.calculated.powerDensity = turbineData.performance.ratedPower / turbineData.calculated.rotorArea; % W/m²
    turbineData.calculated.rotorSpeedRange_rads = turbineData.performance.rotorSpeedRange * 2 * pi / 60; % rad/s
    
    % Units information
    turbineData.units = struct();
    turbineData.units.power = 'W';
    turbineData.units.length = 'm';
    turbineData.units.speed = 'm/s';
    turbineData.units.rotorSpeed = 'RPM';
    turbineData.units.angle = 'degrees';
    
end

% ============================================================================
% COMMON FUNCTIONS
% ============================================================================
% Consolidated helper functions used across multiple deliverables
% 
% BEM ANALYSIS FUNCTIONS:
%   solveBEMSection() - BEM analysis for airfoil sections
%   solveCircleSection() - BEM analysis for circular sections  
%   getSectionCoefficients() - Unified section coefficient calculation
%   cylinderCDlocal() - Drag coefficient for circular cylinders
%
% TOWER ANALYSIS FUNCTIONS:
%   calculateRotorThrust() - Calculate rotor thrust force
%   defineLoadCases() - Define loading scenarios
%   computeTowerSectionProperties() - Calculate tower section properties
%   solveTowerDeflection() - Solve beam deflection equations
%   calculateDeflectionCase() - Calculate deflection for specific load case
%   computeWindLoading() - Calculate distributed wind loads
%   computeStressAnalysis() - Perform stress and fatigue analysis
%
% DATA EXTRACTION FUNCTIONS:
%   ParameterExtraction() - Main data extraction function
%   extractBladeProfile() - Extract blade geometry data
%   extractTowerSpecs() - Extract tower specifications
%   extractAirfoilCoordinates() - Extract airfoil coordinate data
%   extractAirfoilPerformance() - Extract airfoil performance data
%   extractMaterialProperties() - Extract material properties
%   extractTurbineSpecifications() - Extract turbine specifications
%
% VISUALIZATION FUNCTIONS:
%   createVisualization() - D1 results visualization
%   createPitchOptimizationPlot() - D2 optimization plot
%   create2DOptimizationPlot() - D3 optimization plot
%   createTowerDeflectionPlot() - D5 deflection plot
%   createMohrCirclePlots() - D5 Mohr circle plots
%   createGoodmanDiagram() - D5 Goodman diagram
%   plotMohrCircle() - Helper for Mohr circle plotting
% ============================================================================

function [a, a_prime, CL, CD, Cn, Ct, V_rel] = solveBEMSection(r, c, twist_rad, perf_data, lambda_r, V_wind, omega_rad, pitch_rad)
% SOLVEBEMSECTION Closed-form BEM analysis for airfoil sections
% Calculates induction factors and sectional loads using Blade Element Momentum theory
% without iteration for improved computational efficiency.
%
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
% SOLVECIRCLESECTION BEM analysis for circular sections
% Calculates induction factors and loads for circular (non-lifting) sections
% with zero lift coefficient and Reynolds-dependent drag coefficient.
%
% Inputs:
%   r           - radial position [m]
%   c           - local chord (diameter) [m]
%   twist_rad   - local geometric twist [rad]
%   lambda_r    - local tip-speed ratio (λ * r / R)
%   V_wind      - freestream wind speed [m/s]
%   omega_rad   - rotor speed [rad/s]
%   pitch_rad   - blade pitch angle [rad]
%   rho         - air density [kg/m³]
%   mu          - air dynamic viscosity [Pa·s]
%
% Outputs:
%   a, a_prime  - axial and tangential induction factors
%   CL, CD      - lift and drag coefficients
%   Cn, Ct      - normal and tangential force coefficients
%   V_rel       - relative velocity [m/s]
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
% CYLINDERCDLOCAL Drag coefficient for circular cylinders
% Calculates drag coefficient for smooth circular cylinders in cross-flow
% using empirical correlations based on Reynolds number.
%
% Inputs:
%   Re          - Reynolds number based on cylinder diameter
%
% Outputs:
%   C_D         - Drag coefficient
    if Re < 2e5
        C_D = 11 * Re.^(-0.75) + 0.9 * (1.0 - exp(-1000./Re)) + 1.2 * (1.0 - exp(-(Re./4500).^0.7));
    elseif Re <= 5e5
        C_D = 10.^(0.32*tanh(44.4504 - 8 * log10(Re)) - 0.238793158);
    else
        C_D = 0.1 * log10(Re) - 0.2533429;
    end
end

function [a, a_prime, CL, CD, Cn, Ct, V_rel] = getSectionCoefficients(airfoil, twist_deg, r, c, lambda_r, V_wind, omega_rad, pitch_rad, data, rho, mu, varargin)
% GETSECTIONCOEFFICIENTS Unified section coefficient calculation
% Calculates BEM coefficients for airfoil or circular sections based on airfoil type.
% Automatically handles both airfoil and circular section calculations.
%
% Inputs:
%   airfoil     - Airfoil name or 'Circle' for circular sections
%   twist_deg   - Local geometric twist [degrees]
%   r           - Radial position [m]
%   c           - Local chord [m]
%   lambda_r    - Local tip-speed ratio
%   V_wind      - Freestream wind speed [m/s]
%   omega_rad   - Rotor speed [rad/s]
%   pitch_rad   - Blade pitch angle [rad]
%   data        - Wind turbine data structure
%   rho         - Air density [kg/m³]
%   mu          - Air dynamic viscosity [Pa·s]
%   varargin    - Optional parameters (B, R for future use)
%
% Outputs:
%   a, a_prime  - Axial and tangential induction factors
%   CL, CD      - Lift and drag coefficients
%   Cn, Ct      - Normal and tangential force coefficients
%   V_rel       - Relative velocity [m/s]

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
% CREATEVISUALIZATION Create visualization of BEM analysis results
% Generates comprehensive plots showing power and thrust distributions,
% sectional contributions, and performance metrics.
%
% Inputs:
%   r_stations  - Radial station positions [m]
%   dCP         - Differential power coefficient distribution
%   dCT         - Differential thrust coefficient distribution
%   CP          - Total power coefficient
%   CT          - Total thrust coefficient
%   lambda      - Tip speed ratio
%   V_wind      - Wind speed [m/s]
%   omega       - Rotor speed [rad/s]

    % Access global configuration variables
    global SAVE_PLOTS
    
    figure(1); set(gcf, 'Position', [100, 100, 1200, 800]);
    
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
    % Save plot if enabled
    if SAVE_PLOTS
        saveas(gcf, 'Deliverable1_Results.png');
        fprintf('Results visualization saved as Deliverable1_Results.png\n');
    end
end

function createPitchOptimizationPlot(pitch_range, CP_values, CT_values, optimal_pitch, CP_max, V_wind, lambda)
% CREATEPITCHOPTIMIZATIONPLOT Create visualization of CP and CT vs Pitch Angle
% Generates comprehensive plots showing power and thrust coefficients versus
% pitch angle with optimal point highlighting and dual y-axis formatting.
%
% Inputs:
%   pitch_range  - Array of pitch angles [degrees]
%   CP_values    - Power coefficient values
%   CT_values    - Thrust coefficient values
%   optimal_pitch - Optimal pitch angle [degrees]
%   CP_max       - Maximum power coefficient
%   V_wind       - Wind speed [m/s]
%   lambda       - Tip speed ratio

    % Access global configuration variables
    global SAVE_PLOTS
    
    figure(2); set(gcf, 'Position', [100, 100, 1000, 700]);
    
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
    
    % Save plot if enabled
    if SAVE_PLOTS
        saveas(gcf, 'Pitch_Optimization_Results.png');
        fprintf('Pitch optimization results visualization saved as Pitch_Optimization_Results.png\n');
    end
end

function create2DOptimizationPlot(lambda_range, pitch_range, CP_matrix, CT_matrix, optimal_lambda, optimal_pitch, CP_max, V_wind)
% CREATE2DOPTIMIZATIONPLOT Create 2D visualization of CP and CT optimization
% Generates comprehensive 2D plots including contour plots, surface plots,
% and results summary for tip speed ratio and pitch angle optimization.
%
% Inputs:
%   lambda_range   - Array of tip speed ratios
%   pitch_range    - Array of pitch angles [degrees]
%   CP_matrix      - Power coefficient matrix [lambda x pitch]
%   CT_matrix      - Thrust coefficient matrix [lambda x pitch]
%   optimal_lambda - Optimal tip speed ratio
%   optimal_pitch  - Optimal pitch angle [degrees]
%   CP_max         - Maximum power coefficient
%   V_wind         - Wind speed [m/s]

    % Access global configuration variables
    global SAVE_PLOTS
    
    figure(3); set(gcf, 'Position', [100, 100, 1200, 800]);
    
    % Subplot 1: CP contour plot
    subplot(2, 2, 1);
    [LAMBDA, PITCH] = meshgrid(lambda_range, pitch_range);
    contourf(LAMBDA, PITCH, CP_matrix, 20);
    colorbar;
    xlabel('Tip Speed Ratio (\lambda)');
    ylabel('Pitch Angle (degrees)');
    title('Power Coefficient (C_P) Contour');
    hold on;
    plot(optimal_lambda, optimal_pitch, 'wo', 'MarkerSize', 10, 'LineWidth', 2, 'MarkerFaceColor', 'red');
    hold off;
    
    % Subplot 2: CT contour plot
    subplot(2, 2, 2);
    contourf(LAMBDA, PITCH, CT_matrix, 20);
    colorbar;
    xlabel('Tip Speed Ratio (\lambda)');
    ylabel('Pitch Angle (degrees)');
    title('Thrust Coefficient (C_T) Contour');
    hold on;
    plot(optimal_lambda, optimal_pitch, 'wo', 'MarkerSize', 10, 'LineWidth', 2, 'MarkerFaceColor', 'red');
    hold off;
    
    % Subplot 3: CP surface plot
    subplot(2, 2, 3);
    surf(LAMBDA, PITCH, CP_matrix);
    xlabel('Tip Speed Ratio (\lambda)');
    ylabel('Pitch Angle (degrees)');
    zlabel('Power Coefficient (C_P)');
    title('Power Coefficient Surface');
    shading interp;
    colorbar;
    
    % Subplot 4: Results summary
    subplot(2, 2, 4);
    text(0.1, 0.8, sprintf('Wind Speed: %.1f m/s', V_wind), 'FontSize', 14, 'FontWeight', 'bold');
    text(0.1, 0.7, sprintf('Max C_P: %.4f', CP_max), 'FontSize', 14, 'FontWeight', 'bold', 'Color', 'blue');
    text(0.1, 0.6, sprintf('Optimal λ: %.3f', optimal_lambda), 'FontSize', 12);
    text(0.1, 0.5, sprintf('Optimal θ: %.1f°', optimal_pitch), 'FontSize', 12);
    text(0.1, 0.4, sprintf('Optimal RPM: %.1f', optimal_lambda * V_wind / 50 * 60 / (2*pi)), 'FontSize', 12);
    axis off;
    title('2D Optimization Results');
    
    sgtitle('Wind Turbine 2D Performance Optimization', 'FontSize', 16, 'FontWeight', 'bold');
    
    % Save plot if enabled
    if SAVE_PLOTS
        saveas(gcf, '2D_CP_Optimization_Results.png');
        fprintf('2D optimization results visualization saved as 2D_CP_Optimization_Results.png\n');
    end
end


