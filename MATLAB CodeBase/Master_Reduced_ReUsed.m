% ============================================================================
% MASTER WIND TURBINE ANALYSIS SCRIPT
% ============================================================================
% 
% Comprehensive wind turbine analysis tool consolidating all deliverables into a
% single executable script with structured configuration system.
%
% DELIVERABLES:
%   Deliverable1() - Basic BEM Analysis (CP, CT calculation)
%   Deliverable2() - Pitch Optimization (find optimal pitch angle)
%   Deliverable3() - 2D CP Optimization (lambda and pitch optimization)
%   Deliverable4() - Rated Power Pitch Control (pitch for rated power)
%   Deliverable5() - Tower Deflection and Stress Analysis (structural analysis)
%
% USAGE:
%   1. Edit configuration in createConfig() function
%   2. Hit Run button or press F5 to execute
%   3. Or run individual deliverables: Deliverable1(), Deliverable2(), etc.

% ============================================================================

%% ============================================================================
%% UTILITY FUNCTIONS
%% ============================================================================

function rad = rpm2rad(rpm)
% RPM2RAD Convert RPM to rad/s
% Converts rotational speed from revolutions per minute to radians per second.
%
% Syntax:
%   rad = rpm2rad(rpm)
%
% Input Arguments:
%   rpm - Rotational speed [RPM]
%
% Output Arguments:
%   rad - Rotational speed [rad/s]
%
% Example:
%   omega_rad = rpm2rad(14);  % Convert 14 RPM to rad/s

    rad = rpm * 2 * pi / 60;  % Convert RPM to rad/s
end

function rpm = rad2rpm(rad)
% RAD2RPM Convert rad/s to RPM
% Converts rotational speed from radians per second to revolutions per minute.
%
% Syntax:
%   rpm = rad2rpm(rad)
%
% Input Arguments:
%   rad - Rotational speed [rad/s]
%
% Output Arguments:
%   rpm - Rotational speed [RPM]
%
% Example:
%   omega_rpm = rad2rpm(1.47);  % Convert 1.47 rad/s to RPM

    rpm = rad * 60 / (2 * pi);  % Convert rad/s to RPM
end

function config = createConfig() % USED FOR CHOOSING OUTPUT PLOTS 
% CREATECONFIG Create configuration structure
% Returns configuration structure with all analysis settings, plotting options,
% and visual formatting parameters for wind turbine analysis.
%
% Syntax:
%   config = createConfig()
%
% Output Arguments:
%   config - Configuration structure containing:
%     .run_deliverable_* - Execution flags for each deliverable
%     .plot_d*_*        - Plotting flags for visualizations
%     .save_plots       - Flag to save plots as PNG files
%     .color_*          - Color scheme for plots
%     .parameters_path  - Path to parameter files
%
% Description:
%   Creates comprehensive configuration structure with all settings needed
%   for wind turbine analysis including execution flags, plotting options,
%   and visual formatting parameters.
%
% Example:
%   config = createConfig();
%   config.plot_d1_results = true;  % Enable plotting for Deliverable 1

    config = struct();
    
    % ============================================================================
    % DELIVERABLE EXECUTION CONTROL
    % ============================================================================
    % Set to true to run each deliverable, false to skip
    % Dependencies: D5 requires D4 results for realistic operating conditions
    config.run_deliverable_1 =      true;    % Basic BEM Analysis (CP, CT calculation)
    config.run_deliverable_2 =      true;    % Pitch Optimization (optimal pitch finding)
    config.run_deliverable_3 =      true;    % 2D CP Optimization (lambda & pitch optimization)
    config.run_deliverable_4 =      true;    % Rated Power Pitch Control (pitch for 2.5 MW)
    config.run_deliverable_5 =      true;    % Tower Structural Analysis (deflection & stress)
    
    % ============================================================================
    % VISUALIZATION CONTROL
    % ============================================================================
    % Enable/disable specific plot generation for each deliverable
    % Set to true to generate plots, false to skip (saves computation time)
    config.plot_d1_results =        true;    % D1: Side-by-side CP/CT distribution plots
    config.plot_d2_optimization =   true;    % D2: CP/CT vs pitch angle optimization
    config.plot_d3_optimization =   true;    % D3: 3D surface plot of CP optimization
    config.plot_d4_power =          true;    % D4: Power vs pitch for rated power control
    config.plot_d5_deflection =     true;    % D5: Tower deflection profile plots
    config.plot_d5_mohr =           true;    % D5: Mohrs circle stress analysis
    config.plot_d5_goodman =        true;    % D5: Goodman diagram fatigue analysis
    config.plot_d5_tower =          true;    % D5: Comprehensive tower analysis plots
    
    % ============================================================================
    % OUTPUT AND FILE MANAGEMENT
    % ============================================================================
    config.save_plots = false;          % Save all generated plots as PNG files
    config.parameters_path = 'Auxilary Information/Given Parameters/'; % Path to input data files
    
    % ============================================================================
    % VISUAL STYLING
    % ============================================================================
    % Consistent color scheme for all plots
    config.color_primary = 'b';          % Primary color (blue) - main data series
    config.color_secondary = 'r';        % Secondary color (red) - comparison data
    config.color_accent = 'g';           % Accent color (green) - highlights/annotations
end

%% ============================================================================
%% BEM HELPER FUNCTIONS
%% ============================================================================

function blade_stations = prepareBladeStations(data)
% PREPAREBLADESTATIONS Extract and prepare blade station data
% Extracts blade geometry data from profile table and filters out hub region
% to prepare station data for BEM analysis.
%
% Syntax:
%   blade_stations = prepareBladeStations(data)
%
% Input Arguments:
%   data - Wind turbine data structure containing blade profile
%
% Output Arguments:
%   blade_stations - Structure containing blade station data:
%     .r_stations     - Radial positions [m]
%     .chord_m        - Chord lengths [m]
%     .twist_deg_vec  - Twist angles [degrees]
%     .airfoil_raw    - Airfoil names for each station
%     .n_stations     - Number of stations
%
% Description:
%   Extracts blade geometry from profile data, converts units from mm to m,
%   and filters out stations within the hub radius to prepare clean data
%   for BEM analysis calculations.
%
% Example:
%   blade_stations = prepareBladeStations(data);
%   fprintf('Number of stations: %d\n', blade_stations.n_stations);

    % ============================================================================
    % BLADE GEOMETRY EXTRACTION AND PREPROCESSING
    % ============================================================================
    % Extract blade profile data from input structure
    blade_profile = data.blade.profile;  % Complete blade profile table
    hub_radius = data.turbine.performance.hubRadius;  % Hub radius [m] - filter boundary
    
    % ============================================================================
    % HUB REGION FILTERING
    % ============================================================================
    % Filter out stations within hub radius (where BEM theory doesnt apply)
    % Hub region has complex 3D flow and no meaningful aerodynamic analysis
    use_idx = blade_profile.DistanceFromCenterOfRotation / 1000 >= hub_radius;  % Boolean filter
    
    % ============================================================================
    % UNIT CONVERSION AND DATA ORGANIZATION
    % ============================================================================
    % Convert from mm to m and organize into structure for BEM analysis
    blade_stations.r_stations = blade_profile.DistanceFromCenterOfRotation(use_idx) / 1000;  % Radial positions [m]
    blade_stations.chord_m = blade_profile.ChordLength(use_idx) / 1000;  % Chord lengths [m]
    blade_stations.twist_deg_vec = blade_profile.BladeTwist(use_idx);  % Twist angles [deg]
    blade_stations.airfoil_raw = blade_profile.Airfoil(use_idx);  % Airfoil names for each station
    blade_stations.n_stations = sum(use_idx);  % Number of valid stations for analysis
end

function [dT, dQ, dP] = calculateBEMAtStation(station_data, conditions, data)
% CALCULATEBEMATSTATION Calculate BEM coefficients for a single station
% Calculates elemental thrust, torque, and power for a single blade station
% using Blade Element Momentum theory.
%
% Syntax:
%   [dT, dQ, dP] = calculateBEMAtStation(station_data, conditions, data)
%
% Input Arguments:
%   station_data - Structure containing station geometry:
%     .airfoil    - Airfoil name
%     .twist_deg  - Twist angle [degrees]
%     .r          - Radial position [m]
%     .c          - Chord length [m]
%   conditions   - Structure containing operating conditions:
%     .lambda_r   - Local tip speed ratio
%     .V_wind     - Wind speed [m/s]
%     .omega_rad  - Rotor speed [rad/s]
%     .pitch_rad  - Pitch angle [rad]
%   data         - Wind turbine data structure
%
% Output Arguments:
%   dT - Elemental thrust force [N]
%   dQ - Elemental torque [N·m]
%   dP - Elemental power [W]
%
% Description:
%   Calculates elemental loads for a single blade station using BEM theory.
%   Integrates aerodynamic coefficients with local geometry and operating
%   conditions to determine thrust, torque, and power contributions.
%
% Example:
%   [dT, dQ, dP] = calculateBEMAtStation(station_data, conditions, data);

    % ============================================================================
    % BLADE ELEMENT MOMENTUM (BEM) ANALYSIS
    % ============================================================================
    % Calculate aerodynamic coefficients using BEM theory
    % Returns: Cn (normal force coefficient), Ct (tangential force coefficient), V_rel (relative velocity)
    [~, ~, ~, ~, Cn, Ct, V_rel] = getSectionCoefficients(...
        station_data.airfoil, station_data.twist_deg, station_data.r, station_data.c, ...
        conditions.lambda_r, conditions.V_wind, conditions.omega_rad, conditions.pitch_rad, ...
        data, data.materials.air.density, data.materials.air.viscosity);
    
    % Calculate elemental loads using aerodynamic coefficients
    % Thrust: dT = 0.5 * ρ * V_rel² * c * Cn
    dT = 0.5 * data.materials.air.density * V_rel^2 * station_data.c * Cn;  % Elemental thrust [N]
    
    % Torque: dQ = 0.5 * ρ * V_rel² * c * Ct * r
    dQ = 0.5 * data.materials.air.density * V_rel^2 * station_data.c * Ct * station_data.r;  % Elemental torque [N·m]
    
    % Power: dP = dQ * ω (torque times angular velocity)
    dP = dQ * conditions.omega_rad;  % Elemental power [W]
end

function [CP, CT, P, T] = integrateBladeLoads(r_stations, dT, dQ, omega_rad, B, A, V_wind, rho)
% INTEGRATEBLADELOADS Integrate blade loads to get total coefficients
% Integrates elemental loads across blade stations to calculate total
% power and thrust coefficients and forces.
%
% Syntax:
%   [CP, CT, P, T] = integrateBladeLoads(r_stations, dT, dQ, omega_rad, B, A, V_wind, rho)
%
% Input Arguments:
%   r_stations - Radial positions of blade stations [m]
%   dT         - Elemental thrust forces [N]
%   dQ         - Elemental torques [N·m]
%   omega_rad  - Rotor angular velocity [rad/s]
%   B          - Number of blades
%   A          - Rotor swept area [m²]
%   V_wind     - Wind speed [m/s]
%   rho        - Air density [kg/m³]
%
% Output Arguments:
%   CP - Power coefficient
%   CT - Thrust coefficient
%   P  - Total power [W]
%   T  - Total thrust [N]
%
% Description:
%   Integrates elemental loads across all blade stations using trapezoidal
%   integration to calculate total rotor performance metrics.
%
% Example:
%   [CP, CT, P, T] = integrateBladeLoads(r_stations, dT, dQ, omega, B, A, V_wind, rho);

    % ============================================================================
    % INTEGRATION OF BLADE LOADS TO TOTAL ROTOR PERFORMANCE
    % ============================================================================
    % Integrate elemental loads across all blade stations using trapezoidal integration
    % Multiply by number of blades (B) to get total rotor loads
    
    T = B * trapz(r_stations, dT);  % Total thrust [N] - sum of all elemental thrust forces
    P = B * trapz(r_stations, dQ * omega_rad);  % Total power [W] - sum of all elemental power contributions
    
    % Calculate dimensionless performance coefficients
    % Power coefficient: CP = P / (0.5 * ρ * A * V³)
    CP = P / (0.5 * rho * A * V_wind^3);  % Power coefficient
    
    % Thrust coefficient: CT = T / (0.5 * ρ * A * V²)
    CT = T / (0.5 * rho * A * V_wind^2);  % Thrust coefficient
end

function [a, a_prime, CL, CD, Cn, Ct, V_rel] = solveBEMSection(r, ~, twist_rad, perf_data, lambda_r, V_wind, omega_rad, pitch_rad)
% SOLVEBEMSECTION Closed-form BEM analysis for airfoil sections
% Calculates induction factors and sectional loads using Blade Element Momentum theory
% without iteration for improved computational efficiency.
%
% Syntax:
%   [a, a_prime, CL, CD, Cn, Ct, V_rel] = solveBEMSection(r, ~, twist_rad, perf_data, lambda_r, V_wind, omega_rad, pitch_rad)
%
% Input Arguments:
%   r           - Radial position [m]
%   ~           - Local chord [m] (unused in this implementation)
%   twist_rad   - Local geometric twist [rad]
%   perf_data   - Airfoil polar struct with fields .AoA, .CL, .CD
%   lambda_r    - Local tip-speed ratio (λ * r / R)
%   V_wind      - Freestream wind speed [m/s]
%   omega_rad   - Rotor speed [rad/s]
%   pitch_rad   - Blade pitch angle [rad]
%
% Output Arguments:
%   a, a_prime  - Axial and tangential induction factors
%   CL, CD      - Section lift and drag coefficients at computed α
%   Cn, Ct      - Normal and tangential force coefficients
%   V_rel       - Relative velocity magnitude at section [m/s]
%
% Description:
%   Performs closed-form BEM analysis assuming momentum-limit induction
%   (a = 1/3) and calculates corresponding tangential induction factor.
%   Interpolates airfoil performance data to find lift and drag coefficients.
%
% Example:
%   [a, a_prime, CL, CD, Cn, Ct, V_rel] = solveBEMSection(r, c, twist, perf_data, lambda_r, V_wind, omega, pitch);

    % ============================================================================
    % CLOSED-FORM BEM ANALYSIS (NON-ITERATIVE)
    % ============================================================================
    % Use momentum-limit assumption for computational efficiency
    % This avoids iterative solution while maintaining reasonable accuracy
    a = 1/3;  % Assumed momentum-limit axial induction factor (maximum theoretical value)
    
    % Calculate tangential induction factor from momentum theory
    % Derived from: a_prime = -0.5 + 0.5 * sqrt(1 + (4/λr²) * a * (1-a))
    a_prime = -0.5 + 0.5 * sqrt(1 + (4/(lambda_r^2)) * a * (1 - a));  % Tangential induction factor
    
    % ============================================================================
    % AERODYNAMIC ANGLE CALCULATIONS
    % ============================================================================
    % Calculate inflow angle from velocity triangle
    phi = atan((1 - a) / ((1 + a_prime) * lambda_r));  % Inflow angle [rad]
    
    % Calculate angle of attack: α = φ - (θ + β)
    % where θ = geometric twist, β = pitch angle
    alpha_deg = rad2deg(phi - (twist_rad + pitch_rad));  % Angle of attack [deg]
    
    % ============================================================================
    % AIRFOIL PERFORMANCE INTERPOLATION
    % ============================================================================
    % Interpolate airfoil performance data at calculated angle of attack
    % Uses linear interpolation with extrapolation for angles outside data range
    CL = interp1(perf_data.AoA, perf_data.CL, alpha_deg, 'linear', 'extrap');  % Lift coefficient
    CD = interp1(perf_data.AoA, perf_data.CD, alpha_deg, 'linear', 'extrap');  % Drag coefficient
    
    % ============================================================================
    % FORCE COEFFICIENT TRANSFORMATION
    % ============================================================================
    % Transform lift/drag coefficients to normal/tangential force coefficients
    % using inflow angle transformation
    s = sin(phi); cos_phi = cos(phi);  % Trigonometric functions for efficiency
    Cn = CL * cos_phi + CD * s;        % Normal force coefficient (thrust direction)
    Ct = CL * s - CD * cos_phi;        % Tangential force coefficient (torque direction)
    
    % ============================================================================
    % RELATIVE VELOCITY CALCULATION
    % ============================================================================
    % Calculate relative velocity magnitude from velocity triangle
    % V_rel = sqrt(V_axial² + V_tangential²)
    V_rel = sqrt((V_wind*(1-a))^2 + (omega_rad*r*(1+a_prime))^2);  % Relative velocity [m/s]
end

function [a, a_prime, CL, CD, Cn, Ct, V_rel] = solveCircleSection(r, c, lambda_r, V_wind, omega_rad, rho, mu)
% SOLVECIRCLESECTION BEM analysis for circular sections
% Calculates induction factors and loads for circular (non-lifting) sections
% with zero lift coefficient and Reynolds-dependent drag coefficient.
%
% Syntax:
%   [a, a_prime, CL, CD, Cn, Ct, V_rel] = solveCircleSection(r, c, lambda_r, V_wind, omega_rad, rho, mu)
%
% Input Arguments:
%   r           - Radial position [m]
%   c           - Local chord (diameter) [m]
%   lambda_r    - Local tip-speed ratio (λ * r / R)
%   V_wind      - Freestream wind speed [m/s]
%   omega_rad   - Rotor speed [rad/s]
%   rho         - Air density [kg/m³]
%   mu          - Air dynamic viscosity [Pa·s]
%
% Output Arguments:
%   a, a_prime  - Axial and tangential induction factors
%   CL, CD      - Lift and drag coefficients
%   Cn, Ct      - Normal and tangential force coefficients
%   V_rel       - Relative velocity [m/s]
%
% Description:
%   Performs BEM analysis for circular sections (e.g., tower sections)
%   assuming zero lift coefficient and Reynolds-dependent drag coefficient.
%   Uses empirical drag coefficient correlations for circular cylinders.
%
% Example:
%   [a, a_prime, CL, CD, Cn, Ct, V_rel] = solveCircleSection(r, c, lambda_r, V_wind, omega, rho, mu);
    
    % ============================================================================
    % CIRCULAR SECTION BEM ANALYSIS
    % ============================================================================
    % Use same momentum-limit assumption as airfoil sections for consistency
    a = 1/3;  % Assumed momentum-limit axial induction factor
    a_prime = -0.5 + 0.5 * sqrt(1 + (4/(lambda_r^2)) * a * (1 - a));  % Tangential induction factor
    
    % Calculate inflow angle (same as airfoil sections)
    phi = atan((1 - a) / ((1 + a_prime) * lambda_r));  % Inflow angle [rad]
    
    % ============================================================================
    % CIRCULAR SECTION AERODYNAMICS
    % ============================================================================
    % Calculate relative velocity and Reynolds number for drag coefficient
    V_rel = sqrt((V_wind*(1-a))^2 + (omega_rad*r*(1+a_prime))^2);  % Relative velocity [m/s]
    Re = max(1, rho * V_rel * c / mu);  % Reynolds number (minimum 1 to avoid division by zero)
    
    % Circular sections have zero lift and Reynolds-dependent drag
    CD = cylinderCDlocal(Re);  % Drag coefficient from empirical correlation
    CL = 0;  % Zero lift coefficient for circular sections (no camber)
    
    % ============================================================================
    % FORCE COEFFICIENT CALCULATION
    % ============================================================================
    % Transform to normal/tangential force coefficients (same as airfoil sections)
    s = sin(phi); cphi = cos(phi);  % Trigonometric functions
    Cn = CL * cphi + CD * s;        % Normal force coefficient (primarily drag)
    Ct = CL * s - CD * cphi;        % Tangential force coefficient (primarily drag)
end

function C_D = cylinderCDlocal(Re)
% CYLINDERCDLOCAL Drag coefficient for circular cylinders
% Calculates drag coefficient for smooth circular cylinders in cross-flow
% using empirical correlations based on Reynolds number.
%
% Syntax:
%   C_D = cylinderCDlocal(Re)
%
% Input Arguments:
%   Re - Reynolds number based on cylinder diameter
%
% Output Arguments:
%   C_D - Drag coefficient
%
% Description:
%   Uses empirical correlations to calculate drag coefficient for circular
%   cylinders in cross-flow. Different correlations are used for different
%   Reynolds number ranges to capture transition effects.
%
% Example:
%   Re = 1e5;
%   CD = cylinderCDlocal(Re);
%   fprintf('Drag coefficient: %.3f\n', CD);

    % ============================================================================
    % REYNOLDS NUMBER-BASED DRAG COEFFICIENT CORRELATIONS
    % ============================================================================
    % Use different empirical correlations for different Reynolds number ranges
    % to capture transition effects and flow regime changes
    
    if Re < 2e5
        % ============================================================================
        % LOW REYNOLDS NUMBER REGIME (Re < 2×10⁵)
        % ============================================================================
        % Laminar flow with boundary layer effects
        % Correlation accounts for viscous effects and boundary layer development
        C_D = 11 * Re.^(-0.75) + 0.9 * (1.0 - exp(-1000./Re)) + 1.2 * (1.0 - exp(-(Re./4500).^0.7));  % Low Re correlation
        
    elseif Re <= 5e5
        % ============================================================================
        % TRANSITION REGIME (2×10⁵ ≤ Re ≤ 5×10⁵)
        % ============================================================================
        % Critical Reynolds number region where boundary layer transitions from laminar to turbulent
        % Drag coefficient drops significantly due to turbulent boundary layer
        C_D = 10.^(0.32*tanh(44.4504 - 8 * log10(Re)) - 0.238793158);  % Transition region
        
    else
        % ============================================================================
        % HIGH REYNOLDS NUMBER REGIME (Re > 5×10⁵)
        % ============================================================================
        % Fully turbulent flow with logarithmic dependence on Reynolds number
        % Represents the "supercritical" regime for circular cylinders
        C_D = 0.1 * log10(Re) - 0.2533429;  % High Re correlation
    end
end

function [a, a_prime, CL, CD, Cn, Ct, V_rel] = getSectionCoefficients(airfoil, twist_deg, r, c, lambda_r, V_wind, omega_rad, pitch_rad, data, rho, mu, varargin)
% GETSECTIONCOEFFICIENTS Unified section coefficient calculation
% Calculates BEM coefficients for airfoil or circular sections based on airfoil type.
% Automatically handles both airfoil and circular section calculations.
%
% Syntax:
%   [a, a_prime, CL, CD, Cn, Ct, V_rel] = getSectionCoefficients(airfoil, twist_deg, r, c, lambda_r, V_wind, omega_rad, pitch_rad, data, rho, mu, varargin)
%
% Input Arguments:
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
% Output Arguments:
%   a, a_prime  - Axial and tangential induction factors
%   CL, CD      - Lift and drag coefficients
%   Cn, Ct      - Normal and tangential force coefficients
%   V_rel       - Relative velocity [m/s]
%
% Description:
%   Unified function that automatically determines whether to use airfoil
%   or circular section analysis based on the airfoil name. Handles both
%   lifting airfoil sections and non-lifting circular sections.
%
% Example:
%   [a, a_prime, CL, CD, Cn, Ct, V_rel] = getSectionCoefficients('DU96_W_180', 5, r, c, lambda_r, V_wind, omega, pitch, data, rho, mu);

    airfoil_name = regexprep(regexprep(airfoil, '\s+', ''), '[-–—]', '_');  % Clean airfoil name
    twist_rad = deg2rad(twist_deg);  % Convert twist to radians
    
    if strcmpi(airfoil_name, 'circle')
        [a, a_prime, CL, CD, Cn, Ct, V_rel] = solveCircleSection(r, c, ...
            lambda_r, V_wind, omega_rad, rho, mu);  % Circular section analysis
    else
        perf_data = data.airfoilPerformance.(airfoil_name);  % Airfoil performance data
        [a, a_prime, CL, CD, Cn, Ct, V_rel] = solveBEMSection(r, c, twist_rad, perf_data, ...
            lambda_r, V_wind, omega_rad, pitch_rad);  % Airfoil section analysis
    end
end

%% ============================================================================
%% SCRIPT EXECUTION
%% ============================================================================
% 
% TO RUN THE ANALYSIS:
% 1. Edit configuration in createConfig() function
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
% Configuration is controlled by the createConfig() function.
% Edit settings in createConfig() to control which deliverables and plots are executed.

    % Get configuration settings (deliverable flags, plotting options, etc.)
    config = createConfig();
    
    % Extract all wind turbine data from parameter files once for efficiency
    % This includes blade geometry, airfoil data, material properties, etc.
    data = ParameterExtraction(config);

    
    fprintf('\n');

    % Initialize results structure to store all deliverable outputs
    results = struct();
    
    % ============================================================================
    % DELIVERABLE 1: BASIC BEM ANALYSIS
    % ============================================================================
    % Calculates power coefficient (CP) and thrust coefficient (CT) using
    % Blade Element Momentum theory for specified operating conditions
    if config.run_deliverable_1
        fprintf('Running Deliverable 1...\n');
        [CP1, CT1] = Deliverable1(config, data);
        results.D1 = struct('CP', CP1, 'CT', CT1);
        fprintf('D1 Complete: CP=%.4f, CT=%.4f\n', CP1, CT1);
    else
        fprintf('Skipping Deliverable 1 (disabled)\n\n');
    end
    
    % ============================================================================
    % DELIVERABLE 2: PITCH OPTIMIZATION
    % ============================================================================
    % Finds optimal pitch angle for maximum power coefficient at fixed tip speed ratio
    % Uses BEM analysis to evaluate performance across pitch angle range
    if config.run_deliverable_2
        fprintf('Running Deliverable 2...\n');
        [CP2, optimal_conditions] = Deliverable2(config, data);
        results.D2 = struct('CP', CP2, 'optimal_conditions', optimal_conditions);
        fprintf('D2 Complete: Max CP=%.4f at pitch=%.1f°\n', CP2, optimal_conditions.pitch_angle);
    else
        fprintf('Skipping Deliverable 2 (disabled)\n\n');
    end
    
    % ============================================================================
    % DELIVERABLE 3: 2D CP OPTIMIZATION
    % ============================================================================
    % Performs 2D optimization over tip speed ratio and pitch angle
    % Lambda range calculated from actual turbine speed limits (9.6-15.5 RPM)
    % Finds global maximum power coefficient across both variables
    if config.run_deliverable_3
        fprintf('Running Deliverable 3...\n');
        [CP3, optimal_conditions_3D] = Deliverable3(config, data);
        results.D3 = struct('CP', CP3, 'optimal_conditions', optimal_conditions_3D);
        fprintf('D3 Complete: Max CP=%.4f at λ=%.2f, pitch=%.1f°\n', CP3, optimal_conditions_3D.lambda, optimal_conditions_3D.pitch_angle);
    else
        fprintf('Skipping Deliverable 3 (disabled)\n\n');
    end
    
    % ============================================================================
    % DELIVERABLE 4: RATED POWER PITCH CONTROL
    % ============================================================================
    % Determines required pitch angle to achieve rated power at specified wind speed
    % Uses optimization to find pitch that produces exactly 2.5 MW output
    % Lambda range calculated from actual turbine speed limits
    if config.run_deliverable_4
        fprintf('Running Deliverable 4...\n');
        result4 = Deliverable4(config, data);
        results.D4 = result4;
        fprintf('D4 Complete: Required pitch=%.1f° at λ=%.2f\n', result4.pitch_deg, result4.lambda);
    else
        fprintf('Skipping Deliverable 4 (disabled)\n\n');
        % Create dummy result for D5 if needed
        result4 = struct('lambda', 5.0, 'pitch_deg', 0.0);
    end
    
    % ============================================================================
    % DELIVERABLE 5: TOWER STRUCTURAL ANALYSIS
    % ============================================================================
    % Comprehensive structural analysis including:
    % - Tower deflection under wind loading
    % - Stress analysis with Mohrs circle
    % - Fatigue analysis using Goodman diagram
    % - Safety factor calculations for both static and fatigue failure
    % Uses results from D4 for realistic operating conditions
    if config.run_deliverable_5
        fprintf('Running Deliverable 5...\n');
        result5 = Deliverable5(config, data, result4.lambda, result4.pitch_deg);
        results.D5 = result5;
        fprintf('D5 Complete: Max deflection=%.3f m, Fatigue SF=%.2f, Static SF=%.2f\n', ...
            max(abs(result5.deflection.deflection)), result5.stress.fatigue_safety_factor, result5.stress.static_failure.min_SF);
    else
        fprintf('Skipping Deliverable 5 (disabled)\n\n');
    end
    
    % Summary
    fprintf('=== ALL DELIVERABLES COMPLETE ===\n');
    if config.run_deliverable_1
        fprintf('D1: CP=%.4f, CT=%.4f\n', results.D1.CP, results.D1.CT);
    end
    if config.run_deliverable_2
        fprintf('D2: Max CP=%.4f at pitch=%.1f°\n', results.D2.CP, results.D2.optimal_conditions.pitch_angle);
    end
    if config.run_deliverable_3
        fprintf('D3: Max CP=%.4f at λ=%.2f, pitch=%.1f°\n', results.D3.CP, results.D3.optimal_conditions.lambda, results.D3.optimal_conditions.pitch_angle);
    end
    if config.run_deliverable_4
        fprintf('D4: Required pitch=%.1f° at λ=%.2f\n', results.D4.pitch_deg, results.D4.lambda);
    end
    if config.run_deliverable_5
        fprintf('D5: Max deflection=%.3f m, Fatigue SF=%.2f, Static SF=%.2f\n', ...
            max(abs(results.D5.deflection.deflection)), results.D5.stress.fatigue_safety_factor, results.D5.stress.static_failure.min_SF);
    end

end

% ============================================================================
% DELIVERABLE 1: BASIC BEM ANALYSIS
% ============================================================================
% Calculates power and thrust coefficients using BEM theory

function [CP, CT] = Deliverable1(config, data)
% DELIVERABLE1 Wind Turbine BEM Analysis
% Calculates power and thrust coefficients for specified operating conditions
% Usage: [CP, CT] = Deliverable1(config, data);

    % Extract operating conditions and calculate derived parameters
    V_wind = data.deliverables.part1.V_wind;
    omega_rad = rpm2rad(data.deliverables.part1.omega_rpm);
    pitch_rad = deg2rad(data.deliverables.part1.pitch_deg);
    R = data.turbine.performance.rotorRadius;
    A = data.turbine.calculated.rotorArea;
    rho = data.materials.air.density;
    lambda = omega_rad * R / V_wind;
    
    % Prepare blade stations and initialize arrays
    blade_stations = prepareBladeStations(data);
    dT = zeros(blade_stations.n_stations, 1);
    dQ = zeros(blade_stations.n_stations, 1);
    dP = zeros(blade_stations.n_stations, 1);
    dCP = zeros(blade_stations.n_stations, 1);
    dCT = zeros(blade_stations.n_stations, 1);
    
    % Calculate BEM for each station
    for i = 1:blade_stations.n_stations
        station_data = struct('r', blade_stations.r_stations(i), 'c', blade_stations.chord_m(i), ...
            'twist_deg', blade_stations.twist_deg_vec(i), 'airfoil', blade_stations.airfoil_raw{i});
        conditions = struct('lambda_r', lambda * blade_stations.r_stations(i) / R, ...
            'V_wind', V_wind, 'omega_rad', omega_rad, 'pitch_rad', pitch_rad);
        
        [dT(i), dQ(i), dP(i)] = calculateBEMAtStation(station_data, conditions, data);
        dCP(i) = dP(i) / (0.5 * rho * A * V_wind^3);
        dCT(i) = dT(i) / (0.5 * rho * A * V_wind^2);
    end
    
    % Integrate loads to get total coefficients
    [CP, CT] = integrateBladeLoads(blade_stations.r_stations, dT, dQ, omega_rad, ...
        data.turbine.characteristics.blades, A, V_wind, rho);
    
    % Create visualization if enabled
    if config.plot_d1_results
        createVisualization(blade_stations.r_stations, dCP, dCT, CP, CT, lambda, V_wind, ...
            data.deliverables.part1.omega_rpm, config);
    end
end

% ============================================================================
% DELIVERABLE 2: PITCH OPTIMIZATION
% ============================================================================
% Finds optimal pitch angle for maximum power coefficient

function [CP_max, optimal_conditions] = Deliverable2(config, data, V_wind, lambda, pitch_min, pitch_max, pitch_step)
% DELIVERABLE2 Wind Turbine Pitch Angle Optimization
% Finds maximum power coefficient by varying pitch angle
% Usage: [CP_max, optimal_conditions] = Deliverable2(config, data);

    % Defaults from predefined deliverables when inputs are omitted
    if nargin < 3 || isempty(V_wind), V_wind = data.deliverables.part2.V_wind; end
    if nargin < 4 || isempty(lambda), lambda = data.deliverables.part2.lambda; end
    if nargin < 5 || isempty(pitch_min), pitch_min = -15; end
    if nargin < 6 || isempty(pitch_max), pitch_max = 15; end
    if nargin < 7 || isempty(pitch_step), pitch_step = 1; end

    % Calculate derived parameters
    R = data.turbine.performance.rotorRadius;
    A = data.turbine.calculated.rotorArea;
    rho = data.materials.air.density;
    omega_rad = (lambda * V_wind) / R;
    
    % Prepare blade stations and setup optimization
    blade_stations = prepareBladeStations(data);
    pitch_range = pitch_min:pitch_step:pitch_max;
    n_points = length(pitch_range);
    CP_values = zeros(n_points, 1);
    CT_values = zeros(n_points, 1);
    P_values = zeros(n_points, 1);
    T_values = zeros(n_points, 1);
    
    % Optimize over pitch angles
    for j = 1:n_points
        pitch_rad = deg2rad(pitch_range(j));
        dT = zeros(blade_stations.n_stations, 1);
        dQ = zeros(blade_stations.n_stations, 1);
        dP = zeros(blade_stations.n_stations, 1);
        
        % Calculate BEM for each station
        for i = 1:blade_stations.n_stations
            station_data = struct('r', blade_stations.r_stations(i), 'c', blade_stations.chord_m(i), ...
                'twist_deg', blade_stations.twist_deg_vec(i), 'airfoil', blade_stations.airfoil_raw{i});
            conditions = struct('lambda_r', lambda * blade_stations.r_stations(i) / R, ...
                'V_wind', V_wind, 'omega_rad', omega_rad, 'pitch_rad', pitch_rad);
            
            [dT(i), dQ(i), dP(i)] = calculateBEMAtStation(station_data, conditions, data);
        end
        
        % Integrate loads to get total coefficients
        [CP_values(j), CT_values(j), P_values(j), T_values(j)] = integrateBladeLoads(...
            blade_stations.r_stations, dT, dQ, omega_rad, data.turbine.characteristics.blades, A, V_wind, rho);
    end
    
    % Find optimal conditions
    [CP_max, max_idx] = max(CP_values);
    optimal_conditions = struct('pitch_angle', pitch_range(max_idx), 'lambda', lambda, ...
        'omega_rad', omega_rad, 'omega_rpm', rad2rpm(omega_rad), 'CP', CP_max, ...
        'CT', CT_values(max_idx), 'P', P_values(max_idx), 'T', T_values(max_idx), 'V_wind', V_wind);
    
    % Create visualization if enabled
    if config.plot_d2_optimization
        createPitchOptimizationPlot(pitch_range, CP_values, CT_values, pitch_range(max_idx), CP_max, V_wind, lambda, config);
    end
end

% ============================================================================
% DELIVERABLE 3: 2D CP OPTIMIZATION
% ============================================================================
% Performs 2D optimization over tip speed ratio and pitch angle to find
% maximum power coefficient using nested optimization loops.

function [CP_max, optimal_conditions] = Deliverable3(config, data)
% DELIVERABLE3 Wind Turbine 2D CP Optimization
% Finds maximum power coefficient by varying tip speed ratio and pitch angle
% using Blade Element Momentum theory for comprehensive performance analysis.
%
% Syntax:
%   [CP_max, optimal_conditions] = Deliverable3(config, data)
%
% Input Arguments:
%   config - Configuration structure with plotting options
%   data   - Wind turbine data structure containing parameters
%
% Output Arguments:
%   CP_max            - Maximum power coefficient achieved
%   optimal_conditions - Structure containing optimal operating conditions
%
% Description:
%   Performs 2D optimization over tip speed ratio (3-10) and pitch angle 
%   (-15° to 15°) to find maximum power coefficient. Uses BEM analysis
%   to calculate sectional loads and integrates across blade stations.
%
% Example:
%   [CP_max, optimal] = Deliverable3(config, data);
%   fprintf('Maximum CP: %.4f at λ=%.1f, θ=%.1f°\n', ...
%       CP_max, optimal.lambda, optimal.pitch_angle);

    % Define optimization parameters
    V_wind = data.deliverables.part3.V_wind;  % Wind speed [m/s]
    pitch_min = -15; pitch_max = 15; pitch_step = 1;  % Pitch angle range [deg]
    
    % Extract turbine parameters
    R = data.turbine.performance.rotorRadius;  % Rotor radius [m]
    A = data.turbine.calculated.rotorArea;     % Rotor swept area [m²]
    rho = data.materials.air.density;         % Air density [kg/m³]
    B = data.turbine.characteristics.blades;   % Number of blades
    
    % Calculate lambda range based on actual turbine speed limits
    omega_min_rpm = data.turbine.performance.rotorSpeedRange(1);  % Minimum rotor speed [RPM]
    omega_max_rpm = data.turbine.performance.rotorSpeedRange(2);  % Maximum rotor speed [RPM]
    omega_min_rad = rpm2rad(omega_min_rpm);  % Convert to rad/s
    omega_max_rad = rpm2rad(omega_max_rpm);  % Convert to rad/s
    
    lambda_min = omega_min_rad * R / V_wind;  % Minimum tip speed ratio
    lambda_max = omega_max_rad * R / V_wind;  % Maximum tip speed ratio
    lambda_step = (lambda_max - lambda_min) / 10;  % Dynamic step size
    
    % Create optimization ranges
    lambda_range = lambda_min:lambda_step:lambda_max;  % Tip speed ratio array
    pitch_range = pitch_min:pitch_step:pitch_max;      % Pitch angle array [deg]
    [n_pitch, n_lambda] = deal(length(pitch_range), length(lambda_range));  % Array sizes
    
    % Initialize result matrices
    CP_matrix = zeros(n_pitch, n_lambda);  % Power coefficient matrix
    CT_matrix = zeros(n_pitch, n_lambda);  % Thrust coefficient matrix
    P_matrix = zeros(n_pitch, n_lambda);   % Power output matrix [W]
    T_matrix = zeros(n_pitch, n_lambda);   % Thrust force matrix [N]
    
    % Prepare blade stations
    blade_stations = prepareBladeStations(data);  % Blade geometry data
    
    % 2D optimization loops
    for j = 1:n_lambda
        lambda = lambda_range(j);  % Current tip speed ratio
        omega_rad = lambda * V_wind / R;  % Rotor angular velocity [rad/s]
        
        for k = 1:n_pitch
            pitch_rad = deg2rad(pitch_range(k));  % Pitch angle [rad]
            dT = zeros(blade_stations.n_stations, 1);  % Thrust force per station [N]
            dQ = zeros(blade_stations.n_stations, 1);  % Torque per station [N·m]
            
            % Calculate BEM for each station
            for i = 1:blade_stations.n_stations
                lambda_r = lambda * blade_stations.r_stations(i) / R;  % Local tip speed ratio
                [~, ~, ~, ~, Cn, Ct, V_rel] = getSectionCoefficients(blade_stations.airfoil_raw{i}, blade_stations.twist_deg_vec(i), blade_stations.r_stations(i), blade_stations.chord_m(i), ...
                    lambda_r, V_wind, omega_rad, pitch_rad, data, rho, data.materials.air.viscosity);
                
                dT(i) = 0.5 * rho * V_rel^2 * blade_stations.chord_m(i) * Cn;  % Sectional thrust [N]
                dQ(i) = 0.5 * rho * V_rel^2 * blade_stations.chord_m(i) * Ct * blade_stations.r_stations(i);  % Sectional torque [N·m]
            end
            
            % Integrate loads and calculate coefficients
            T_total = B * trapz(blade_stations.r_stations, dT);  % Total thrust [N]
            P_total = B * trapz(blade_stations.r_stations, dQ) * omega_rad;  % Total power [W]
            
            CP_matrix(k, j) = P_total / (0.5 * rho * A * V_wind^3);  % Power coefficient
            CT_matrix(k, j) = T_total / (0.5 * rho * A * V_wind^2);  % Thrust coefficient
            P_matrix(k, j) = P_total;  % Power output [W]
            T_matrix(k, j) = T_total;  % Thrust force [N]
        end
    end
    
    % Find optimal conditions
    [CP_max, max_idx] = max(CP_matrix(:));  % Maximum power coefficient
    [optimal_pitch_idx, optimal_lambda_idx] = ind2sub(size(CP_matrix), max_idx);  % Optimal indices
    
    optimal_lambda = lambda_range(optimal_lambda_idx);  % Optimal tip speed ratio
    optimal_pitch = pitch_range(optimal_pitch_idx);     % Optimal pitch angle [deg]
    optimal_omega_rad = optimal_lambda * V_wind / R;    % Optimal rotor speed [rad/s]
    
    % Compile results
    optimal_conditions = struct('lambda', optimal_lambda, 'pitch_angle', optimal_pitch, ...
        'omega_rad', optimal_omega_rad, 'omega_rpm', optimal_omega_rad * 60 / (2 * pi), ...
        'CP', CP_max, 'CT', CT_matrix(optimal_pitch_idx, optimal_lambda_idx), ...
        'P', P_matrix(optimal_pitch_idx, optimal_lambda_idx), ...
        'T', T_matrix(optimal_pitch_idx, optimal_lambda_idx), 'V_wind', V_wind);
    
    % Create visualization if enabled
    if config.plot_d3_optimization
        create3DOptimizationPlot(lambda_range, pitch_range, CP_matrix, optimal_lambda, optimal_pitch, CP_max, V_wind, config);
    end
end

% ============================================================================
% DELIVERABLE 4: RATED POWER PITCH CONTROL
% ============================================================================
% Determines required pitch angle to maintain rated power at high wind speeds
% using iterative BEM analysis with power constraint.

function result = Deliverable4(config, data)
% DELIVERABLE4 Rated Power Pitch Control
% Determines the blade pitch angle required to maintain rated power output
% at high wind speeds using pitch control optimization.
%
% Syntax:
%   result = Deliverable4(config, data)
%
% Input Arguments:
%   config - Configuration structure with plotting options
%   data   - Wind turbine data structure containing parameters
%
% Output Arguments:
%   result - Structure containing pitch control results
%     .pitch_deg  - Required pitch angle [degrees]
%     .lambda     - Optimal tip speed ratio
%     .omega_rad  - Rotor angular velocity [rad/s]
%     .omega_rpm  - Rotor speed [RPM]
%     .P          - Power output [W]
%     .CP         - Power coefficient
%
% Description:
%   Finds the minimum pitch angle required to keep turbine power output
%   at or below rated power for high wind speed conditions. Optimizes
%   over rotor speed range and pitch angles to find operating point.
%
% Example:
%   result = Deliverable4(config, data);
%   fprintf('Required pitch: %.1f° for %.1f kW power\n', ...
%       result.pitch_deg, result.P/1000);

    % Define parameters
    V_wind = data.deliverables.part4.V_wind;  % High wind speed [m/s]
    ratedPower = data.turbine.performance.ratedPower;  % Rated power limit [W]
    R = data.turbine.performance.rotorRadius;  % Rotor radius [m]
    A = data.turbine.calculated.rotorArea;     % Rotor swept area [m²]
    rho = data.materials.air.density;         % Air density [kg/m³]
    B = data.turbine.characteristics.blades;   % Number of blades
    
    % Calculate lambda range from rotor speed limits
    omega_min_rpm = data.turbine.performance.rotorSpeedRange(1);  % Minimum rotor speed [RPM]
    omega_max_rpm = data.turbine.performance.rotorSpeedRange(2);  % Maximum rotor speed [RPM]
    omega_min_rad = rpm2rad(omega_min_rpm);  % Convert to rad/s
    omega_max_rad = rpm2rad(omega_max_rpm);  % Convert to rad/s
    
    lambda_min = omega_min_rad * R / V_wind;  % Minimum tip speed ratio
    lambda_max = omega_max_rad * R / V_wind;  % Maximum tip speed ratio
    lambda_range = lambda_min:0.25:lambda_max;  % Tip speed ratio array
    
    % Define pitch range
    pitch_range = 0:0.1:30;  % Pitch angle range [deg]
    
    % Prepare blade stations
    blade_stations = prepareBladeStations(data);  % Blade geometry data
    
    % Initialize results
    CP_pitch = zeros(numel(pitch_range), 1);  % Power coefficient array
    lambda_at_CP = zeros(numel(pitch_range), 1);  % Optimal lambda array
    
    % Optimize over pitch angles
    for j = 1:numel(pitch_range)
        pitch_rad = deg2rad(pitch_range(j));  % Current pitch angle [rad]
        best_CP = -inf;  % Initialize best power coefficient
        
        % Find maximum CP across lambda range for this pitch
        for lam = lambda_range
            omega_rad = lam * V_wind / R;  % Rotor angular velocity [rad/s]
            dT = zeros(blade_stations.n_stations, 1);  % Thrust per station [N]
            dQ = zeros(blade_stations.n_stations, 1);  % Torque per station [N·m]
            
            % Calculate BEM for each station
            for i = 1:blade_stations.n_stations
                lambda_r = lam * blade_stations.r_stations(i) / R;  % Local tip speed ratio
                [~, ~, ~, ~, Cn, Ct, V_rel] = getSectionCoefficients(blade_stations.airfoil_raw{i}, blade_stations.twist_deg_vec(i), blade_stations.r_stations(i), blade_stations.chord_m(i), ...
                    lambda_r, V_wind, omega_rad, pitch_rad, data, rho, data.materials.air.viscosity);
                
                dT(i) = 0.5 * rho * V_rel^2 * blade_stations.chord_m(i) * Cn;  % Sectional thrust [N]
                dQ(i) = 0.5 * rho * V_rel^2 * blade_stations.chord_m(i) * Ct * blade_stations.r_stations(i);  % Sectional torque [N·m]
            end
            
            % Calculate power and CP
            P_total = B * trapz(blade_stations.r_stations, dQ) * omega_rad;  % Total power [W]
            CP_val = P_total / (0.5 * rho * A * V_wind^3);  % Power coefficient
            
            if CP_val > best_CP
                best_CP = CP_val;  % Update best power coefficient
                lambda_at_CP(j) = lam;  % Store optimal lambda
            end
        end
        
        CP_pitch(j) = best_CP;  % Store best power coefficient
    end
    
    % Find required pitch to meet rated power
    P_pitch = CP_pitch * (0.5 * rho * A * V_wind^3);  % Power array [W]
    idx_ok = find(P_pitch <= ratedPower, 1, 'first');  % First acceptable index
    if isempty(idx_ok)
        idx_ok = numel(pitch_range);  % Use maximum pitch if no solution
    end
    
    % Compile results
    pitch_req = pitch_range(idx_ok);  % Required pitch angle [deg]
    lambda_req = lambda_at_CP(idx_ok);  % Required tip speed ratio
    omega_req = lambda_req * V_wind / R;  % Required rotor speed [rad/s]
    
    result = struct('pitch_deg', pitch_req, 'lambda', lambda_req, 'omega_rad', omega_req, ...
        'omega_rpm', omega_req*60/(2*pi), 'P', P_pitch(idx_ok), 'CP', CP_pitch(idx_ok), ...
        'ratedPower', ratedPower, 'V_wind', V_wind);
    
    % Create visualization if enabled
    if config.plot_d4_power
        createPowerPitchPlot(pitch_range, P_pitch, pitch_req, P_pitch(idx_ok), ratedPower, V_wind, config);
    end
end

% ============================================================================
% DELIVERABLE 5: TOWER DEFLECTION AND STRESS ANALYSIS
% ============================================================================
% Computes tower deflection and stress analysis considering distributed wind
% loading, thrust forces, and fatigue analysis with Mohrs circle and Goodman diagram.

function result = Deliverable5(config, data, lambda, pitch_deg)
% DELIVERABLE5 Tower Deflection and Stress Analysis
% Computes comprehensive tower structural analysis including deflection,
% stress analysis, and fatigue assessment using beam theory and BEM analysis.
%
% Syntax:
%   result = Deliverable5(config, data, lambda, pitch_deg)
%
% Input Arguments:
%   config     - Configuration structure with plotting options
%   data       - Wind turbine data structure containing parameters
%   lambda     - Tip speed ratio from Deliverable 4 optimization
%   pitch_deg  - Pitch angle from Deliverable 4 optimization [degrees]
%
% Output Arguments:
%   result - Structure containing comprehensive analysis results
%     .deflection        - Tower deflection analysis results
%     .stress           - Stress analysis results (Mohrs circle, fatigue)
%     .thrust_force     - Rotor thrust force [N]
%     .tower_height     - Tower height [m]
%     .hub_height       - Hub height [m]
%     .material_properties - Steel material properties
%
% Description:
%   Performs complete structural analysis of wind turbine tower including:
%   - Distributed wind loading using atmospheric boundary layer
%   - Rotor thrust forces from BEM analysis
%   - Beam deflection using numerical integration
%   - Stress analysis with Mohrs circle theory
%   - Fatigue analysis using Goodman diagram
%
% Example:
%   result = Deliverable5(config, data, 6.5, 12.3);
%   fprintf('Max deflection: %.3f m\n', max(abs(result.deflection.deflection)));

    % Extract parameters
    V_wind = data.deliverables.part4.V_wind;  % Wind speed [m/s]
    tower_specs = data.tower.specs;           % Tower specifications
    E = data.materials.steel.youngsModulus;   % Youngs modulus [Pa]
    S_ut = data.materials.steel.tensileStrength;  % Ultimate tensile strength [Pa]
    S_y = data.materials.steel.yieldStrength;      % Yield strength [Pa]
    
    % Calculate thrust force and define loading scenarios
    thrust_force = calculateRotorThrust(data, V_wind, lambda, pitch_deg);  % Rotor thrust [N]
    load_cases = defineLoadCases(V_wind, thrust_force, data, lambda, pitch_deg);  % Load scenarios
    
    % Calculate tower properties and solve deflection
    section_props = computeTowerSectionProperties(tower_specs);  % Geometric properties
    deflection_results = solveTowerDeflection(section_props, load_cases, E, data);  % Deflection analysis
    stress_results = computeStressAnalysis(section_props, load_cases, deflection_results, E, data);  % Stress analysis
    
    % Create visualizations if enabled
    if config.plot_d5_deflection
        createTowerDeflectionPlot(deflection_results, section_props, config);
    end
    if config.plot_d5_mohr
        createMohrCirclePlots(stress_results, config);
    end
    if config.plot_d5_goodman
        createGoodmanDiagram(stress_results, S_ut, S_y, data, config);
    end
    if config.plot_d5_tower
        createTowerAnalysisPlots(deflection_results, load_cases, section_props, data, config);
    end
    
    % Display safety factor results
    fprintf('\n=== DELIVERABLE 5 SAFETY FACTOR ANALYSIS ===\n');
    fprintf('Fatigue Safety Factor: %.3f\n', stress_results.fatigue_safety_factor);
    fprintf('Static Failure Safety Factors:\n');
    fprintf('  Maximum Normal Stress Theory: %.3f\n', stress_results.static_failure.MNST_SF);
    fprintf('  Maximum Shear Stress Theory: %.3f\n', stress_results.static_failure.MSST_SF);
    fprintf('  Distortion Energy Theory: %.3f\n', stress_results.static_failure.DET_SF);
    fprintf('  Minimum Static Safety Factor: %.3f\n', stress_results.static_failure.min_SF);
    fprintf('===============================================\n\n');
    
    % Compile results
    result = struct('deflection', deflection_results, 'stress', stress_results, ...
        'thrust_force', thrust_force, 'tower_height', tower_specs.Height_mm_(end) / 1000, ...
        'hub_height', data.turbine.performance.hubHeight, ...
        'material_properties', struct('E', E, 'S_ut', S_ut, 'S_y', S_y));
end

%% ============================================================================
%% LOADING, DEFLECTION, AND STRESS FUNCTIONS
%% ============================================================================

function thrust_force = calculateRotorThrust(data, V_wind, lambda, pitch_deg)
% CALCULATEROTORTHRUST Calculate rotor thrust force using BEM analysis
% Computes total rotor thrust force by integrating sectional thrust forces
% across all blade stations using Blade Element Momentum theory.
%
% Syntax:
%   thrust_force = calculateRotorThrust(data, V_wind, lambda, pitch_deg)
%
% Input Arguments:
%   data      - Wind turbine data structure containing parameters
%   V_wind    - Wind speed [m/s]
%   lambda    - Tip speed ratio
%   pitch_deg - Blade pitch angle [degrees]
%
% Output Arguments:
%   thrust_force - Total rotor thrust force [N]
%
% Description:
%   Calculates rotor thrust force by performing BEM analysis at each blade
%   station and integrating the sectional thrust forces across the blade.
%   Uses airfoil performance data and blade geometry for accurate results.
%
% Example:
%   thrust = calculateRotorThrust(data, 14.6, 6.5, 12.3);
%   fprintf('Rotor thrust: %.1f kN\n', thrust/1000);

    % Extract parameters and prepare blade stations
    R = data.turbine.performance.rotorRadius;  % Rotor radius [m]
    rho = data.materials.air.density;         % Air density [kg/m³]
    B = data.turbine.characteristics.blades;   % Number of blades
    blade_stations = prepareBladeStations(data);  % Blade geometry data
    
    % Calculate thrust using BEM analysis
    omega_rad = lambda * V_wind / R;  % Rotor angular velocity [rad/s]
    pitch_rad = deg2rad(pitch_deg);   % Pitch angle [rad]
    
    dT = zeros(blade_stations.n_stations, 1);  % Thrust per station [N]
    for i = 1:blade_stations.n_stations
        lambda_r = lambda * blade_stations.r_stations(i) / R;  % Local tip speed ratio
        [~, ~, ~, ~, Cn, ~, V_rel] = getSectionCoefficients(blade_stations.airfoil_raw{i}, blade_stations.twist_deg_vec(i), blade_stations.r_stations(i), blade_stations.chord_m(i), ...
            lambda_r, V_wind, omega_rad, pitch_rad, data, rho, data.materials.air.viscosity);
        dT(i) = 0.5 * rho * V_rel^2 * blade_stations.chord_m(i) * Cn;  % Sectional thrust [N]
    end
    
    thrust_force = B * trapz(blade_stations.r_stations, dT);  % Total thrust [N]
end

function load_cases = defineLoadCases(V_wind, thrust_force, data, lambda, pitch_deg)
% DEFINELOADCASES Define loading scenarios for fatigue analysis
% Creates two load cases for fatigue analysis: maximum and minimum loading
% scenarios with different wind speeds and directions.
%
% Inputs:
%   v_wind      - Reference wind speed [m/s]
%   thrust_force - Thrust force from rotor [N]
%   data        - Wind turbine data structure
%   lambda      - Tip speed ratio
%   pitch_deg   - Blade pitch angle [degrees]
%
% Outputs:
%   load_cases  - Structure containing case1 and case2 load scenarios
    
    % Load Case 1: Wind from 315° at magnitude 1.0 (maximum)
    load_cases.case1 = struct('wind_speed', V_wind, 'wind_direction', 315, ...
        'thrust_force', thrust_force, 'description', 'Maximum loading - wind from 315°');
    
    % Load Case 2: Wind from 157° at magnitude 0.5 (minimum)
    load_cases.case2 = struct('wind_speed', V_wind * 0.5, 'wind_direction', 157, ...
        'thrust_force', calculateRotorThrust(data, V_wind * 0.5, lambda, pitch_deg), ...
        'description', 'Minimum loading - wind from 157°');
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
    
    % Convert units and calculate properties
    section_props.height = tower_specs.Height_mm_ / 1000;
    section_props.OD = tower_specs.OD_mm_ / 1000;
    section_props.wall_thickness = tower_specs{:,3} / 1000;
    section_props.ID = section_props.OD - 2 * section_props.wall_thickness;
    
    % Calculate section properties
    section_props.area = pi/4 * (section_props.OD.^2 - section_props.ID.^2);
    section_props.I = pi/64 * (section_props.OD.^4 - section_props.ID.^4);
    section_props.c = section_props.OD / 2;
    section_props.section_modulus = section_props.I ./ section_props.c;
end

function deflection_results = solveTowerDeflection(section_props, load_cases, E, data)
% SOLVETOWERDEFLECTION Solve beam deflection using numerical integration
% Calculates tower deflection for multiple load cases using beam theory
% with proper boundary conditions and distributed loading.
%
% Inputs:
%   section_props - Tower section properties structure
%   load_cases    - Loading scenarios structure
%   e            - Youngs modulus [Pa]
%   data         - Wind turbine data structure
%
% Outputs:
%   deflection_results - Structure containing deflection and moment results
    
    % Discretize tower into elements
    n_elements = 150;
    hub_height = data.turbine.performance.hubHeight;
    tower_height = section_props.height(end);
    total_height = tower_height + (hub_height - tower_height) * 2;
    z = linspace(0, total_height, n_elements+1);
    
    % Calculate deflection for both load cases
    [deflection_case1, moment_case1] = calculateDeflectionCase(section_props, load_cases.case1, z, E, data);
    [deflection_case2, moment_case2] = calculateDeflectionCase(section_props, load_cases.case2, z, E, data);
    
    % Compile results
    deflection_results = struct('z', z, 'deflection', deflection_case1, 'moment', moment_case1, ...
        'case1', struct('deflection', deflection_case1, 'moment', moment_case1), ...
        'case2', struct('deflection', deflection_case2, 'moment', moment_case2));
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
%   e            - Youngs modulus [Pa]
%   data         - Wind turbine data structure
%
% Outputs:
%   deflection   - Deflection at each height [m]
%   moment       - Bending moment at each height [N·m]
    
    n_points = length(z);
    moment = zeros(size(z));
    
    % Calculate wind profile and distributed loading
    [~, drag_force, nacelle_load] = computeWindLoading(section_props, load_case, z, data);
    
    % Calculate moment distribution using proper beam theory
    for i = 1:n_points
        wind_moment = 0;
        nacelle_moment = 0;
        
        for j = i:n_points-1
            if j < n_points
                dz = z(j+1) - z(j);
                lever_arm = z(j) - z(i);
                if ~isnan(dz) && ~isnan(lever_arm) && ~isnan(drag_force(j))
                    wind_moment = wind_moment + drag_force(j) * dz * lever_arm;
                end
                if ~isnan(dz) && ~isnan(lever_arm) && ~isnan(nacelle_load(j))
                    nacelle_moment = nacelle_moment + nacelle_load(j) * dz * lever_arm;
                end
            end
        end
        
        moment(i) = wind_moment + nacelle_moment;
    end
    
    % Calculate deflection using proper beam theory (double integration)
    I = interp1(section_props.height, section_props.I, z, 'linear', 'extrap');
    curvature = moment ./ (E * I);
    curvature(z > section_props.height(end)) = 0;  % Set curvature to zero in nacelle region
    
    % Double integration to get deflection
    slope = cumtrapz(z, curvature);
    deflection = cumtrapz(z, slope);
    deflection = deflection - deflection(1);  % Apply boundary conditions
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
    
    % Calculate wind profile and drag force
    z_ref = data.turbine.performance.hubHeight;
    U_ref = load_case.wind_speed;
    epsilon = 1/7;
    tower_height = section_props.height(end);
    
    wind_profile = U_ref * (z / z_ref).^epsilon;
    wind_profile(z > tower_height) = 0;
    
    D = interp1(section_props.height, section_props.OD, z, 'linear', 'extrap');
    rho_air = data.materials.air.density;
    mu_air = data.materials.air.viscosity;
    
    Re = rho_air * wind_profile .* D / mu_air;
    C_d = arrayfun(@cylinderCDlocal, Re);
    C_d(z > tower_height) = 0;
    
    drag_force = 0.5 * rho_air * wind_profile.^2 .* D .* C_d;
    drag_force(z > tower_height) = 0;
    
    % Calculate nacelle distributed load
    hub_height = data.turbine.performance.hubHeight;
    nacelle_height = (hub_height - tower_height) * 2;
    total_height = tower_height + nacelle_height;
    
    nacelle_load = zeros(size(z));
    nacelle_region = (z > tower_height) & (z <= total_height);
    if any(nacelle_region) && nacelle_height > 0
        nacelle_load(nacelle_region) = load_case.thrust_force / nacelle_height;
    end
end

function stress_results = computeStressAnalysis(section_props, load_cases, deflection_results, ~, data)
% COMPUTESTRESSANALYSIS Calculate stress analysis at tower base
% Performs stress analysis including bending, axial, and combined stresses
% with fatigue analysis using Goodman diagram methodology.
%
% Inputs:
%   section_props    - Tower section properties structure
%   load_cases       - Loading scenarios structure
%   deflection_results - Deflection analysis results
%   e               - Youngs modulus [Pa]
%   data            - Wind turbine data structure
%
% Outputs:
%   stress_results  - Structure containing stress analysis results
    
    % ============================================================================
    % STRESS CALCULATION AT TOWER BASE
    % ============================================================================
    % Extract geometric properties at tower base (most critical location)
    base_moment_of_inertia = section_props.I(1);           % Second moment of area [m^4]
    base_distance_to_outer_fiber = section_props.c(1);     % Distance to outer fiber [m]
    
    % Get bending moments at tower base from deflection analysis
    base_moment_case1 = deflection_results.case1.moment(1);  % Maximum loading case [N·m]
    base_moment_case2 = deflection_results.case2.moment(1);  % Minimum loading case [N·m]
    
    % Calculate bending stresses using beam theory: σ = My/I
    bending_stress_case1 = base_moment_case1 * base_distance_to_outer_fiber / base_moment_of_inertia;
    bending_stress_case2 = base_moment_case2 * base_distance_to_outer_fiber / base_moment_of_inertia;
    
    % ============================================================================
    % DIRECTIONAL STRESS ANALYSIS
    % ============================================================================
    % Account for wind direction differences between load cases
    % Case 1: Reference direction (0° - maximum loading)
    % Case 2: Different wind direction - stress reduced by cosine factor
    wind_direction_angle_case1 = 0;  % Case 1 is reference (maximum) condition
    wind_direction_angle_case2 = load_cases.case2.wind_direction - load_cases.case1.wind_direction;
    maximum_stress_case1 = bending_stress_case1 * cosd(wind_direction_angle_case1);  % = bending_stress_case1 (cos(0) = 1)
    maximum_stress_case2 = bending_stress_case2 * cosd(wind_direction_angle_case2);
    
    % Mohrs Circle Analysis (uniaxial stress state)
    principal_stress_1_case1 = maximum_stress_case1; principal_stress_2_case1 = 0; max_shear_stress_case1 = maximum_stress_case1 / 2;
    principal_stress_1_case2 = maximum_stress_case2; principal_stress_2_case2 = 0; max_shear_stress_case2 = maximum_stress_case2 / 2;
    
    % ============================================================================
    % FATIGUE ANALYSIS USING GOODMAN DIAGRAM METHODOLOGY
    % ============================================================================
    % Calculate modified endurance limit using Marin factors
    % S_n = S_e * k_a * k_b * k_c * k_d * k_e * k_f
    % where: k_a = surface factor, k_b = size factor, k_c = reliability factor
    ultimate_tensile_strength = data.materials.steel.tensileStrength;
    modified_endurance_limit = data.materials.steel.enduranceLimitFactor * ultimate_tensile_strength * ...
          data.materials.steel.fatigueFactors.sizeFactor * ...
          data.materials.steel.fatigueFactors.surfaceFactor * ...
          data.materials.steel.fatigueFactors.reliabilityFactor;  % Modified endurance limit
    
    % Calculate mean and alternating stress components for Goodman analysis
    mean_stress = (maximum_stress_case1 + maximum_stress_case2) / 2;        % Average stress [Pa]
    alternating_stress = abs(maximum_stress_case1 - maximum_stress_case2) / 2; % Stress amplitude [Pa]
    
    % Goodman fatigue safety factor: n = 1 / (σa/Se + σm/Sut)
    % This accounts for both mean stress and alternating stress effects
    fatigue_safety_factor = 1 / (alternating_stress/modified_endurance_limit + mean_stress/ultimate_tensile_strength);
    
    % ============================================================================
    % STATIC FAILURE ANALYSIS USING MULTIPLE FAILURE THEORIES
    % ============================================================================
    % Analyze static failure using Case 1 (maximum loading) conditions
    yield_strength = data.materials.steel.yieldStrength;
    normal_stress = maximum_stress_case1;  % Maximum normal stress [Pa]
    shear_stress = 0;                      % No shear stress in this analysis [Pa]
    
    % Maximum Normal Stress Theory (Rankine): n = Sy/σ
    safety_factor_maximum_normal_stress_theory = yield_strength / normal_stress;
    
    % Maximum Shear Stress Theory (Tresca): n = (Sy/2)/(σ/2) = Sy/σ
    safety_factor_maximum_shear_stress_theory = (yield_strength / 2) / (normal_stress / 2);
    
    % Distortion Energy Theory (von Mises): n = Sy/σeq where σeq = sqrt(σ² + 3τ²)
    safety_factor_distortion_energy_theory = yield_strength / sqrt(normal_stress^2 + 3*shear_stress^2);
    
    % Compile results
    stress_results = struct('mohr_case1', struct('sigma_1', principal_stress_1_case1, 'sigma_2', principal_stress_2_case1, ...
        'tau_max', max_shear_stress_case1, 'sigma_center', principal_stress_1_case1/2, 'radius', principal_stress_1_case1/2), ...
        'mohr_case2', struct('sigma_1', principal_stress_1_case2, 'sigma_2', principal_stress_2_case2, ...
        'tau_max', max_shear_stress_case2, 'sigma_center', principal_stress_1_case2/2, 'radius', principal_stress_1_case2/2), ...
        'case1', struct('sigma_bending', bending_stress_case1, 'sigma_axial', 0, ...
        'sigma_max', maximum_stress_case1, 'moment', base_moment_case1), ...
        'case2', struct('sigma_bending', bending_stress_case2, 'sigma_axial', 0, ...
        'sigma_max', maximum_stress_case2, 'moment', base_moment_case2), ...
        'sigma_mean', mean_stress, 'sigma_alt', alternating_stress, 'max_stress', [maximum_stress_case1, maximum_stress_case2], ...
        'fatigue_safety_factor', fatigue_safety_factor, ...
        'static_failure', struct('MNST_SF', safety_factor_maximum_normal_stress_theory, 'MSST_SF', safety_factor_maximum_shear_stress_theory, 'DET_SF', safety_factor_distortion_energy_theory, ...
        'tau_max', normal_stress/2, 'sigma_eq', sqrt(normal_stress^2 + 3*shear_stress^2), ...
        'min_SF', min([safety_factor_maximum_normal_stress_theory, safety_factor_maximum_shear_stress_theory, safety_factor_distortion_energy_theory])));
end

%% ============================================================================
%% PLOTTING FUNCTIONS
%% ============================================================================

function createTowerDeflectionPlot(deflection_results, ~, config)
% CREATETOWERDEFLECTIONPLOT Create tower deflection visualization
% Generates plots showing tower deflection profiles for different load cases
% with proper scaling and formatting.
%
% Inputs:
%   deflection_results - Deflection analysis results structure
%   section_props      - Tower section properties structure
%   config             - Configuration structure
    
    if ~config.plot_d5_deflection, return; end
    
    figure(5); set(gcf, 'Position', [100, 100, 800, 400]);
    plot(deflection_results.z, deflection_results.case1.deflection, [config.color_primary '-'], 'LineWidth', 2);
    hold on;
    plot(deflection_results.z, deflection_results.case2.deflection, [config.color_secondary '--'], 'LineWidth', 2);
    xlabel('Height (m)'); ylabel('Deflection (m)'); title('Tower Deflection Profile');
    legend('Load Case 1', 'Load Case 2', 'Location', 'best'); grid on;
    
    [max_deflection_case1, ~] = max(abs(deflection_results.case1.deflection));
    text(50, 0.35, sprintf('Max Deflection: %.3f m', max_deflection_case1), ...
        'FontSize', 10, 'FontWeight', 'bold', 'Color', config.color_primary, ...
        'BackgroundColor', 'white', 'EdgeColor', config.color_primary);
    
    if config.save_plots
        saveas(gcf, 'Tower_Deflection_Analysis.png');
        fprintf('Tower deflection plot saved as Tower_Deflection_Analysis.png\n');
    end
end

function createMohrCirclePlots(stress_results, config)
% CREATEMOHRCIRCLEPLOTS Create Mohrs circle visualizations
% Generates Mohrs circle plots for stress state analysis showing principal
% stresses and maximum shear stress for different load cases.
%
% Inputs:
%   stress_results - Stress analysis results structure
%   config         - Configuration structure
    
    if ~config.plot_d5_mohr, return; end
    
    figure(6); set(gcf, 'Position', [200, 200, 1200, 500]);
    
    subplot(1,2,1);
    plotMohrCircle(stress_results.case1.sigma_max, 0, 0, 'Load Case 1', config);
    
    subplot(1,2,2);
    plotMohrCircle(stress_results.case2.sigma_max, 0, 0, 'Load Case 2', config);
    
    if config.save_plots
        saveas(gcf, 'Mohr_Circle_Analysis.png');
        fprintf('Mohr circle plots saved as Mohr_Circle_Analysis.png\n');
    end
end

function plotMohrCircle(sigma_x, sigma_y, tau_xy, title_str, config)
% PLOTMOHRCIRCLE Plot Mohrs circle for given stress state
% Creates Mohrs circle visualization for a specific stress state showing
% principal stresses, maximum shear stress, and stress components.
%
% Inputs:
%   sigma_x     - Normal stress in x-direction [Pa]
%   sigma_y     - Normal stress in y-direction [Pa]
%   tau_xy      - Shear stress [Pa]
%   title_str   - Title string for the plot
%   config      - Configuration structure
    
    % Calculate center and radius
    sigma_center = (sigma_x + sigma_y) / 2;
    R = sqrt(((sigma_x - sigma_y) / 2)^2 + tau_xy^2);
    
    % Principal stresses
    sigma_1 = sigma_center + R;
    sigma_2 = sigma_center - R;
    
    % Create circle
    theta = linspace(0, 2*pi, 100);
    x_circle = sigma_center + R * cos(theta);
    y_circle = R * sin(theta);
    
    plot(x_circle/1e6, y_circle/1e6, [config.color_primary '-'], 'LineWidth', 2);
    hold on;
    
    % Plot principal stresses
    plot([sigma_1/1e6, sigma_2/1e6], [0, 0], 'ro', 'MarkerSize', 8, 'LineWidth', 2);
    
    % Plot center point
    plot(sigma_center/1e6, 0, 'ko', 'MarkerSize', 6);
    
    % Plot one stress state point
    plot(sigma_x/1e6, tau_xy/1e6, 'gs', 'MarkerSize', 8, 'LineWidth', 2);
    
    xlabel('Normal Stress (MPa)');
    ylabel('Shear Stress (MPa)');
    title(title_str);
    grid on;
    axis equal;
    
    % Add padding to prevent lines from touching plot edges
    xlim_current = xlim;
    ylim_current = ylim;
    xlim([xlim_current(1) - (xlim_current(2) - xlim_current(1)) * 0.1, xlim_current(2) + (xlim_current(2) - xlim_current(1)) * 0.1]);
    ylim([ylim_current(1) - (ylim_current(2) - ylim_current(1)) * 0.1, ylim_current(2) + (ylim_current(2) - ylim_current(1)) * 0.1]);
    
end

function createGoodmanDiagram(stress_results, S_ut, S_y, ~, config)
% CREATEGOODMANDIAGRAM Create Goodman diagram for fatigue analysis
% Generates Goodman diagram showing fatigue safety factors, endurance limits,
% and operating points for fatigue analysis of the tower structure.
%
% Inputs:
%   stress_results - Stress analysis results structure
%   S_ut          - Ultimate tensile strength [Pa]
%   S_y           - Yield strength [Pa]
%   data          - Wind turbine data structure
%   config        - Configuration structure
    
    if ~config.plot_d5_goodman, return; end
    
    figure(7); set(gcf, 'Position', [300, 300, 800, 600]);
    
    S_e = 0.5 * S_ut * 1.0 * 0.9 * 0.7;  % Modified endurance limit
    sigma_m = stress_results.sigma_mean; sigma_a = stress_results.sigma_alt;
    
    sigma_m_range = linspace(0, S_ut, 100);
    sigma_a_goodman = S_e * (1 - sigma_m_range / S_ut);
    
    plot(sigma_m_range/1e6, sigma_a_goodman/1e6, [config.color_primary '-'], 'LineWidth', 2, 'DisplayName', 'Goodman Line');
    hold on;
    plot(sigma_m/1e6, sigma_a/1e6, 'ko', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Operating Point');
    
    sigma_m_yield = linspace(0, S_y, 50);
    sigma_a_yield = S_y - sigma_m_yield;
    plot(sigma_m_yield/1e6, sigma_a_yield/1e6, [config.color_accent '-'], 'LineWidth', 2, 'DisplayName', 'Yield Line');
    
    if sigma_m > 0
        slope = sigma_a / sigma_m;
        x_prop = linspace(0, 130e6, 50);
        y_prop = slope * x_prop;
        plot(x_prop/1e6, y_prop/1e6, [config.color_secondary '--'], 'LineWidth', 1.5, 'DisplayName', 'Load Line');
    end
    
    xlabel('Mean Stress (MPa)'); ylabel('Alternating Stress (MPa)');
    title('Goodman Diagram for Fatigue Analysis'); legend('Location', 'best'); grid on;
    
    text(S_ut/1e6, S_e/1e6*0.1, 'S_{ut}', 'HorizontalAlignment', 'center', 'FontSize', 10, 'Color', config.color_primary);
    text(S_e/1e6*0.1, S_e/1e6, 'S_e', 'HorizontalAlignment', 'center', 'FontSize', 10, 'Color', config.color_primary);
    text(S_y/1e6, S_e/1e6*0.1, 'S_y', 'HorizontalAlignment', 'center', 'FontSize', 10, 'Color', config.color_accent);
    text(S_e/1e6*0.1, S_y/1e6, 'S_y', 'HorizontalAlignment', 'center', 'FontSize', 10, 'Color', config.color_accent);
    
    n_fatigue = stress_results.fatigue_safety_factor;
    text(100, 125, sprintf('Safety Factor: %.2f', n_fatigue), ...
        'BackgroundColor', 'white', 'EdgeColor', 'black');
    
    if config.save_plots
        saveas(gcf, 'Goodman_Diagram_Analysis.png');
        fprintf('Goodman diagram saved as Goodman_Diagram_Analysis.png\n');
    end
end

function createTowerAnalysisPlots(deflection_results, load_cases, section_props, data, config)
% CREATETOWERANALYSISPLOTS Create comprehensive tower analysis plots
% Generates vertically stacked plots showing load distribution, shear force,
% bending moment, slope, and deflection for both load cases, similar to HW02Q01.m
%
% Inputs:
%   deflection_results - Deflection analysis results structure
%   load_cases       - Loading scenarios structure
%   section_props    - Tower section properties structure
%   data             - Wind turbine data structure
%   config           - Configuration structure
    
    % Check if plotting is enabled
    if ~config.plot_d5_tower
        return;
    end
    
    % Get data
    z = deflection_results.z;
    tower_height = section_props.height(end);
    total_height = max(z);
    
    % Calculate distributed loads for both cases
    [~, drag_force_1, nacelle_load_1] = computeWindLoading(section_props, load_cases.case1, z, data);
    [~, drag_force_2, nacelle_load_2] = computeWindLoading(section_props, load_cases.case2, z, data);
    
    
    % Calculate shear forces from moment gradients
    dz = z(2) - z(1);
    shear_case1 = -diff(deflection_results.case1.moment) / dz;
    shear_case2 = -diff(deflection_results.case2.moment) / dz;
    z_shear = z(1:end-1) + dz/2;  % Shear at midpoints
    
    % Calculate slopes (first derivative of deflection)
    slope_case1 = diff(deflection_results.case1.deflection) / dz;
    slope_case2 = diff(deflection_results.case2.deflection) / dz;
    z_slope = z(1:end-1) + dz/2;  % Slope at midpoints
    
    % Figure 1: Load Distribution Cases
    figure('Position', [100, 100, 800, 400]);
    
    % Plot 1: Load Distribution - Case 1 (Dual Y-axis)
    subplot(2,1,1);
    hold off; % Ensure clean state
    yyaxis left;
    plot(z, drag_force_1, [config.color_accent '-'], 'LineWidth', 2);
    ylabel('Wind Drag (N/m)', 'Color', config.color_accent);
    ylim([0, max(drag_force_1)*1.1]);
    hold on;
    
    yyaxis right;
    plot(z, nacelle_load_1, [config.color_secondary '-'], 'LineWidth', 2);
    ylabel('Nacelle Load (N/m)', 'Color', config.color_secondary);
    if max(nacelle_load_1) > 1000
        ylim([0, max(nacelle_load_1)*1.1]);
    else
        ylim([0, 1000]); % Default scale if no nacelle load
    end
    ax = gca;
    ax.YAxis(2).Exponent = 3;     % exponent of 3
    ax.YAxis(2).TickLabelFormat = '%.0f';
    
    title('Tower Analysis - Load Case 1');
    grid on; xlim([0, total_height]);
    yyaxis left; % Switch to left axis for vertical line
    ylim([0, max(drag_force_1)*1.1]); % Ensure left axis has correct limits
    plot([tower_height, tower_height], ylim, 'k:', 'LineWidth', 1);
    
    % Plot 2: Load Distribution - Case 2 (Dual Y-axis)
    subplot(2,1,2);
    hold off; % Ensure clean state
    yyaxis left;
    plot(z, drag_force_2, 'g-', 'LineWidth', 2);
    ylabel('Wind Drag (N/m)', 'Color', 'g');
    ylim([0, max(drag_force_2)*1.1]);
    hold on;
    
    yyaxis right;
    plot(z, nacelle_load_2, 'r-', 'LineWidth', 2);
    ylabel('Nacelle Load (N/m)', 'Color', 'r');
    if max(nacelle_load_2) > 1000
        ylim([0, max(nacelle_load_2)*1.1]);
    else
        ylim([0, 1000]); % Default scale if no nacelle load
    end
    ax = gca;
    ax.YAxis(2).Exponent = 3;     % exponent of 3
    ax.YAxis(2).TickLabelFormat = '%.0f';
    
    title('Tower Analysis - Load Case 2');
    grid on; xlim([0, total_height]);
    yyaxis left; % Switch to left axis for vertical line
    ylim([0, max(drag_force_2)*1.1]); % Ensure left axis has correct limits
    plot([tower_height, tower_height], ylim, 'k:', 'LineWidth', 1);
    xlabel('Height (m)');
    
    % Save load distribution plot if enabled
    if config.save_plots
        saveas(gcf, 'Tower_Load_Distribution.png');
        fprintf('Tower load distribution plots saved as Tower_Load_Distribution.png\n');
    end
    
    % Figure 2: Deflection-Related Plots
    figure('Position', [200, 200, 800, 800]);
    
    % Plot 1: Shear Force Distribution - Both Cases
    subplot(4,1,1);
    h1 = plot(z_shear, shear_case1/1000, [config.color_primary '-'], 'LineWidth', 2); hold on;
    h2 = plot(z_shear, shear_case2/1000, [config.color_secondary '--'], 'LineWidth', 2);
    ylabel('Shear (kN)'); 
    title('Shear Force Distribution');
    grid on; xlim([0, total_height]);
    plot([tower_height, tower_height], ylim, 'k:', 'LineWidth', 1);
    plot([0, total_height], [0, 0], 'k--', 'LineWidth', 0.5);
    legend([h1, h2], {'Case 1', 'Case 2'}, 'Location', 'west');
    
    % Plot 2: Bending Moment Distribution - Both Cases
    subplot(4,1,2);
    h1 = plot(z, deflection_results.case1.moment/1e6, [config.color_primary '-'], 'LineWidth', 2); hold on;
    h2 = plot(z, deflection_results.case2.moment/1e6, [config.color_secondary '--'], 'LineWidth', 2);
    ylabel('Moment (MN·m)'); 
    title('Bending Moment Distribution');
    grid on; xlim([0, total_height]);
    plot([tower_height, tower_height], ylim, 'k:', 'LineWidth', 1);
    plot([0, total_height], [0, 0], 'k--', 'LineWidth', 0.5);
    legend([h1, h2], {'Case 1', 'Case 2'}, 'Location', 'west');
    
    % Plot 3: Slope Distribution - Both Cases
    subplot(4,1,3);
    h1 = plot(z_slope, slope_case1*1000, [config.color_primary '-'], 'LineWidth', 2); hold on;
    h2 = plot(z_slope, slope_case2*1000, [config.color_secondary '--'], 'LineWidth', 2);
    ylabel('Slope (mrad)'); 
    title('Tower Slope Distribution');
    grid on; xlim([0, total_height]);
    plot([tower_height, tower_height], ylim, 'k:', 'LineWidth', 1);
    plot([0, total_height], [0, 0], 'k--', 'LineWidth', 0.5);
    legend([h1, h2], {'Case 1', 'Case 2'}, 'Location', 'west');
    
    % Plot 4: Deflection Distribution - Both Cases
    subplot(4,1,4);
    h1 = plot(z, deflection_results.case1.deflection*1000, [config.color_primary '-'], 'LineWidth', 2); hold on;
    h2 = plot(z, deflection_results.case2.deflection*1000, [config.color_secondary '--'], 'LineWidth', 2);
    xlabel('Height (m)'); ylabel('Deflection (mm)'); 
    title('Tower Deflection Distribution');
    grid on; xlim([0, total_height]);
    plot([tower_height, tower_height], ylim, 'k:', 'LineWidth', 1);
    plot([0, total_height], [0, 0], 'k--', 'LineWidth', 0.5);
    legend([h1, h2], {'Case 1', 'Case 2'}, 'Location', 'west');
    
    
    % Save deflection-related plots if enabled
    if config.save_plots
        saveas(gcf, 'Tower_Deflection_Analysis.png');
        fprintf('Tower deflection analysis plots saved as Tower_Deflection_Analysis.png\n');
    end
end

function createVisualization(r_stations, dCP, dCT, CP, CT, ~, ~, ~, config)
% CREATEVISUALIZATION Create visualization of BEM analysis results
% Generates plots showing power and thrust distributions side by side
% with CP and CT values displayed on the power coefficient plot.
%
% Inputs:
%   r_stations  - Radial station positions [m]
%   dCP         - Differential power coefficient distribution
%   dCT         - Differential thrust coefficient distribution
%   CP          - Total power coefficient
%   CT          - Total thrust coefficient
%   ~           - Tip speed ratio (unused)
%   ~           - Wind speed [m/s] (unused)
%   ~           - Rotor speed [rad/s] (unused)
%   config      - Configuration structure
    
    if ~config.plot_d1_results
        return;
    end
    
    figure(1); set(gcf, 'Position', [100, 100, 1000, 400]);
    
    if ~isempty(dCP) && ~isempty(dCT)
        % Left plot: Power coefficient distribution with CP and CT values
        subplot(1, 2, 1);
        area(r_stations, dCP, 'FaceColor', 'blue', 'FaceAlpha', 0.3, 'EdgeColor', 'blue', 'LineWidth', 2);
        xlabel('Radius [m]');
        ylabel('Local C_P (per unit area)');
        title('Local Power Coefficient Distribution');
        % Format y-axis to avoid scientific notation
        set(gca, 'YTickLabel', num2str(get(gca, 'YTick')', '%.4f'));
        grid on;
        
        % Add CP value to the power coefficient plot
        text(0.05, 0.75, sprintf('C_P = %.4f', CP), 'Units', 'normalized', ...
            'FontSize', 10, 'FontWeight', 'bold', 'Color', 'blue', ...
            'BackgroundColor', 'white', 'EdgeColor', 'blue');
        
        % Right plot: Thrust coefficient distribution
        subplot(1, 2, 2);
        area(r_stations, dCT, 'FaceColor', 'red', 'FaceAlpha', 0.3, 'EdgeColor', 'red', 'LineWidth', 2);
        xlabel('Radius [m]');
        ylabel('Local C_T (per unit area)');
        title('Local Thrust Coefficient Distribution');
        % Format y-axis to avoid scientific notation
        set(gca, 'YTickLabel', num2str(get(gca, 'YTick')', '%.4f'));
        grid on;
        
        % Add CT value to the thrust coefficient plot
        text(0.05, 0.75, sprintf('C_T = %.4f', CT), 'Units', 'normalized', ...
            'FontSize', 10, 'FontWeight', 'bold', 'Color', 'red', ...
            'BackgroundColor', 'white', 'EdgeColor', 'red');
        
    else
        % Fallback for theoretical plots if no data available
        subplot(1, 2, 1);
        a_range = 0:0.01:0.5;
        CP_theory = 4*a_range.*(1-a_range).^2;
        plot(a_range, CP_theory, [config.color_primary '-'], 'LineWidth', 2);
        xlabel('Axial Induction Factor (a)');
        ylabel('Power Coefficient (C_P)');
        title('Theoretical C_P vs Axial Induction Factor');
        set(gca, 'YTickLabel', num2str(get(gca, 'YTick')', '%.3f'));
        grid on;
        
        subplot(1, 2, 2);
        CT_theory = 4*a_range.*(1-a_range);
        plot(a_range, CT_theory, 'r-', 'LineWidth', 2);
        xlabel('Axial Induction Factor (a)');
        ylabel('Thrust Coefficient (C_T)');
        title('Theoretical C_T vs Axial Induction Factor');
        set(gca, 'YTickLabel', num2str(get(gca, 'YTick')', '%.3f'));
        grid on;
    end
    
    sgtitle('Wind Turbine Analysis - Deliverable 1', 'FontSize', 16, 'FontWeight', 'bold');
    
    % Save plot if enabled
    if config.save_plots
        saveas(gcf, 'Deliverable1_Results.png');
        fprintf('Results visualization saved as Deliverable1_Results.png\n');
    end
end

function createPitchOptimizationPlot(pitch_range, CP_values, CT_values, optimal_pitch, CP_max, V_wind, lambda, config)
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
%   config       - Configuration structure
    
    if ~config.plot_d2_optimization, return; end
    
    figure(2); set(gcf, 'Position', [100, 100, 1000, 700]);
    
    yyaxis left;
    plot(pitch_range, CP_values, [config.color_primary '-o'], 'LineWidth', 2, 'MarkerSize', 4);
    ylabel('Coefficient of Power (C_P)'); ylim_left = ylim;
    
    yyaxis right;
    plot(pitch_range, CT_values, [config.color_secondary '-x'], 'LineWidth', 2, 'MarkerSize', 4);
    ylabel('Coefficient of Thrust (C_T)'); ylim_right = ylim;
    
    ylim_combined = [min(ylim_left(1), ylim_right(1)), max(ylim_left(2), ylim_right(2))];
    yyaxis left; ylim(ylim_combined);
    yyaxis right; ylim(ylim_combined);
    
    [~, optimal_idx] = min(abs(pitch_range - optimal_pitch));
    CT_at_optimal = CT_values(optimal_idx);
    
    hold on;
    xline(optimal_pitch, 'k--', 'LineWidth', 2, 'DisplayName', sprintf('Optimal \\theta = %.1f°', optimal_pitch));
    plot(optimal_pitch, CP_max, 'go', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', sprintf('Max C_P = %.4f', CP_max));
    
    yyaxis left;
    text(optimal_pitch + 0.5, CP_max + 0.075, sprintf('C_P = %.4f', CP_max), 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'blue', 'BackgroundColor', 'white');
    
    yyaxis right;
    text(optimal_pitch + 0.5, CT_at_optimal + 0.075, sprintf('C_T = %.4f', CT_at_optimal), 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'red', 'BackgroundColor', 'white');
    
    hold off;
    xlabel('Pitch Angle (degrees)');
    title(sprintf('Wind Turbine Pitch Optimization (V = %.1f m/s, \\lambda = %.1f)', V_wind, lambda));
    legend('C_P', 'C_T', sprintf('Optimal \\theta = %.1f°', optimal_pitch), 'Optimal C_P', 'Location', 'best'); grid on;
    
    if config.save_plots
        saveas(gcf, 'Pitch_Optimization_Results.png');
        fprintf('Pitch optimization results visualization saved as Pitch_Optimization_Results.png\n');
    end
end

function create3DOptimizationPlot(lambda_range, pitch_range, CP_matrix, optimal_lambda, optimal_pitch, CP_max, V_wind, config)
% CREATE3DOPTIMIZATIONPLOT Create 3D visualization of CP optimization
% Generates a 3D surface plot showing power coefficient versus tip speed ratio
% and pitch angle with optimal operating point highlighting.
%
% Inputs:
%   lambda_range   - Array of tip speed ratios
%   pitch_range    - Array of pitch angles [degrees]
%   CP_matrix      - Power coefficient matrix [lambda x pitch]
%   optimal_lambda - Optimal tip speed ratio
%   optimal_pitch  - Optimal pitch angle [degrees]
%   CP_max         - Maximum power coefficient
%   V_wind         - Wind speed [m/s]
%   config         - Configuration structure
    
    if ~config.plot_d3_optimization
        return;
    end
    
    figure(3); set(gcf, 'Position', [100, 100, 800, 600]);
    
    % Create meshgrid for 3D plotting (flipped axes)
    [PITCH, LAMBDA] = meshgrid(pitch_range, lambda_range);
    
    % 3D surface plot (flipped axes)
    surf(PITCH, LAMBDA, CP_matrix');
    xlabel('Pitch Angle (degrees)');
    ylabel('Tip Speed Ratio (\lambda)');
    zlabel('Power Coefficient (C_P)');
    title(sprintf('Wind Turbine 3D Performance Optimization (V = %.1f m/s)', V_wind)); %'
    shading interp;
    colorbar;
    
    % Highlight optimal point
    hold on;
    plot3(optimal_pitch, optimal_lambda, CP_max, 'ro', 'MarkerSize', 12, 'LineWidth', 3, 'MarkerFaceColor', 'red');
    hold off;
    
    % Add text annotation for optimal point (moved vertically by +0.75, pitch axis by +5)
    text(optimal_pitch + 5, optimal_lambda, CP_max + 0.75, ...
        sprintf('Max C_P = %.4f\nλ = %.3f\nθ = %.1f°', CP_max, optimal_lambda, optimal_pitch), ...
        'FontSize', 10, 'FontWeight', 'bold', 'Color', 'red', ...
        'BackgroundColor', 'white', 'EdgeColor', 'red');
    
    % Improve view angle
    view(45, 60);
    
    % Save plot if enabled
    if config.save_plots
        saveas(gcf, '2D_CP_Optimization_Results.png');
        fprintf('3D optimization results visualization saved as 2D_CP_Optimization_Results.png\n');
    end
end

function createPowerPitchPlot(pitch_range, P_pitch, pitch_req, P_req, ratedPower, V_wind, config)
% CREATEPOWERPITCHPLOT Create visualization of power vs pitch angle
% Generates plot showing power output versus pitch angle with rated power
% constraint and optimal operating point highlighting.
%
% Inputs:
%   pitch_range  - Array of pitch angles [degrees]
%   P_pitch      - Power output at each pitch angle [W]
%   pitch_req    - Required pitch angle [degrees]
%   P_req        - Power at required pitch angle [W]
%   ratedPower   - Rated power limit [W]
%   V_wind       - Wind speed [m/s]
%   config       - Configuration structure
    
    if ~config.plot_d4_power, return; end
    
    figure(4); set(gcf, 'Position', [100, 100, 600, 500]);
    plot(pitch_range, P_pitch/1e3, [config.color_primary '-'], 'LineWidth', 2); hold on;
    yline(ratedPower/1e3, 'r--', 'Rated');
    plot(pitch_req, P_req/1e3, 'ko', 'MarkerSize', 8, 'LineWidth', 2);
    text(pitch_req + 0.5, 2750, sprintf('Pitch: %.1f°', pitch_req), ...
        'FontSize', 10, 'FontWeight', 'bold', 'Color', 'black', ...
        'BackgroundColor', 'white', 'EdgeColor', 'black');
    xlabel('Pitch (deg)'); ylabel('Power (kW)'); 
    title(sprintf('Power vs Pitch (V = %.1f m/s)', V_wind)); grid on;
    
    if config.save_plots
        saveas(gcf, 'Deliverable4_Power_vs_Pitch.png');
        fprintf('Power vs Pitch plot saved as Deliverable4_Power_vs_Pitch.png\n');
    end
end

%% ============================================================================
%% DATA EXTRACTION FUNCTIONS
%% ============================================================================

function data = ParameterExtraction(config)
% PARAMETEREXTRACTION Extracts all data from the Given Parameters folder
% and organizes it into a structured format for wind turbine analysis.
%
% Syntax:
%   data = ParameterExtraction(config)
%
% Input Arguments:
%   config - Configuration structure with parameters_path field
%
% Output Arguments:
%   data - Structure containing all extracted parameters organized by category
%     .blade              - Blade profile data and geometry
%     .tower              - Tower specifications and properties
%     .airfoils           - Airfoil coordinate data
%     .airfoilPerformance - Airfoil performance data (CL, CD, CM vs AoA)
%     .materials          - Material properties (air, steel)
%     .turbine            - Wind turbine specifications
%     .deliverables       - Predefined parameters for each deliverable
%     .metadata           - Extraction metadata and timestamps
%
% Description:
%   Comprehensive data extraction function that reads all wind turbine
%   parameters from CSV and DAT files, organizes them into structured
%   format, and adds calculated parameters for analysis.
%
% Example:
%   config.parameters_path = 'Auxilary Information/Given Parameters/';
%   data = ParameterExtraction(config);
%   fprintf('Turbine: %s\n', data.turbine.model);

    % ============================================================================
    % DATA EXTRACTION SETUP
    % ============================================================================
    basePath = config.parameters_path;  % Base path to parameters folder
    data = struct();  % Initialize main data structure
    
    try
        % ============================================================================
        % PRIMARY DATA EXTRACTION
        % ============================================================================
        % Extract all major data components from their respective files
        data.blade = extractBladeProfile(fullfile(basePath, 'BladeProfile.csv'));  % Blade geometry and airfoil assignments
        data.tower = extractTowerSpecs(fullfile(basePath, 'towerSpecs.csv'));       % Tower structural specifications
        data.airfoils = extractAirfoilCoordinates(basePath);                        % Airfoil coordinate data (.dat files)
        data.airfoilPerformance = extractAirfoilPerformance(basePath);             % Airfoil performance data (.csv files)
        data.materials = extractMaterialProperties();                               % Material properties (air, steel)
        data.turbine = extractTurbineSpecifications();                             % Wind turbine specifications (Clipper Liberty C96)

        % ============================================================================
        % DELIVERABLE-SPECIFIC PARAMETERS
        % ============================================================================
        % Predefined operating conditions for each deliverable
        % These match the requirements specified in the project documentation
        data.deliverables = struct('part1', struct('V_wind', 10, 'omega_rpm', 14, 'pitch_deg', 0), ...
            'part2', struct('V_wind', 8, 'lambda', 6.91), ...
            'part3', struct('V_wind', 6), ...
            'part4', struct('V_wind', 14.6));
        
        % ============================================================================
        % METADATA AND DOCUMENTATION
        % ============================================================================
        % Add extraction metadata for traceability and documentation
        data.metadata = struct('extractionDate', datetime('now'), 'sourceFolder', basePath, ...
            'description', 'Wind turbine parameters extracted from Given Parameters folder');
        
    catch ME
        % Error handling for file reading failures
        error('Parameter extraction failed: %s', ME.message);
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
    
    bladeTable = readtable(filePath, detectImportOptions(filePath));
    bladeData = struct('profile', bladeTable, 'description', 'Blade profile data including geometry, twist, and airfoil assignments', ...
        'airfoilTypes', unique(bladeTable.Airfoil), 'totalLength', max(bladeTable.DistanceFromCenterOfRotation), ...
        'stations', height(bladeTable));
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
    
    towerTable = readtable(filePath, detectImportOptions(filePath));
    towerData = struct('specs', towerTable, 'description', 'Tower specifications including height, diameter, and wall thickness', ...
        'dragCoefficient', 0.7, 'totalHeight', max(towerTable.Height_mm_) / 1000, ...
        'sections', height(towerTable), 'baseDiameter', towerTable.OD_mm_(1) / 1000, ...
        'topDiameter', towerTable.OD_mm_(end) / 1000);
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
    airfoilFiles = {'180.dat', '210.dat', '250.dat', '300.dat'};
    airfoilNames = {'DU96_W_180', 'DU93_W_210', 'DU91_W2_250', 'DU97_W_300'};
    
    for i = 1:length(airfoilFiles)
        filePath = fullfile(basePath, airfoilFiles{i});
        if exist(filePath, 'file')
            try
                fid = fopen(filePath, 'r');
                lines = cell(1000, 1); line_count = 0;
                line = fgetl(fid);
                while ischar(line)
                    if ~startsWith(strtrim(line), '%')
                        line_count = line_count + 1; lines{line_count} = line;
                    end
                    line = fgetl(fid);
                end
                fclose(fid); lines = lines(1:line_count);
                
                coords = zeros(length(lines), 2);
                for j = 1:length(lines)
                    coords(j, :) = str2double(strsplit(lines{j}));
                end
                
                airfoilData.(airfoilNames{i}) = struct('x', coords(:, 1), 'y', coords(:, 2), ...
                    'description', sprintf('Airfoil coordinates for %s', airfoilNames{i}), 'numPoints', length(coords));
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
    perfFiles = {'DU96-W-180.csv', 'DU93-W-210.csv', 'DU91-W2-250.csv', 'DU97-W-300.csv'};
    perfNames = {'DU96_W_180', 'DU93_W_210', 'DU91_W2_250', 'DU97_W_300'};
    
    for i = 1:length(perfFiles)
        filePath = fullfile(basePath, perfFiles{i});
        if exist(filePath, 'file')
            try
                perfTable = readtable(filePath, detectImportOptions(filePath));
                performanceData.(perfNames{i}) = struct('data', perfTable, 'AoA', perfTable.AoA, ...
                    'CL', perfTable.CL, 'CD', perfTable.CD, 'CM', perfTable.CM, ...
                    'description', sprintf('Performance data for %s', perfNames{i}), ...
                    'numPoints', height(perfTable), 'maxCL', max(perfTable.CL), ...
                    'minCD', min(perfTable.CD), 'AoARange', [min(perfTable.AoA), max(perfTable.AoA)]);
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
    
    materialData = struct('air', struct('density', 1.1, 'viscosity', 1.8e-5, ...
        'description', 'Air properties for aerodynamic calculations', ...
        'units', struct('density', 'kg/m³', 'viscosity', 'Ns/m²')), ...
        'steel', struct('type', 'ASTM A572, Grade 50', 'density', 7850, ...
        'tensileStrength', 450e6, 'yieldStrength', 345e6, 'youngsModulus', 200e9, ...
        'description', 'Steel properties for structural calculations', ...
        'units', struct('density', 'kg/m³', 'tensileStrength', 'Pa', 'yieldStrength', 'Pa', 'youngsModulus', 'Pa'), ...
        'safetyFactor', 1.5, 'allowableStress', 345e6/1.5, 'enduranceLimitFactor', 0.5, ...
        'fatigueFactors', struct('sizeFactor', 1.0, 'surfaceFactor', 0.9, 'reliabilityFactor', 0.7)));
end

function turbineData = extractTurbineSpecifications()
% EXTRACTTURBINESPECIFICATIONS Extract wind turbine specifications
% Defines comprehensive wind turbine specifications for Clipper Liberty C96
% including performance characteristics, dimensions, and calculated parameters.
%
% Outputs:
%   turbineData - Structure containing complete turbine specifications
    
    turbineData = struct('model', 'Clipper Liberty C96 2.5 MW', 'location', 'Rosemount, MN', ...
        'description', 'UMN Clipper C96 Wind Turbine specifications', ...
        'characteristics', struct('blades', 3, 'orientation', 'upwind', ...
        'control', 'pitch controlled, variable speed', 'yawControl', true, 'yawAngle', 0), ...
        'performance', struct('ratedPower', 2.5e6, 'bladeRadius', 48, 'rotorRadius', 48, ...
        'hubHeight', 80.4, 'hubRadius', 1.3, 'cutInSpeed', 4, 'ratedSpeed', 11, ...
        'cutOutSpeed', 25, 'rotorSpeedRange', [9.6, 15.5]), ...
        'calculated', struct('rotorArea', pi * 48^2, 'sweptArea', pi * 48^2, ...
        'powerDensity', 2.5e6 / (pi * 48^2), 'rotorSpeedRange_rads', [9.6, 15.5] * 2 * pi / 60), ...
        'units', struct('power', 'W', 'length', 'm', 'speed', 'm/s', 'rotorSpeed', 'RPM', 'angle', 'degrees'));
end

% ============================================================================
% CODE SUMMARY AND CAPABILITIES
% ============================================================================
%
% This comprehensive wind turbine analysis code provides the following capabilities:
%
% AERODYNAMIC ANALYSIS:
% - Blade Element Momentum (BEM) theory implementation
% - Airfoil performance interpolation and analysis
% - Power and thrust coefficient calculations
% - 2D optimization over tip speed ratio and pitch angle
% - Rated power control analysis
%
% STRUCTURAL ANALYSIS:
% - Tower deflection analysis under wind loading
% - Stress analysis using beam theory
% - Mohrs circle stress state visualization
% - Fatigue analysis using Goodman diagram methodology
% - Multiple static failure theories (Rankine, Tresca, von Mises)
% - Safety factor calculations for both static and fatigue failure
%
% KEY FEATURES:
% - Parameterized design (no hardcoded values)
% - Realistic turbine specifications (Clipper Liberty C96)
% - Dynamic lambda range calculation from actual speed limits
% - Comprehensive visualization suite
% - Modular design for easy modification and extension
%
% DELIVERABLES:
% D1: Basic BEM Analysis - CP and CT calculation
% D2: Pitch Optimization - Optimal pitch angle finding
% D3: 2D CP Optimization - Global maximum power coefficient
% D4: Rated Power Control - Pitch for rated power output
% D5: Structural Analysis - Tower deflection and stress analysis
%
% The code uses actual wind turbine specifications and material properties
% to provide realistic analysis results suitable for engineering applications.
% ============================================================================



