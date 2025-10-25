function [CP_max, optimal_conditions] = Deliverable3_Experimental(V_wind, lambda_min, lambda_max, lambda_step, pitch_min, pitch_max, pitch_step)
% DELIVERABLE3 Wind Turbine 2D CP Optimization
% Finds maximum power coefficient by varying tip speed ratio and pitch angle
% Usage: [CP_max, optimal_conditions] = Deliverable3(10, 3, 12, 0.5, -5, 15, 2);

    clc; close all;
    fprintf('=== Wind Turbine 2D CP Optimization ===\n\n');
    
    fprintf('Extracting wind turbine parameters...\n');
    data = ParameterExtraction();

    % Defaults from predefined deliverables when inputs are omitted
    if nargin < 1 || isempty(V_wind), V_wind = data.deliverables.part3.V_wind; end
    if nargin < 2 || isempty(lambda_min), lambda_min = 3; end
    if nargin < 3 || isempty(lambda_max), lambda_max = 50; end
    if nargin < 4 || isempty(lambda_step), lambda_step = 1; end
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
    
    % Debug logging (disabled by default). Set DEBUG=true to enable.
    DEBUG = true;
    DEBUG_FILE = 'Deliverable3_DebugLog.txt';
    if DEBUG
        debugInit(DEBUG_FILE, 'j k i lambda pitch r c a a_prime CL CD Cn Ct V_rel dT dQ dP');
    end
    blade_profile = data.blade.profile;
    % Precompute invariants for clarity and speed
    r_stations = blade_profile.DistanceFromCenterOfRotation / 1000; % station radii [m]
    chord_m = blade_profile.ChordLength / 1000; % chord lengths [m]
    twist_deg_vec = blade_profile.BladeTwist; % twist [deg]
    airfoil_raw = data.blade.profile.Airfoil; % raw airfoil labels

    % Exclude inner hub region: remove stations with r < hub radius
    HUB_EXCLUDE = true;
    if isfield(data.turbine.performance, 'hubRadius')
        hub_radius = data.turbine.performance.hubRadius; % [m]
    elseif isfield(data.turbine.performance, 'hubDiameter')
        hub_radius = data.turbine.performance.hubDiameter / 2; % assume meters
    else
        hub_radius = 0.2 * R; % fallback: 20% of rotor radius
    end
    if HUB_EXCLUDE
        use_idx = r_stations >= hub_radius;
        r_stations = r_stations(use_idx);
        chord_m = chord_m(use_idx);
        twist_deg_vec = twist_deg_vec(use_idx);
        airfoil_raw = airfoil_raw(use_idx);
    end
    n_stations = numel(r_stations); % number of spanwise blade stations (rows in profile)
    B = 3;
    
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
                    lambda_r, V_wind, omega_rad, pitch_rad, data, rho, data.materials.air.viscosity, B, R);
                
                % Apply Prandtl tip and root loss factor F to sectional forces
                F = prandtlLossFactor(B, r, R, lambda_r, a, a_prime);
                Cn = F * Cn;
                Ct = F * Ct;
                
                dT(i) = 0.5 * rho * V_rel^2 * c * Cn;
                dQ(i) = 0.5 * rho * V_rel^2 * c * Ct * r;
                dP(i) = dQ(i) * omega_rad;
                
                if DEBUG
                    debugLog(DEBUG_FILE, '%d %d %d %.3f %.2f %.3f %.3f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n', ...
                        j, k, i, lambda, pitch_angle, r, c, a, a_prime, CL, CD, Cn, Ct, V_rel, dT(i), dQ(i), dP(i));
                end
            end
            
            T_total = B * trapz(r_stations, dT);
            Q_total = B * trapz(r_stations, dQ);
            P_total = B * trapz(r_stations, dP);
            
            CP_matrix(k, j) = P_total / (0.5 * rho * A * V_wind^3);
            CT_matrix(k, j) = T_total / (0.5 * rho * A * V_wind^2);
            P_matrix(k, j) = P_total;
            T_matrix(k, j) = T_total;
            
            if mod(current_iteration, 20) == 0 || current_iteration == total_iterations
                fprintf('  Progress: %.1f%% (λ=%.1f, θ=%.1f°)\n', current_iteration/total_iterations*100, lambda, pitch_angle);
            end
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
    
    create2DOptimizationPlot(lambda_range, pitch_range, CP_matrix, CT_matrix, optimal_lambda, optimal_pitch, CP_max, V_wind);
    fprintf('\n2D optimization complete!\n');
end

function F = prandtlLossFactor(B, r, R, lambda_r, a, a_prime)
% Prandtl tip and root loss correction for axial/tangential induction
% Returns a factor F in (0,1] applied to normal/tangential coefficients
    % Guard small angles; compute inflow angle from λ_r, a, a_prime
    phi = atan((1 - a) / ((1 + a_prime) * lambda_r + eps));
    sphi = sin(abs(phi)) + eps; % avoid division by zero
    mu = r / R;                 % nondimensional radius
    f_tip = (B/2) * (1 - mu) / sphi;
    f_root = (B/2) * (mu) / sphi;
    F_tip = (2/pi) * acos(exp(-max(0, f_tip)));
    F_root = (2/pi) * acos(exp(-max(0, f_root)));
    F = F_tip * F_root;
end

function debugInit(filePath, header)
% Initialize a debug log file with column header
    fid = fopen(filePath, 'w');
    if fid ~= -1
        fprintf(fid, '%s\n', header);
        fclose(fid);
    end
end

function debugLog(filePath, fmt, varargin)
% Append a single formatted line to the debug log; also echo to console
    fid = fopen(filePath, 'a');
    if fid ~= -1
        fprintf(fid, fmt, varargin{:});
        fclose(fid);
    end
    fprintf(fmt, varargin{:});
end

function [a, a_prime, CL, CD, Cn, Ct, V_rel] = solveBEMSection(r, c, twist_rad, perf_data, lambda_r, V_wind, omega_rad, pitch_rad)
% Closed-form induction factors and sectional loads (no iteration)
    a = 1/3;
    a_prime = -0.5 + 0.5 * sqrt(1 + (4/(lambda_r^2)) * a * (1 - a));

    % No caps on induction factors

    phi = atan((1 - a) / ((1 + a_prime) * lambda_r));
    alpha = phi - (twist_rad + pitch_rad);
    alpha_deg = rad2deg(alpha);

    % Interpolate coefficients from airfoil data
    aoa = perf_data.AoA; clv = perf_data.CL; cdv = perf_data.CD;
    [aoa_sorted, idx] = sort(aoa);
    cl_sorted = clv(idx); cd_sorted = cdv(idx);
    CL = interp1(aoa_sorted, cl_sorted, alpha_deg, 'linear', 'extrap');
    CD = interp1(aoa_sorted, cd_sorted, alpha_deg, 'linear', 'extrap');

    s = sin(phi); c = cos(phi);
    Cn = CL * c + CD * s;
    Ct = CL * s - CD * c;

    V_rel = sqrt((V_wind*(1-a))^2 + (omega_rad*r*(1+a_prime))^2);
end

function [a, a_prime, CL, CD, Cn, Ct, V_rel] = solveCircleSection(r, c, twist_rad, lambda_r, V_wind, omega_rad, pitch_rad, rho, mu)
    a = 1/3;
    a_prime = -0.5 + 0.5 * sqrt(1 + (4/(lambda_r^2)) * a * (1 - a));

    % No caps on induction factors
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

function create2DOptimizationPlot(lambda_range, pitch_range, CP_matrix, CT_matrix, optimal_lambda, optimal_pitch, CP_max, V_wind)
% Create 2D visualization of CP and CT vs Tip Speed Ratio and Pitch Angle
    
    figure('Position', [100, 100, 1200, 800]);
    
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
    
    saveas(gcf, '2D_CP_Optimization_Results.png');
    fprintf('2D optimization results visualization saved as 2D_CP_Optimization_Results.png\n');
end

function [a, a_prime, CL, CD, Cn, Ct, V_rel] = getSectionCoefficients(airfoil, twist_deg, r, c, lambda_r, V_wind, omega_rad, pitch_rad, data, rho, mu, B, R)
    airfoil_name = regexprep(airfoil, '\s+', '');
    airfoil_name = regexprep(airfoil_name, '[-–—]', '_');
    USE_ITERATIVE_BEM = true; % toggle momentum-consistent a, a_prime
    if strcmpi(airfoil_name, 'circle')
        twist_rad = deg2rad(twist_deg);
        [a, a_prime, CL, CD, Cn, Ct, V_rel] = solveCircleSection(r, c, twist_rad, ...
            lambda_r, V_wind, omega_rad, pitch_rad, rho, mu);
    else
        perf_data = data.airfoilPerformance.(airfoil_name);
        twist_rad = deg2rad(twist_deg);
        if USE_ITERATIVE_BEM
            [a, a_prime, CL, CD, Cn, Ct, V_rel] = solveBEMIterative(r, c, twist_rad, perf_data, ...
                lambda_r, V_wind, omega_rad, pitch_rad, B, R, rho, mu);
        else
            [a, a_prime, CL, CD, Cn, Ct, V_rel] = solveBEMSection(r, c, twist_rad, perf_data, ...
                lambda_r, V_wind, omega_rad, pitch_rad);
        end
    end
end

function [a, a_prime, CL, CD, Cn, Ct, V_rel] = solveBEMIterative(r, c, twist_rad, perf_data, lambda_r, V_wind, omega_rad, pitch_rad, B, R, rho, mu)
% Momentum-consistent iterative BEM with Prandtl losses and AoA clamp
    a = 0.1; a_prime = 0.01;
    max_iter = 50; tol = 1e-4; relax = 0.5;
    for it = 1:max_iter
        phi = atan((1 - a) / ((1 + a_prime) * lambda_r + eps));
        alpha_deg = rad2deg(phi - (twist_rad + pitch_rad));
        [aoa_sorted, idx] = sort(perf_data.AoA); cl_sorted = perf_data.CL(idx); cd_sorted = perf_data.CD(idx);
        CL = interp1(aoa_sorted, cl_sorted, alpha_deg, 'linear', 'extrap');
        CD = interp1(aoa_sorted, cd_sorted, alpha_deg, 'linear', 'extrap');
        s = sin(phi); cphi = cos(phi);
        Cn = CL * cphi + CD * s; Ct = CL * s - CD * cphi;
        F = prandtlLossFactor(B, r, R, lambda_r, a, a_prime);
        sigma = (B * c) / (2*pi*r + eps);
        a_new = 1 / (1 + (4*F*s*s)/(sigma * max(1e-8, Cn)));
        a_prime_new = 1 / ((4*F*s*cphi)/(sigma * max(1e-8, Ct)) - 1);
        % No caps on induction factors
        a = a + relax*(a_new - a);
        a_prime = a_prime + relax*(a_prime_new - a_prime);
        if abs(a_new - a) < tol && abs(a_prime_new - a_prime) < tol
            break;
        end
    end
    phi = atan((1 - a) / ((1 + a_prime) * lambda_r + eps));
    alpha_deg = rad2deg(phi - (twist_rad + pitch_rad));
    [aoa_sorted, idx] = sort(perf_data.AoA); cl_sorted = perf_data.CL(idx); cd_sorted = perf_data.CD(idx);
    CL = interp1(aoa_sorted, cl_sorted, alpha_deg, 'linear', 'extrap');
    CD = interp1(aoa_sorted, cd_sorted, alpha_deg, 'linear', 'extrap');
    s = sin(phi); cphi = cos(phi);
    Cn = CL * cphi + CD * s; Ct = CL * s - CD * cphi;
    F = prandtlLossFactor(B, r, R, lambda_r, a, a_prime);
    Cn = F * Cn; Ct = F * Ct;
    V_rel = sqrt((V_wind*(1-a))^2 + (omega_rad*r*(1+a_prime))^2);
end