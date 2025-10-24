function result = Deliverable4()
% DELIVERABLE4 Rated Power Pitch Control
% Determine the blade pitch angle required to keep turbine power <= rated
% at the predefined high wind speed in ParameterExtraction (part4).
%
% Usage: result = Deliverable4();
% Returns struct with fields: pitch_deg, lambda, omega_rad, omega_rpm, P, CP

    clc; close all;
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

                % Apply Prandtl loss to loads as well
                F = prandtlLossFactor(B, r, R, lambda_r, a, a_prime);
                Cn = F * Cn; Ct = F * Ct;

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
    figure('Position', [100, 100, 600, 500]);
    plot(pitch_range, P_pitch/1e3, 'b-', 'LineWidth', 2); hold on;
    yline(ratedPower/1e3, 'r--', 'Rated');
    plot(pitch_req, P_req/1e3, 'ko', 'MarkerSize', 8, 'LineWidth', 2);
    xlabel('Pitch (deg)'); ylabel('Power (kW)'); title(sprintf('Power vs Pitch (V = %.1f m/s)', V_wind)); grid on;
end

% ===== Helpers (adapted from Deliverables 2/3) =====
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
        % Apply Prandtl factor to loads in caller for clarity
    end
end

function [a, a_prime, CL, CD, Cn, Ct, V_rel] = solveBEMSection(r, c, twist_rad, perf_data, lambda_r, V_wind, omega_rad, pitch_rad)
% Closed-form induction factors and sectional loads (no iteration)
    a = 1/3;
    a_prime = -0.5 + 0.5 * sqrt(1 + (4/(lambda_r^2)) * a * (1 - a));

    phi = atan((1 - a) / ((1 + a_prime) * lambda_r));
    alpha = phi - (twist_rad + pitch_rad);
    alpha_deg = rad2deg(alpha);

    % Clamp AoA to polar range
    aoa = perf_data.AoA; clv = perf_data.CL; cdv = perf_data.CD;
    [aoa_sorted, idx] = sort(aoa); cl_sorted = clv(idx); cd_sorted = cdv(idx);
    alpha_clamped = min(max(alpha_deg, aoa_sorted(1)), aoa_sorted(end));
    CL = interp1(aoa_sorted, cl_sorted, alpha_clamped, 'linear');
    CD = interp1(aoa_sorted, cd_sorted, alpha_clamped, 'linear');

    s = sin(phi); cphi = cos(phi);
    Cn = CL * cphi + CD * s;
    Ct = CL * s - CD * cphi;

    V_rel = sqrt((V_wind*(1-a))^2 + (omega_rad*r*(1+a_prime))^2);
end

function [a, a_prime, CL, CD, Cn, Ct, V_rel] = solveCircleSection(r, c, twist_rad, lambda_r, V_wind, omega_rad, pitch_rad, rho, mu)
    a = 1/3; a_prime = -0.5 + 0.5 * sqrt(1 + (4/(lambda_r^2)) * a * (1 - a));
    phi = atan((1 - a) / ((1 + a_prime) * lambda_r)); %#ok<NASGU>
    V_rel = sqrt((V_wind*(1-a))^2 + (omega_rad*r*(1+a_prime))^2);
    Re = max(1, rho * V_rel * c / mu);
    CD = cylinderCDlocal(Re); CL = 0;
    s = sin(atan((1 - a) / ((1 + a_prime) * lambda_r))); cphi = cos(atan((1 - a) / ((1 + a_prime) * lambda_r)));
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

