function [CP_max, optimal_conditions] = Deliverable2(V_wind, lambda, pitch_min, pitch_max, pitch_step)
% DELIVERABLE2 Wind Turbine Pitch Angle Optimization
% Finds maximum power coefficient by varying pitch angle at fixed wind speed and tip speed ratio
% Usage: [CP_max, optimal_conditions] = Deliverable2(10, 7, -5, 15, 1);

    clc; close all;
    fprintf('=== Wind Turbine Pitch Angle Optimization ===\n\n');
    
    fprintf('Optimizing CP for:\n');
    fprintf('  Wind velocity: %.1f m/s\n', V_wind);
    fprintf('  Tip speed ratio: %.1f\n', lambda);
    fprintf('  Pitch angle range: %.1f to %.1f degrees\n', pitch_min, pitch_max);
    fprintf('  Step size: %.1f degrees\n\n', pitch_step);
    
    fprintf('Extracting wind turbine parameters...\n');
    data = ParameterExtraction();
    
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
    n_stations = height(blade_profile);
    r_stations = blade_profile.DistanceFromCenterOfRotation / 1000;
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
        
        for i = 1:n_stations
            r = r_stations(i);
            c = blade_profile.ChordLength(i) / 1000;
            twist = blade_profile.BladeTwist(i);
            airfoil = blade_profile.Airfoil{i};
            
            lambda_r = lambda * r / R;
            
            [a, a_prime, CL, CD, Cn, Ct] = solveBEMIteration(r, c, twist, airfoil, ...
                                                             lambda_r, V_wind, omega_rad, ...
                                                             data.airfoilPerformance, rho, ...
                                                             data.materials.air.viscosity, pitch_angle);
            
            phi = atan((1-a)*V_wind / ((1+a_prime)*omega_rad*r));
            V_rel = sqrt((V_wind*(1-a))^2 + (omega_rad*r*(1+a_prime))^2);
            
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

function [a, a_prime, CL, CD, Cn, Ct] = solveBEMIteration(r, c, twist, airfoil, lambda_r, V_wind, omega_rad, airfoilData, rho, mu, pitch_angle)
% Robust BEM iteration solver
    
    airfoil_name = strrep(airfoil, '-', '_');
    if isfield(airfoilData, airfoil_name)
        perf_data = airfoilData.(airfoil_name);
    else
        perf_data = airfoilData.DU96_W_180;
    end
    
    a = 0.1;
    a_prime = 0.01;
    
    max_iter = 50;
    tolerance = 1e-4;
    relaxation = 0.2;
    
    for iter = 1:max_iter
        a_old = a;
        a_prime_old = a_prime;
        
        phi = atan((1-a)*V_wind / ((1+a_prime)*omega_rad*r));
        alpha = phi - deg2rad(twist) - deg2rad(pitch_angle);
        alpha_deg = rad2deg(alpha);
        
        CL = interp1(perf_data.AoA, perf_data.CL, alpha_deg, 'linear', 'extrap');
        CD = interp1(perf_data.AoA, perf_data.CD, alpha_deg, 'linear', 'extrap');
        
        Cn = CL * cos(phi) + CD * sin(phi);
        Ct = CL * sin(phi) - CD * cos(phi);
        
        sigma = c / (2 * pi * r);
        
        if Cn > 0 && sigma > 0
            a_new = 1 / (1 + 4*sin(phi)^2 / (sigma * Cn));
        else
            a_new = 0;
        end
        
        if Ct > 0 && sigma > 0
            a_prime_new = 1 / (4*sin(phi)*cos(phi) / (sigma * Ct) - 1);
        else
            a_prime_new = 0;
        end
        
        a_new = max(0, min(a_new, 0.5));
        a_prime_new = max(0, min(a_prime_new, 1.0));
        
        a = a + relaxation * (a_new - a);
        a_prime = a_prime + relaxation * (a_prime_new - a_prime);
        
        if abs(a - a_old) < tolerance && abs(a_prime - a_prime_old) < tolerance
            break;
        end
        
        if iter > 10 && (abs(a - a_old) > 0.1 || abs(a_prime - a_prime_old) > 0.1)
            relaxation = relaxation * 0.9;
        end
    end
    
    phi = atan((1-a)*V_wind / ((1+a_prime)*omega_rad*r));
    alpha = phi - deg2rad(twist) - deg2rad(pitch_angle);
    alpha_deg = rad2deg(alpha);
    
    CL = interp1(perf_data.AoA, perf_data.CL, alpha_deg, 'linear', 'extrap');
    CD = interp1(perf_data.AoA, perf_data.CD, alpha_deg, 'linear', 'extrap');
    
    Cn = CL * cos(phi) + CD * sin(phi);
    Ct = CL * sin(phi) - CD * cos(phi);
    
    if iter == max_iter
        fprintf('Warning: BEM did not converge at r=%.1f: a=%.3f, a''=%.3f\n', r, a, a_prime);
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
