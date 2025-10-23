function [CP_max, optimal_conditions] = Deliverable3(V_wind, lambda_min, lambda_max, lambda_step, pitch_min, pitch_max, pitch_step)
% DELIVERABLE3 Wind Turbine 2D CP Optimization
% Finds maximum power coefficient by varying tip speed ratio and pitch angle
% Usage: [CP_max, optimal_conditions] = Deliverable3(10, 3, 12, 0.5, -5, 15, 2);

    clc; close all;
    fprintf('=== Wind Turbine 2D CP Optimization ===\n\n');
    
    fprintf('Optimizing CP for:\n');
    fprintf('  Wind velocity: %.1f m/s\n', V_wind);
    fprintf('  Tip speed ratio range: %.1f to %.1f (step: %.1f)\n', lambda_min, lambda_max, lambda_step);
    fprintf('  Pitch angle range: %.1f to %.1f degrees (step: %.1f)\n', pitch_min, pitch_max, pitch_step);
    
    fprintf('Extracting wind turbine parameters...\n');
    data = ParameterExtraction();
    
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
    n_stations = height(blade_profile);
    r_stations = blade_profile.DistanceFromCenterOfRotation / 1000;
    B = 3;
    
    total_iterations = n_lambda * n_pitch;
    current_iteration = 0;
    
    for j = 1:n_lambda
        lambda = lambda_range(j);
        omega_rad = (lambda * V_wind) / R;
        
        for k = 1:n_pitch
            pitch_angle = pitch_range(k);
            current_iteration = current_iteration + 1;
            
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