function [CP, CT] = Deliverable1()
% DELIVERABLE1 Wind Turbine BEM Analysis
% Calculates power and thrust coefficients for specified operating conditions
% Usage: [CP, CT] = Deliverable1();

    clc; close all;
    fprintf('=== Wind Turbine Analysis - Deliverable 1 ===\n\n');
    
    fprintf('Extracting wind turbine parameters...\n');
    data = ParameterExtraction();
    
    V_wind = 10;
    omega = 14;
    pitch_angle = 0;
    
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
        
        dCP(i) = dP(i) / (0.5 * rho * V_wind^3);
        dCT(i) = dT(i) / (0.5 * rho * V_wind^2);
    end
    
    T = B * trapz(r_stations, dT);
    Q = B * trapz(r_stations, dQ);
    P = B * trapz(r_stations, dP);
    
    CP = P / (0.5 * rho * A * V_wind^3);
    CT = T / (0.5 * rho * A * V_wind^2);
    
    fprintf('\n=== RESULTS ===\n');
    fprintf('Coefficient of Power (C_P): %.4f\n', CP);
    fprintf('Coefficient of Thrust (C_T): %.4f\n', CT);
    fprintf('\nCalculated Values:\n');
    fprintf('Power: %.1f kW\n', P/1000);
    fprintf('Thrust: %.1f kN\n', T/1000);
    fprintf('Torque: %.1f kN·m\n', Q/1000);
    
    createVisualization(r_stations, dCP, dCT, CP, CT, lambda);
    fprintf('\nAnalysis complete!\n');
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

function createVisualization(r_stations, dCP, dCT, CP, CT, lambda)
% Create visualization of the analysis results
    
    figure('Position', [100, 100, 1200, 800]);
    
    if ~isempty(dCP) && ~isempty(dCT)
        subplot(2, 2, 1);
        area(r_stations, dCP, 'FaceColor', 'blue', 'FaceAlpha', 0.3, 'EdgeColor', 'blue', 'LineWidth', 2);
        xlabel('Radius [m]');
        ylabel('Local C_P');
        title('Local Power Coefficient Distribution');
        text(0.95, 0.05, sprintf('C_P = ∫c_p dr = %.4f', CP), 'Units', 'normalized', ...
             'FontSize', 12, 'FontWeight', 'bold', 'BackgroundColor', 'white', ...
             'EdgeColor', 'black', 'Margin', 2, 'HorizontalAlignment', 'right', ...
             'VerticalAlignment', 'bottom');
        grid on;
        
        subplot(2, 2, 2);
        area(r_stations, dCT, 'FaceColor', 'red', 'FaceAlpha', 0.3, 'EdgeColor', 'red', 'LineWidth', 2);
        xlabel('Radius [m]');
        ylabel('Local C_T');
        title('Local Thrust Coefficient Distribution');
        text(0.95, 0.05, sprintf('C_T = ∫c_t dr = %.4f', CT), 'Units', 'normalized', ...
             'FontSize', 12, 'FontWeight', 'bold', 'BackgroundColor', 'white', ...
             'EdgeColor', 'black', 'Margin', 2, 'HorizontalAlignment', 'right', ...
             'VerticalAlignment', 'bottom');
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
    text(0.1, 0.2, 'Wind Velocity: 10 m/s', 'FontSize', 12);
    text(0.1, 0.1, 'Rotational Speed: 14 rpm', 'FontSize', 12);
    axis off;
    title('Analysis Results Summary');
    
    sgtitle('Wind Turbine Analysis - Deliverable 1', 'FontSize', 16, 'FontWeight', 'bold');
    saveas(gcf, 'Deliverable1_Results.png');
    fprintf('Results visualization saved as Deliverable1_Results.png\n');
end