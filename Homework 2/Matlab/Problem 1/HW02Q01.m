%% Cantilevered Beam Analysis (ODE45)
% Uniform load q=100 N/m over first 6m, L=10m, EI=250,000 Nm²

clear; clc; close all;

%% Functions

function dydx = beam_ode(x_pos, y)
% ODE system for beam deflection: d²δ/dx² = M(x)/EI

% Beam parameters
L_load = 6; q = 100; EI = 250000;

% Reactions
P_eff = q * L_load; x_eff = L_load/2; R_y = P_eff; M_react = P_eff * x_eff;

% Moment calculation
if x_pos <= L_load
    moment = -(M_react - R_y * x_pos + q * x_pos^2 / 2);
else
    moment = -(M_react - R_y * x_pos + P_eff * (x_pos - x_eff));
end

% ODE: [dδ/dx; dθ/dx] = [θ; M/EI]
dydx = [y(2); moment/EI];

end

%% Parameters
L_total = 10; L_load = 6; q = 100; EI = 250000;

%% Calculations
% Reactions
P_eff = q * L_load; x_eff = L_load/2; R_y = P_eff; M_react = P_eff * x_eff;

% Shear and moment
x = linspace(0, L_total, 1000);
V = zeros(size(x)); M_b = zeros(size(x));

for i = 1:length(x)
    if x(i) <= L_load
        V(i) = R_y - q * x(i);
        M_b(i) = -(M_react - R_y * x(i) + q * x(i)^2 / 2);
    else
        V(i) = 0;
        M_b(i) = -(M_react - R_y * x(i) + P_eff * (x(i) - x_eff));
    end
end

% Deflection using ODE45
y0 = [0; 0];  % Fixed end: δ=0, θ=0
[x_ode, y_ode] = ode45(@beam_ode, [0, L_total], y0);
delta = interp1(x_ode, y_ode(:,1), x, 'linear', 'extrap');
theta = interp1(x_ode, y_ode(:,2), x, 'linear', 'extrap');

%% Results
fprintf('=== CANTILEVERED BEAM ANALYSIS ===\n');
fprintf('Effective load: P = %.0f N at x = %.1f m\n', P_eff, x_eff);
fprintf('Reactions: R_y = %.0f N, M = %.0f Nm\n', R_y, M_react);
fprintf('Max deflection: %.4f m (%.2f mm) at x = %.1f m\n', max(abs(delta)), max(abs(delta))*1000, x(abs(delta) == max(abs(delta))));
fprintf('Max angle: %.4f rad (%.2f°) at x = %.1f m\n', max(abs(theta)), max(abs(theta))*180/pi, x(abs(theta) == max(abs(theta))));
fprintf('\n');

%% Plots
figure('Position', [100, 100, 800, 1200]);

% Load distribution
q_plot = zeros(size(x)); q_plot(x <= L_load) = q;
subplot(5,1,1);
plot(x, q_plot, 'b-', 'LineWidth', 2);
ylabel('q (N/m)'); title('Cantilevered Beam Analysis (ODE45)');
grid on; xlim([0, L_total]); ylim([0, q*1.1]);

% Shear force
subplot(5,1,2);
plot(x, V, 'r-', 'LineWidth', 2);
ylabel('V (N)'); grid on; xlim([0, L_total]);
hold on; plot([0, L_total], [0, 0], 'k--', 'LineWidth', 0.5);

% Bending moment
subplot(5,1,3);
plot(x, M_b, 'g-', 'LineWidth', 2);
ylabel('M_b (Nm)'); grid on; xlim([0, L_total]);
hold on; plot([0, L_total], [0, 0], 'k--', 'LineWidth', 0.5);

% Slope
subplot(5,1,4);
plot(x, theta*1000, 'm-', 'LineWidth', 2);
ylabel('θ (mrad)'); grid on; xlim([0, L_total]);
hold on; plot([0, L_total], [0, 0], 'k--', 'LineWidth', 0.5);

% Deflection
subplot(5,1,5);
plot(x, delta*1000, 'c-', 'LineWidth', 2);
xlabel('x (m)'); ylabel('δ (mm)'); grid on; xlim([0, L_total]);
hold on; plot([0, L_total], [0, 0], 'k--', 'LineWidth', 0.5);

% Load boundary
for i = 1:5
    subplot(5,1,i);
    hold on; plot([L_load, L_load], ylim, 'k:', 'LineWidth', 1);
    if i == 1
        text(L_load, q*0.8, 'Load ends', 'HorizontalAlignment', 'center', 'BackgroundColor', 'white');
    end
end

saveas(gcf, 'cantilever_beam_analysis.png');
fprintf('Figure saved: cantilever_beam_analysis.png\n');
