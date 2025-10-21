function [x, y, theta, M, V] = cantilever_beam_ode_solver(beam_params, loading_func, boundary_conditions, options)
%% CANTILEVER_BEAM_ODE_SOLVER - Advanced beam analysis using ODE45
%
% This function solves beam deflection problems using differential equations
% with customizable loading, boundary conditions, and material properties.
%
% INPUTS:
%   beam_params     - Structure containing beam properties
%     .L            - Beam length [m]
%     .EI           - Flexural rigidity [Nm^2] (scalar or function handle)
%     .x_nodes      - Integration points [m] (optional, default: 0 to L)
%   loading_func    - Function handle for distributed load q(x) [N/m]
%   boundary_conditions - Structure containing BCs
%     .left_type    - 'fixed', 'pinned', 'free', 'roller'
%     .right_type   - 'fixed', 'pinned', 'free', 'roller'
%     .left_values  - [displacement, rotation] at left end [m, rad]
%     .right_values - [displacement, rotation] at right end [m, rad]
%   options         - Structure with solver options (optional)
%     .tolerance    - ODE solver tolerance (default: 1e-6)
%     .max_step     - Maximum step size (default: L/1000)
%     .plot_results - Plot results (default: true)
%
% OUTPUTS:
%   x       - Position along beam [m]
%   y       - Deflection [m]
%   theta   - Rotation angle [rad]
%   M       - Bending moment [Nm]
%   V       - Shear force [N]
%
% THEORY:
% The beam deflection is governed by the differential equation:
%   d^4y/dx^4 = q(x)/EI(x)
%
% This is converted to a system of first-order ODEs:
%   dy/dx = theta
%   dtheta/dx = M/EI
%   dM/dx = V
%   dV/dx = -q(x)
%
% EXAMPLE:
%   % Define beam parameters
%   beam_params.L = 10;
%   beam_params.EI = 250000;
%   
%   % Define loading (uniform load over first 6m)
%   loading_func = @(x) 100 * (x <= 6);
%   
%   % Define boundary conditions (cantilever)
%   boundary_conditions.left_type = 'fixed';
%   boundary_conditions.right_type = 'free';
%   boundary_conditions.left_values = [0, 0];
%   
%   % Solve
%   [x, y, theta, M, V] = cantilever_beam_ode_solver(beam_params, loading_func, boundary_conditions);
%
% Author: Generated for Project 2 MEM
% Date: 2024

%% Input Validation and Defaults
if nargin < 3
    error('Function requires at least 3 inputs: beam_params, loading_func, boundary_conditions');
end

if nargin < 4
    options = struct();
end

% Set default options
if ~isfield(options, 'tolerance'), options.tolerance = 1e-6; end
if ~isfield(options, 'max_step'), options.max_step = beam_params.L/1000; end
if ~isfield(options, 'plot_results'), options.plot_results = true; end

% Validate beam parameters
if ~isfield(beam_params, 'L') || beam_params.L <= 0
    error('Beam length L must be positive');
end

if ~isfield(beam_params, 'EI')
    error('Flexural rigidity EI must be specified');
end

% Handle EI as scalar or function
if isnumeric(beam_params.EI)
    EI_func = @(x) beam_params.EI * ones(size(x));
elseif isa(beam_params.EI, 'function_handle')
    EI_func = beam_params.EI;
else
    error('EI must be a scalar or function handle');
end

% Set integration points
if isfield(beam_params, 'x_nodes')
    x_span = beam_params.x_nodes;
else
    x_span = linspace(0, beam_params.L, 1000);
end

%% Boundary Condition Setup
% Convert boundary conditions to initial conditions for ODE solver
[initial_conditions, shooting_params] = setup_boundary_conditions(boundary_conditions, beam_params.L, loading_func);

%% ODE System Definition
% Define the system of first-order ODEs
beam_ode_system = @(x, y) beam_ode_system_func(x, y, loading_func, EI_func);

%% Shooting Method for Boundary Value Problem
% Since we have boundary conditions at both ends, we use shooting method
if length(shooting_params) > 0
    % Use fzero to find correct initial conditions with better error handling
    try
        % For cantilever beam, estimate initial moment based on total load
        if exist('loading_func', 'var') && ~isempty(loading_func)
            % Calculate total load and estimate moment
            x_integrate = linspace(0, beam_params.L, 1000);
            q_values = arrayfun(loading_func, x_integrate);
            total_load = trapz(x_integrate, q_values);
            
            % Estimate moment at fixed end: M ≈ -total_load * L/2 (simplified)
            initial_guess = -total_load * beam_params.L / 2;
        else
            initial_guess = -beam_params.L * 1000;  % Fallback guess
        end
        
        % Try multiple initial guesses if first one fails
        guesses = [initial_guess, initial_guess/2, initial_guess*2, -beam_params.L*500, -beam_params.L*2000];
        shooting_solution = NaN;
        
        for i = 1:length(guesses)
            try
                shooting_solution = fzero(@(param) shooting_residual(param, x_span, beam_ode_system, ...
                    boundary_conditions, beam_params.L, loading_func), guesses(i));
                break;  % If successful, exit loop
            catch
                continue;  % Try next guess
            end
        end
        
        if ~isnan(shooting_solution)
            % Update initial conditions with shooting solution
            initial_conditions(3) = shooting_solution;  % Adjust moment at left end
            fprintf('Shooting method converged with initial moment: %.1f Nm\n', shooting_solution);
        else
            % Use estimated moment if shooting fails
            initial_conditions(3) = initial_guess;
            fprintf('Shooting method failed, using estimated moment: %.1f Nm\n', initial_guess);
        end
        
    catch ME
        warning('Shooting method failed: %s', ME.message);
        warning('Using estimated initial conditions');
        % Use estimated moment if all else fails
        if exist('loading_func', 'var') && ~isempty(loading_func)
            x_integrate = linspace(0, beam_params.L, 1000);
            q_values = arrayfun(loading_func, x_integrate);
            total_load = trapz(x_integrate, q_values);
            initial_conditions(3) = -total_load * beam_params.L / 2;
        else
            initial_conditions(3) = -beam_params.L * 1000;
        end
    end
end

%% Solve ODE System
% Set ODE solver options
ode_options = odeset('RelTol', options.tolerance, 'AbsTol', options.tolerance, ...
    'MaxStep', options.max_step);

% Debug: Print initial conditions
fprintf('Initial conditions: y=%.3f, theta=%.3f, M=%.1f, V=%.1f\n', ...
    initial_conditions(1), initial_conditions(2), initial_conditions(3), initial_conditions(4));

% Solve the ODE system
[x, y_solution] = ode45(beam_ode_system, x_span, initial_conditions, ode_options);

% Debug: Check solution
if isempty(y_solution) || any(isnan(y_solution(:))) || any(isinf(y_solution(:)))
    fprintf('Warning: ODE solution contains NaN or Inf values\n');
    fprintf('Solution size: %dx%d\n', size(y_solution, 1), size(y_solution, 2));
end

% Extract results
y = y_solution(:, 1);      % Displacement
theta = y_solution(:, 2);  % Rotation
M = y_solution(:, 3);      % Bending moment
V = y_solution(:, 4);      % Shear force

%% Validation
% Check boundary conditions
validate_boundary_conditions(x, y, theta, M, V, boundary_conditions);

%% Plotting
if options.plot_results
    plot_beam_results(x, y, theta, M, V, loading_func, beam_params.L);
end

%% Output Results
fprintf('=== BEAM ANALYSIS RESULTS ===\n');
fprintf('Beam length: %.2f m\n', beam_params.L);
fprintf('Maximum deflection: %.4f m (%.2f mm) at x = %.2f m\n', ...
    max(abs(y)), max(abs(y))*1000, x(abs(y) == max(abs(y))));
fprintf('Maximum rotation: %.4f rad (%.2f°) at x = %.2f m\n', ...
    max(abs(theta)), max(abs(theta))*180/pi, x(abs(theta) == max(abs(theta))));
fprintf('Maximum moment: %.2f Nm at x = %.2f m\n', ...
    max(abs(M)), x(abs(M) == max(abs(M))));
fprintf('Maximum shear: %.2f N at x = %.2f m\n', ...
    max(abs(V)), x(abs(V) == max(abs(V))));
fprintf('\n');

end

%% Helper Functions

function [initial_conditions, shooting_params] = setup_boundary_conditions(bc, L, loading_func)
    % Setup initial conditions based on boundary conditions
    initial_conditions = zeros(4, 1);  % [y, theta, M, V]
    shooting_params = [];
    
    % Left boundary conditions
    switch lower(bc.left_type)
        case 'fixed'
            initial_conditions(1) = bc.left_values(1);  % y = 0
            initial_conditions(2) = bc.left_values(2);  % theta = 0
            shooting_params = [shooting_params, 0];     % M unknown
            
            % For cantilever beam, calculate total load to determine V(0)
            if exist('loading_func', 'var') && ~isempty(loading_func)
                % Integrate loading function to get total load
                x_integrate = linspace(0, L, 1000);
                q_values = arrayfun(loading_func, x_integrate);
                total_load = trapz(x_integrate, q_values);
                initial_conditions(4) = total_load;  % V(0) = total distributed load
            else
                initial_conditions(4) = 0;  % Default to zero if no loading function
            end
            
        case 'pinned'
            initial_conditions(1) = bc.left_values(1);  % y = 0
            shooting_params = [shooting_params, 0];     % theta unknown
            initial_conditions(3) = 0;                  % M = 0
            initial_conditions(4) = 0;                  % V = 0
        case 'free'
            shooting_params = [shooting_params, 0];     % y unknown
            shooting_params = [shooting_params, 0];     % theta unknown
            initial_conditions(3) = 0;                  % M = 0
            initial_conditions(4) = 0;                  % V = 0
        case 'roller'
            initial_conditions(1) = bc.left_values(1);  % y = 0
            shooting_params = [shooting_params, 0];     % theta unknown
            shooting_params = [shooting_params, 0];     % M unknown
            initial_conditions(4) = 0;                  % V = 0
    end
end

function residual = shooting_residual(param, x_span, beam_ode_system, bc, L, loading_func)
    % Calculate residual for shooting method
    % This is a simplified version - full implementation would handle multiple shooting parameters
    
    try
        % Check if parameter is reasonable
        if abs(param) > 1e10
            residual = 1e10;
            return;
        end
        
        % Calculate total distributed load for initial shear force
        if exist('loading_func', 'var') && ~isempty(loading_func)
            x_integrate = linspace(0, L, 1000);
            q_values = arrayfun(loading_func, x_integrate);
            total_load = trapz(x_integrate, q_values);
        else
            total_load = 0;
        end
        
        % Set initial conditions with shooting parameter
        initial_conditions = [0; 0; param; total_load];  % [y, theta, M, V] for cantilever
        
        % Solve ODE with error handling and tighter tolerances
        ode_options = odeset('RelTol', 1e-6, 'AbsTol', 1e-6, 'MaxStep', L/1000);
        [~, y_solution] = ode45(beam_ode_system, x_span, initial_conditions, ode_options);
        
        % Check for numerical issues
        if isempty(y_solution) || any(isnan(y_solution(:))) || any(isinf(y_solution(:)))
            residual = 1e10;  % Large penalty for numerical issues
            return;
        end
        
        % Calculate residual based on right boundary conditions
        switch lower(bc.right_type)
            case 'free'
                % For free end: M = 0, V = 0
                residual = y_solution(end, 3)^2 + y_solution(end, 4)^2;
            case 'pinned'
                % For pinned end: y = 0, M = 0
                residual = y_solution(end, 1)^2 + y_solution(end, 3)^2;
            case 'fixed'
                % For fixed end: y = 0, theta = 0
                residual = y_solution(end, 1)^2 + y_solution(end, 2)^2;
            otherwise
                residual = 0;
        end
        
        % Ensure residual is finite and reasonable
        if isnan(residual) || isinf(residual) || residual < 0
            residual = 1e10;
        end
        
    catch
        % Return large penalty if ODE solving fails
        residual = 1e10;
    end
end


function validate_boundary_conditions(x, y, theta, M, V, bc)
    % Validate that boundary conditions are satisfied
    tolerance = 1e-3;
    
    % Check left boundary
    switch lower(bc.left_type)
        case 'fixed'
            if abs(y(1)) > tolerance || abs(theta(1)) > tolerance
                warning('Left fixed boundary conditions not satisfied');
            end
        case 'pinned'
            if abs(y(1)) > tolerance || abs(M(1)) > tolerance
                warning('Left pinned boundary conditions not satisfied');
            end
        case 'free'
            if abs(M(1)) > tolerance || abs(V(1)) > tolerance
                warning('Left free boundary conditions not satisfied');
            end
    end
    
    % Check right boundary
    switch lower(bc.right_type)
        case 'free'
            if abs(M(end)) > tolerance || abs(V(end)) > tolerance
                warning('Right free boundary conditions not satisfied');
            end
        case 'pinned'
            if abs(y(end)) > tolerance || abs(M(end)) > tolerance
                warning('Right pinned boundary conditions not satisfied');
            end
        case 'fixed'
            if abs(y(end)) > tolerance || abs(theta(end)) > tolerance
                warning('Right fixed boundary conditions not satisfied');
            end
    end
end

function plot_beam_results(x, y, theta, M, V, loading_func, L)
    % Plot comprehensive beam analysis results
    
    figure('Position', [100, 100, 1200, 1000]);
    
    % Calculate loading for plot
    q_plot = arrayfun(loading_func, x);
    
    % Subplot 1: Loading
    subplot(5,1,1);
    plot(x, q_plot, 'b-', 'LineWidth', 2);
    ylabel('q (N/m)');
    title('Advanced Beam Analysis - ODE Solution');
    grid on;
    xlim([0, L]);
    ylim([0, max(q_plot)*1.1]);
    
    % Subplot 2: Shear Force
    subplot(5,1,2);
    plot(x, V, 'r-', 'LineWidth', 2);
    ylabel('V (N)');
    grid on;
    xlim([0, L]);
    hold on;
    plot([0, L], [0, 0], 'k--', 'LineWidth', 0.5);
    
    % Subplot 3: Bending Moment
    subplot(5,1,3);
    plot(x, M, 'g-', 'LineWidth', 2);
    ylabel('M (Nm)');
    grid on;
    xlim([0, L]);
    hold on;
    plot([0, L], [0, 0], 'k--', 'LineWidth', 0.5);
    
    % Subplot 4: Rotation
    subplot(5,1,4);
    plot(x, theta*1000, 'm-', 'LineWidth', 2);
    ylabel('θ (mrad)');
    grid on;
    xlim([0, L]);
    hold on;
    plot([0, L], [0, 0], 'k--', 'LineWidth', 0.5);
    
    % Subplot 5: Deflection
    subplot(5,1,5);
    plot(x, y*1000, 'c-', 'LineWidth', 2);
    xlabel('x (m)');
    ylabel('δ (mm)');
    grid on;
    xlim([0, L]);
    hold on;
    plot([0, L], [0, 0], 'k--', 'LineWidth', 0.5);
    
    sgtitle('Advanced Beam Analysis - ODE45 Solution', 'FontSize', 14, 'FontWeight', 'bold');
    
    % Save figure
    saveas(gcf, 'advanced_beam_analysis.png');
    fprintf('Figure saved as: advanced_beam_analysis.png\n');
end

%% Helper Function for ODE System
function dydx = beam_ode_system_func(x, y, loading_func, EI_func)
    % y = [displacement, rotation, moment, shear]
    % dy/dx = [rotation, moment/EI, shear, -loading]
    
    displacement = y(1);
    rotation = y(2);
    moment = y(3);
    shear = y(4);
    
    % Calculate EI at current position
    EI_current = EI_func(x);
    
    % System of first-order ODEs
    dydx = zeros(4, 1);
    dydx(1) = rotation;                    % dy/dx = theta
    dydx(2) = moment / EI_current;         % dtheta/dx = M/EI
    dydx(3) = shear;                       % dM/dx = V
    dydx(4) = -loading_func(x);            % dV/dx = -q(x)
end