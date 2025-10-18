function [U_rel, AoA] = wind_turbine_angle_of_attack(U, a, a_prime, r, Omega, theta_p)
%% WIND_TURBINE_ANGLE_OF_ATTACK - Calculate angle of attack for HAWT blade cross-section
%
% This function calculates the relative velocity magnitude and angle of attack
% for a 3-bladed lift-based Horizontal Axis Wind Turbine (HAWT) blade cross-section.
%
% INPUTS:
%   U       - Freestream wind velocity [m/s]
%   a       - Axial induction factor [-] (0 ≤ a ≤ 1)
%   a_prime - Angular induction factor [-] (typically 0 ≤ a_prime ≤ 0.5)
%   r       - Radial position [m] (distance from hub center)
%   Omega   - Rotational velocity [rad/s]
%   theta_p - Section pitch angle [rad]
%
% OUTPUTS:
%   U_rel   - Relative velocity magnitude [m/s]
%   AoA     - Angle of attack [rad]
%
% THEORY:
% The function implements the following relationships from wind turbine aerodynamics:
% 1. Effective axial velocity: U_axial = U * (1 - a)
% 2. Effective tangential velocity: U_tangential = r * Omega * (1 + a_prime)
% 3. Relative velocity magnitude: U_rel = sqrt(U_axial^2 + U_tangential^2)
% 4. Inflow angle: phi = atan2(U_tangential, U_axial)
% 5. Angle of attack: AoA = theta_p - phi
%
% The function handles both scalar and vector inputs.
%
% EXAMPLE:
%   [U_rel, AoA] = wind_turbine_angle_of_attack(10, 0.3, 0.1, 5, 2, 0.2);
%
% Author: Generated for Project 2 MEM
% Date: 2024

%% Calculations
% Step 1: Calculate effective velocities
% Note: For angular induction factor, we use (1 + a_prime) for tangential velocity
% This is the standard convention in wind turbine aerodynamics
U_axial = U .* (1 - a);                    % Effective axial velocity [m/s]
U_tangential = r .* Omega .* (1 + a_prime); % Effective tangential velocity [m/s]

% Step 2: Calculate relative velocity magnitude using Pythagorean theorem
U_rel = sqrt(U_axial.^2 + U_tangential.^2);

% Step 3: Calculate inflow angle (angle of relative wind)
% Using atan2 to handle all quadrants correctly
phi = atan2(U_tangential, U_axial);

% Step 4: Calculate angle of attack
% AoA = Section pitch angle - Inflow angle
AoA = theta_p - phi;

end
