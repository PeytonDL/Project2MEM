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

    % Define the path to the Given Parameters folder
    basePath = 'Auxilary Information/Given Parameters';
    
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
        
        % Add metadata
        data.metadata = struct();
        data.metadata.extractionDate = datetime('now');
        data.metadata.sourceFolder = basePath;
        data.metadata.description = 'Wind turbine parameters extracted from Given Parameters folder';
        
        fprintf('Parameter extraction completed successfully.\n');
        fprintf('Extracted data includes:\n');
        fprintf('  - Blade profile: %d stations\n', height(data.blade.profile));
        fprintf('  - Tower specs: %d sections\n', height(data.tower.specs));
        fprintf('  - Airfoil coordinates: %d airfoils\n', length(fieldnames(data.airfoils)));
        fprintf('  - Airfoil performance: %d airfoils\n', length(fieldnames(data.airfoilPerformance)));
        fprintf('  - Material properties: %d materials\n', length(fieldnames(data.materials)));
        fprintf('  - Turbine specifications: %s\n', data.turbine.model);
        
    catch ME
        error('Parameter extraction failed: %s', ME.message);
    end
end

function bladeData = extractBladeProfile(filePath)
% Extract blade profile data from CSV file
    
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
    
    fprintf('  Blade profile: %d stations, %d unique airfoils\n', ...
            bladeData.stations, length(bladeData.airfoilTypes));
end

function towerData = extractTowerSpecs(filePath)
% Extract tower specification data from CSV file
    
    % Read the CSV file
    opts = detectImportOptions(filePath);
    towerTable = readtable(filePath, opts);
    
    % Store the data
    towerData = struct();
    towerData.specs = towerTable;
    towerData.description = 'Tower specifications including height, diameter, and wall thickness';
    
    % Calculate additional parameters
    towerData.totalHeight = max(towerTable.Height_mm_) / 1000; % Convert to meters
    towerData.sections = height(towerTable);
    towerData.baseDiameter = towerTable.OD_mm_(1) / 1000; % Convert to meters
    towerData.topDiameter = towerTable.OD_mm_(end) / 1000; % Convert to meters
    
    fprintf('  Tower specs: %d sections, %.1f m height\n', ...
            towerData.sections, towerData.totalHeight);
end

function airfoilData = extractAirfoilCoordinates(basePath)
% Extract airfoil coordinate data from .dat files
    
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
    
    fprintf('  Airfoil coordinates: %d airfoils extracted\n', length(fieldnames(airfoilData)));
end

function performanceData = extractAirfoilPerformance(basePath)
% Extract airfoil performance data from CSV files
    
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
    
    fprintf('  Airfoil performance: %d airfoils extracted\n', length(fieldnames(performanceData)));
end

function materialData = extractMaterialProperties()
% Extract material properties for air and steel
    
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
    materialData.steel.description = 'Steel properties for structural calculations';
    materialData.steel.units = struct('density', 'kg/m³', 'tensileStrength', 'Pa', 'yieldStrength', 'Pa');
    
    % Calculate additional steel properties
    materialData.steel.safetyFactor = 1.5; % Typical safety factor
    materialData.steel.allowableStress = materialData.steel.yieldStrength / materialData.steel.safetyFactor;
    
    fprintf('  Material properties: Air and Steel (ASTM A572 Grade 50)\n');
end

function turbineData = extractTurbineSpecifications()
% Extract wind turbine specifications for Clipper Liberty C96
    
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
    
    fprintf('  Turbine specifications: %s (%.1f MW, %.1f m radius)\n', ...
            turbineData.model, turbineData.performance.ratedPower/1e6, turbineData.performance.rotorRadius);
end
