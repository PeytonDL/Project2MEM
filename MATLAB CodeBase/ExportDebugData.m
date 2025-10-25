% EXPORTDEBUGDATA - Export debug data to CSV format
% This script demonstrates how to export debug data from the experimental
% deliverable to a properly formatted CSV file.

% Clear workspace
clc; clear;

% Define file paths
debugFilePath = 'Deliverable3_DebugLog_WithPrandtl.txt';
csvFilePath = 'Deliverable3_DebugData_WithPrandtl.csv';

% Check if debug file exists
if exist(debugFilePath, 'file')
    fprintf('Found debug file: %s\n', debugFilePath);
    
    % Export to CSV using the function from Master.m
    try
        exportDebugDataToCSV(debugFilePath, csvFilePath);
        fprintf('\nExport completed successfully!\n');
        fprintf('CSV file saved as: %s\n', csvFilePath);
        
        % Display summary of exported data
        if exist(csvFilePath, 'file')
            T = readtable(csvFilePath);
            fprintf('\nData Summary:\n');
            fprintf('  Rows: %d\n', height(T));
            fprintf('  Columns: %d\n', width(T));
            fprintf('  Column names: %s\n', strjoin(T.Properties.VariableNames, ', '));
        end
        
    catch ME
        fprintf('Error during export: %s\n', ME.message);
    end
    
else
    fprintf('Debug file not found: %s\n', debugFilePath);
    fprintf('Make sure to run the experimental deliverable with DEBUG=true first.\n');
end

function exportDebugDataToCSV(debugFilePath, csvFilePath)
% EXPORTDEBUGDATATOCSV - Convert debug log to CSV format
% Reads the debug log file and converts it to a properly formatted CSV

    % Read the debug log file
    fid = fopen(debugFilePath, 'r');
    if fid == -1
        error('Could not open debug file: %s', debugFilePath);
    end
    
    % Read header line
    headerLine = fgetl(fid);
    if headerLine == -1
        fclose(fid);
        error('Debug file is empty or could not read header');
    end
    
    % Parse header to get column names
    headerTokens = strsplit(headerLine);
    nCols = length(headerTokens);
    
    % Read all data lines
    dataLines = {};
    lineNum = 0;
    while ~feof(fid)
        line = fgetl(fid);
        if line ~= -1
            trimmedLine = strtrim(line);
            if ~isempty(trimmedLine)
                lineNum = lineNum + 1;
                dataLines{lineNum} = line;
            end
        end
    end
    fclose(fid);
    
    % Parse data lines
    nRows = length(dataLines);
    data = zeros(nRows, nCols);
    
    for i = 1:nRows
        tokens = strsplit(dataLines{i});
        for j = 1:min(length(tokens), nCols)
            data(i, j) = str2double(tokens{j});
        end
    end
    
    % Create table
    T = array2table(data, 'VariableNames', headerTokens);
    
    % Write to CSV
    writetable(T, csvFilePath);
    
    fprintf('Exported %d rows and %d columns to %s\n', nRows, nCols, csvFilePath);
end