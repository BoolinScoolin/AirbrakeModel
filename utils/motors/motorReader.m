function [time, thrust, mass] = motorReader(filename)
% motorReader - extracts discrete time, thrust, and mass values from an eng
%   file of a motor burn.
%
%   Syntax:
%       [time, thrust, mass] = motorReader(filename)
%
%   Input Arguments:
%       filename - string containing .eng thrust curve from thrustcurve.org
%                  includes file extension
%         string
%
%   Output Arguments:
%       time - discrete time values
%         vector
%       thrust - discrete thrust values corresponding to time
%         vector
%       mass - discrete mass values corresponding to time
%         vector
%
%   Author: Colin Riba
%   Date: December 16, 2024

%% Read the ENG file
fileID = fopen(filename);
if fileID == -1
    error('File not found. Make sure file is in working directory and the filename is spelled correctly (including file extension).')
end
data = textscan(fileID, '%s', 'Delimiter', '\n');
fclose(fileID);

% Extract the second line (row 2) and parse it
secondLine = data{1}{2};
secondLineValues = str2double(strsplit(secondLine));

% Assign variables from the second line
propellantMass = secondLineValues(5);  % propellant mass
totalMass = secondLineValues(6);  % total mass
caseMass = totalMass - propellantMass;

% Read the subsequent rows starting from row 3
numericData = data{1}(3:end);
parsedData = cell2mat(cellfun(@(x) str2double(strsplit(x)), numericData, 'UniformOutput', false));

% Extract vectors from the columns
time = parsedData(:, 1); % Time values
thrust = parsedData(:, 2); % Thrust values

%% Compute Mass
v_e = trapz(time, thrust) / propellantMass; % effective exhaust velocity (m/s)
mdot = thrust / v_e;  % mass flow rate
propellantMass = propellantMass - cumtrapz(time, mdot);  % propellant mass vs time
mass = propellantMass + caseMass;  % total motor mass