 
% PHANTOM DRONE - LUMPED THERMAL NETWORK SIMULATION

% Pretty much imports the Drone.3mf, converts it to a lumped thermal network, and simulates its thermal response
% There is a rotating gaussian laser, atmospheric attenuation, and turbulence effects
% As for the drone, its propellers induce airflow, and the sim sweeps from 0 to 360 deg/s
% MATLAB R2025a
% ================================================================

clear;
clc;
close all;

% 1. USER SETTINGS
DRONE_FILE = "Drone.3mf";

T0 = 298.15;                         % K
Tinf = 298.15;                       % K
t_final = 60.0;                      % s
dt = 0.05;                           % s
OUTPUT_INTERVAL = 1.0;               % terminal output interval [s]

USE_CONVECTION = true;
USE_PROPELLER_AIRFLOW = true;
USE_RADIATION = true;
USE_LASER_NOISE = true;

emissivity = 0.85;
sigma = 5.670374419e-8;              % W/(m^2 K^4)
h_free = 10.0;                       % W/(m^2 K)

% Rotation
rotationSweep_deg_s = 0:60:360;
rotationSweep_RPM = rotationSweep_deg_s/6;
omegaSweep_rad_s = rotationSweep_deg_s*pi/180;

 
% Propeller-induced airflow
% This is a reduced-order propeller model.  Replace the multiplier with measured motor/propeller RPM when available
PROP_RPM_MULTIPLIER = 20.0;
PROP_THRUST_COEFFICIENT = 0.10;
PROP_AIRFLOW_DIRECTIONAL_FACTOR = 0.65;
PROP_AIRFLOW_SPREAD_FACTOR = 1.50;
PROP_MIN_DIAMETER = 0.01;

 
% Air properties
air_density = 1.184;                 % kg/m^3
air_viscosity = 1.849e-5;            % Pa s
air_conductivity = 0.0263;           % W/(m K)
air_prandtl = 0.707;
L_characteristic = 0.3302;           % m

 
% Laser
P0 = 5000.0;                          % W
ALPHA_ATM = 1.65e-4;                  % 1/m
WAVELENGTH = 1064e-9;                % m
D = 0.10;                             % m
M2 = 1.5;
laser_absorptivity = 0.80;

laserOffset = [2000.0, 0.0, -80.0];  % m from assembly centroid
laserAimPoint = "body";
laserTarget = "body";
LASER_SPILLOVER_FRACTION = 0.0;

 
% Stochastic laser beam
LASER_NOISE_SIGMA_LOG = 0.50;
LASER_NOISE_TEMPORAL_TAU = 0.10;      % s
LASER_NOISE_SPATIAL_SCALE = 0.50;     % relative beam-radius scale
LASER_NOISE_SEED = 24681357;

 
% Turbulence / beam spreading
Cn2 = 1.0e-14;                        % m^(-2/3)
USE_TURBULENCE = true;

 
% Thermal contact network
CONTACT_GAP = 0.002;                  % m
CONTACT_AREA_FRACTION = 0.25;
CONTACT_LENGTH = 0.001;               % m
MAX_THERMAL_LAYERS = 10;

 
% Numerical safety
MAX_TEMPERATURE_K = 5000;
MIN_TEMPERATURE_K = 100;

 
% Output
outputFolder = fullfile(pwd,"phantom_drone_lumped_results");
if ~isfolder(outputFolder)
    mkdir(outputFolder);
end

fprintf('\n');
fprintf('       PHANTOM DRONE LUMPED THERMAL SIMULATION\n');
fprintf('NO FEA / NO FEM / NO PDE TOOLBOX\n');
fprintf('\n');

 
% 2. MATERIAL DATABASE
materials = struct();

materials.PLA.rho = 1240;
materials.PLA.cp = 1800;
materials.PLA.k = 0.130;
materials.PLA.emissivity = 0.90;
materials.PLA.absorptivity = 0.80;

materials.Aluminum6061.rho = 2700;
materials.Aluminum6061.cp = 896;
materials.Aluminum6061.k = 167;
materials.Aluminum6061.emissivity = 0.85;
materials.Aluminum6061.absorptivity = 0.80;

materials.Copper.rho = 8960;
materials.Copper.cp = 385;
materials.Copper.k = 401;
materials.Copper.emissivity = 0.80;
materials.Copper.absorptivity = 0.80;

materials.CarbonFiber.rho = 1700;
materials.CarbonFiber.cp = 710;
materials.CarbonFiber.k = 10;
materials.CarbonFiber.emissivity = 0.90;
materials.CarbonFiber.absorptivity = 0.80;

materials.Steel.rho = 7850;
materials.Steel.cp = 490;
materials.Steel.k = 45;
materials.Steel.emissivity = 0.80;
materials.Steel.absorptivity = 0.80;

bodyMaterial = "PLA";
motorMaterial = "Aluminum6061";
propMaterial = "CarbonFiber";
defaultMaterial = "Aluminum6061";

 
% 3. THERMAL BOUNDARY CONDITIONS
fprintf('THERMAL BOUNDARY CONDITIONS\n');
fprintf('------------------------------------------------------------\n');
fprintf('Convection:            %s\n',logicalToOnOff(USE_CONVECTION));
fprintf('Propeller airflow:     %s\n',logicalToOnOff(USE_PROPELLER_AIRFLOW));
fprintf('Radiation:             %s\n',logicalToOnOff(USE_RADIATION));
fprintf('Laser stochasticity:   %s\n',logicalToOnOff(USE_LASER_NOISE));
fprintf('\n');

 
% 4. IMPORT 3MF ASSEMBLY
if ~isfile(DRONE_FILE)
    error('Drone.3mf was not found at: %s',DRONE_FILE);
end

fprintf('IMPORTING 3MF ASSEMBLY\n');
fprintf('------------------------------------------------------------\n');

[assemblyObjects,allPoints,allTriangles] = readDrone3MF(DRONE_FILE);
numObjects = numel(assemblyObjects);

fprintf('Objects:       %d\n',numObjects);
fprintf('Vertices:      %d\n',size(allPoints,1));
fprintf('Triangles:     %d\n',size(allTriangles,1));

 
% 5. CLASSIFY COMPONENTS
for i = 1:numObjects
    nameLower = lower(string(assemblyObjects(i).name));

    motorMatch = regexp(char(nameLower),'motor[^0-9]*([0-9]+)','tokens','once');
    propMatch = regexp(char(nameLower),'prop[^0-9]*([0-9]+)','tokens','once');

    if contains(nameLower,"body") || contains(nameLower,"frame") || ...
            contains(nameLower,"chassis")
        assemblyObjects(i).componentType = "body";
        assemblyObjects(i).componentNumber = 1;
        assemblyObjects(i).material = char(bodyMaterial);
    elseif ~isempty(motorMatch)
        assemblyObjects(i).componentType = "motor";
        assemblyObjects(i).componentNumber = str2double(motorMatch{1});
        assemblyObjects(i).material = char(motorMaterial);
    elseif ~isempty(propMatch)
        assemblyObjects(i).componentType = "prop";
        assemblyObjects(i).componentNumber = str2double(propMatch{1});
        assemblyObjects(i).material = char(propMaterial);
    else
        assemblyObjects(i).componentType = "other";
        assemblyObjects(i).componentNumber = i;
        assemblyObjects(i).material = char(defaultMaterial);
    end
end

% If the CAD names did not identify the body, use the largest mesh.
bodyCandidates = find(string({assemblyObjects.componentType}) == "body");
if isempty(bodyCandidates)
    triangleCounts = arrayfun(@(s)size(s.triangles,1),assemblyObjects);
    [~,bodyIndex] = max(triangleCounts);
    assemblyObjects(bodyIndex).componentType = "body";
    assemblyObjects(bodyIndex).componentNumber = 1;
    assemblyObjects(bodyIndex).material = char(bodyMaterial);
else
    bodyIndex = bodyCandidates(1);
end

fprintf('\nCOMPONENT MAP\n');
fprintf('------------------------------------------------------------\n');
for i = 1:numObjects
    fprintf('Object %2d | %-8s %d | %-18s | %s\n',...
        i,assemblyObjects(i).componentType,...
        assemblyObjects(i).componentNumber,...
        assemblyObjects(i).material,...
        assemblyObjects(i).name);
end

 
% 6. GEOMETRY AND THERMAL PROPERTIES
fprintf('\nCALCULATING COMPONENT PROPERTIES\n');
fprintf('------------------------------------------------------------\n');

for i = 1:numObjects
    p = assemblyObjects(i).points;
    tri = assemblyObjects(i).triangles;

    if isempty(p) || isempty(tri)
        error('Object %d contains no usable mesh.',i);
    end

    v1 = p(tri(:,1),:);
    v2 = p(tri(:,2),:);
    v3 = p(tri(:,3),:);

    cross12 = cross(v2-v1,v3-v1,2);
    triangleAreas = 0.5*sqrt(sum(cross12.^2,2));
    surfaceArea = sum(triangleAreas);

    signedVolume = sum(dot(v1,cross(v2,v3,2),2))/6;
    volume = abs(signedVolume);

    if volume < 1e-12
        % Open/degenerate CAD surfaces can have unreliable signed volume.
        dims = max(p,[],1)-min(p,[],1);
        volume = max(prod(max(dims,1e-6))*0.10,1e-12);
    end

    material = materials.(assemblyObjects(i).material);
    mass = material.rho*volume;
    thermalCapacity = mass*material.cp;
    centroid = mean(p,1);
    dimensions = max(p,[],1)-min(p,[],1);

    assemblyObjects(i).geometry.volume = volume;
    assemblyObjects(i).geometry.surfaceArea = surfaceArea;
    assemblyObjects(i).geometry.mass = mass;
    assemblyObjects(i).geometry.thermalCapacity = thermalCapacity;
    assemblyObjects(i).geometry.centroid = centroid;
    assemblyObjects(i).geometry.dimensions = dimensions;
    assemblyObjects(i).geometry.triangleAreas = triangleAreas;
    assemblyObjects(i).geometry.triangleCentroids = (v1+v2+v3)/3;

    assemblyObjects(i).centroid = centroid;
    assemblyObjects(i).area = surfaceArea;
    assemblyObjects(i).volume = volume;
    assemblyObjects(i).mass = mass;
    assemblyObjects(i).thermalCapacity = thermalCapacity;
    assemblyObjects(i).temperature = T0;
end

assemblyCentroid = mean(allPoints,1);

fprintf('Assembly centroid: [%.6f %.6f %.6f] m\n',assemblyCentroid);
fprintf('\n');

for i = 1:numObjects
    fprintf('Object %2d | %-6s %d | V = %.6e m^3 | m = %.6f kg | A = %.6e m^2\n',...
        i,assemblyObjects(i).componentType,...
        assemblyObjects(i).componentNumber,...
        assemblyObjects(i).volume,...
        assemblyObjects(i).mass,...
        assemblyObjects(i).area);
end

 
% 7. LASER GEOMETRY
laserOrigin = assemblyCentroid + laserOffset;
laserAimPointWorld = getLaserAimPoint(assemblyObjects,bodyIndex,laserAimPoint,assemblyCentroid);
laserDirection = laserAimPointWorld-laserOrigin;
laserRange = norm(laserDirection);
laserDirection = laserDirection/max(laserRange,eps);

fprintf('\nLASER GEOMETRY\n');
fprintf('------------------------------------------------------------\n');
fprintf('Laser origin:   [%.6f %.6f %.6f] m\n',laserOrigin);
fprintf('Laser aim:      [%.6f %.6f %.6f] m\n',laserAimPointWorld);
fprintf('Laser direction: [%.6f %.6f %.6f]\n',laserDirection);
fprintf('Laser range:    %.6f m\n',laserRange);

w0 = D/(pi*M2);
zR = pi*w0^2/(M2*WAVELENGTH);
wVacuum = w0*sqrt(1+(laserRange/max(zR,eps))^2);

r0 = friedParameter(laserRange,WAVELENGTH,Cn2);
wTurb = wVacuum;
if USE_TURBULENCE && isfinite(r0) && r0 > 0
    turbulenceFactor = sqrt(1+(wVacuum/max(r0,eps))^2);
    wTurb = wVacuum*turbulenceFactor;
else
    turbulenceFactor = 1;
end

fprintf('Gaussian waist: %.6e m\n',w0);
fprintf('Rayleigh range: %.6e m\n',zR);
fprintf('Fried parameter: %.6e m\n',r0);
fprintf('Turbulence factor: %.6f\n',turbulenceFactor);
fprintf('Initial turbulent beam width: %.6e m\n',wTurb);

 
% 8. THERMAL CONTACT NETWORK
fprintf('\nBUILDING THERMAL CONTACT NETWORK\n');
fprintf('------------------------------------------------------------\n');

G = zeros(numObjects);

for i = 1:numObjects-1
    for j = i+1:numObjects
        boxImin = min(assemblyObjects(i).points,[],1);
        boxImax = max(assemblyObjects(i).points,[],1);
        boxJmin = min(assemblyObjects(j).points,[],1);
        boxJmax = max(assemblyObjects(j).points,[],1);

        gapVec = max([boxImin-boxJmax;boxJmin-boxImax],[],1);
        gapVec = max(gapVec,0);
        gap = norm(gapVec);

        overlap = min(boxImax,boxJmax)-max(boxImin,boxJmin);
        overlap = max(overlap,0);
        overlapVolume = prod(overlap);

        contactArea = 0;
        if gap <= CONTACT_GAP
            positiveOverlap = overlap(overlap > 0);
            if numel(positiveOverlap) >= 2
                sortedOverlap = sort(positiveOverlap,'descend');
                contactArea = sortedOverlap(1)*sortedOverlap(2);
            elseif overlapVolume > 0
                contactArea = overlapVolume^(2/3);
            end
        end

        if contactArea <= 0
            continue;
        end

        contactArea = CONTACT_AREA_FRACTION*contactArea;
        contactArea = min(contactArea,...
            CONTACT_AREA_FRACTION*min(assemblyObjects(i).area,assemblyObjects(j).area));

        kI = materials.(assemblyObjects(i).material).k;
        kJ = materials.(assemblyObjects(j).material).k;
        kEff = 2*kI*kJ/max(kI+kJ,eps);
        gij = kEff*contactArea/CONTACT_LENGTH;

        if gij > 0
            G(i,j) = gij;
            G(j,i) = gij;
        end
    end
end

connectionCount = nnz(triu(G>0,1));
fprintf('Thermal connections: %d\n',connectionCount);

for i = 1:numObjects
    for j = i+1:numObjects
        if G(i,j) > 0
            fprintf('Object %2d <-> Object %2d | G = %.6f W/K\n',i,j,G(i,j));
        end
    end
end

 
% 9. THERMAL LAYERS
layer = inf(numObjects,1);
layer(bodyIndex) = 0;
queue = bodyIndex;
head = 1;

while head <= numel(queue)
    current = queue(head);
    head = head+1;
    neighbors = find(G(current,:) > 0);
    for n = neighbors
        if isinf(layer(n))
            layer(n) = layer(current)+1;
            queue(end+1) = n; %#ok<AGROW>
        end
    end
end

layer(isinf(layer)) = MAX_THERMAL_LAYERS;

fprintf('\nTHERMAL LAYERS\n');
fprintf('------------------------------------------------------------\n');
for i = 1:numObjects
    fprintf('Object %2d | layer %d\n',i,layer(i));
end

 
% 10. PROPELLER GEOMETRY AND AIRFLOW CALIBRATION
propIndices = find(string({assemblyObjects.componentType}) == "prop");
propDiameter = estimatePropellerDiameter(assemblyObjects,propIndices,PROP_MIN_DIAMETER);
propArea = pi*(propDiameter/2)^2;
numProps = max(numel(propIndices),1);

fprintf('\nPROPELLER AIRFLOW MODEL\n');
fprintf('------------------------------------------------------------\n');
fprintf('Propellers detected: %d\n',numel(propIndices));
fprintf('Effective propeller diameter: %.6f m\n',propDiameter);
fprintf('Disk area per propeller: %.6e m^2\n',propArea);
fprintf('RPM multiplier: %.3f\n',PROP_RPM_MULTIPLIER);
fprintf('Thrust coefficient Ct: %.4f\n',PROP_THRUST_COEFFICIENT);

 
% 11. RESULT ARRAYS
nRotation = numel(rotationSweep_deg_s);

timeVector = 0:dt:t_final;
nTime = numel(timeVector);

maxTemperature_K = zeros(numObjects,nRotation);
finalTemperature_K = zeros(numObjects,nRotation);
finalAverageTemperature_K = zeros(numObjects,nRotation);
timeOfMaximum_s = zeros(numObjects,nRotation);
maxLaserPower_W = zeros(numObjects,nRotation);
totalLaserEnergy_J = zeros(numObjects,nRotation);
averageConvection_W_m2K = zeros(numObjects,nRotation);
averagePropellerAirflow_m_s = zeros(numObjects,nRotation);
maxPropellerAirflow_m_s = zeros(numObjects,nRotation);
meanLaserPower_W = zeros(numObjects,nRotation);
laserPowerStd_W = zeros(numObjects,nRotation);
terminationReason = strings(numObjects,nRotation);

results = cell(numObjects,nRotation);

rng(LASER_NOISE_SEED,'twister');

% 12. ROTATION SWEEP
for rotationIndex = 1:nRotation

    rotation_deg_s = rotationSweep_deg_s(rotationIndex);
    rotationRPM = rotationSweep_RPM(rotationIndex);
    omega_z = omegaSweep_rad_s(rotationIndex);
    frequency_Hz = rotationRPM/60;

    fprintf('\n');
    fprintf('============================================================\n');
    fprintf('STARTING ROTATION CASE\n');
    fprintf('============================================================\n');
    fprintf('Rotation:       %.2f deg/s\n',rotation_deg_s);
    fprintf('Equivalent RPM: %.2f RPM\n',rotationRPM);
    fprintf('Frequency:      %.4f Hz\n',frequency_Hz);
    fprintf('Omega:          %.6f rad/s\n',omega_z);

    propRPM = PROP_RPM_MULTIPLIER*rotationRPM;
    nPropRev_s = propRPM/60;
    thrustPerProp = PROP_THRUST_COEFFICIENT*air_density*nPropRev_s^2*propDiameter^4;
    totalThrust = numProps*thrustPerProp;
    inducedVelocity = sqrt(max(totalThrust,0)/(2*air_density*max(numProps*propArea,eps)));
    inducedVelocity = PROP_AIRFLOW_DIRECTIONAL_FACTOR*inducedVelocity;

    fprintf('Propeller RPM:  %.3f RPM\n',propRPM);
    fprintf('Total thrust:   %.6f N\n',totalThrust);
    fprintf('Induced airspeed: %.6f m/s\n',inducedVelocity);

    for i = 1:numObjects
        [caseResult,stats] = solveRotationCase(...
            assemblyObjects,G,materials,layer,...
            i,rotation_deg_s,omega_z,frequency_Hz,...
            T0,Tinf,t_final,dt,...
            USE_CONVECTION,USE_PROPELLER_AIRFLOW,USE_RADIATION,...
            h_free,air_density,air_viscosity,air_conductivity,air_prandtl,...
            L_characteristic,emissivity,sigma,...
            laserTarget,LASER_SPILLOVER_FRACTION,...
            laserOrigin,assemblyCentroid,P0,ALPHA_ATM,WAVELENGTH,D,M2,...
            laser_absorptivity,Cn2,USE_TURBULENCE,...
            USE_LASER_NOISE,LASER_NOISE_SIGMA_LOG,LASER_NOISE_TEMPORAL_TAU,...
            LASER_NOISE_SPATIAL_SCALE,propRPM,inducedVelocity,...
            PROP_AIRFLOW_SPREAD_FACTOR,LASER_NOISE_SEED+i*1000+rotationIndex,...
            MAX_TEMPERATURE_K,MIN_TEMPERATURE_K);

        results{i,rotationIndex} = caseResult;
        maxTemperature_K(i,rotationIndex) = stats.maxTemperature_K;
        finalTemperature_K(i,rotationIndex) = stats.finalTemperature_K;
        finalAverageTemperature_K(i,rotationIndex) = stats.finalAverageTemperature_K;
        timeOfMaximum_s(i,rotationIndex) = stats.timeOfMaximum_s;
        maxLaserPower_W(i,rotationIndex) = stats.maxLaserPower_W;
        totalLaserEnergy_J(i,rotationIndex) = stats.totalLaserEnergy_J;
        averageConvection_W_m2K(i,rotationIndex) = stats.averageH;
        averagePropellerAirflow_m_s(i,rotationIndex) = stats.averagePropellerAirflow;
        maxPropellerAirflow_m_s(i,rotationIndex) = stats.maxPropellerAirflow;
        meanLaserPower_W(i,rotationIndex) = stats.meanLaserPower_W;
        laserPowerStd_W(i,rotationIndex) = stats.laserPowerStd_W;
        terminationReason(i,rotationIndex) = stats.termination;
    end
end

 
% 13. SUMMARY TABLE
 

rows = numObjects*nRotation;

summaryObject = strings(rows,1);
summaryType = strings(rows,1);
summaryNumber = zeros(rows,1);
summaryMaterial = strings(rows,1);
summaryDeg_s = zeros(rows,1);
summaryRPM = zeros(rows,1);
summaryFrequency_Hz = zeros(rows,1);
summaryMaxK = zeros(rows,1);
summaryMaxC = zeros(rows,1);
summaryFinalK = zeros(rows,1);
summaryFinalC = zeros(rows,1);
summaryTimeMax_s = zeros(rows,1);
summaryMaxLaserPower_W = zeros(rows,1);
summaryMeanLaserPower_W = zeros(rows,1);
summaryLaserStd_W = zeros(rows,1);
summaryLaserEnergy_J = zeros(rows,1);
summaryAverageH = zeros(rows,1);
summaryAveragePropAirflow = zeros(rows,1);
summaryMaxPropAirflow = zeros(rows,1);
summaryTermination = strings(rows,1);

r = 0;
for i = 1:numObjects
    for j = 1:nRotation
        r = r+1;
        summaryObject(r) = assemblyObjects(i).name;
        summaryType(r) = assemblyObjects(i).componentType;
        summaryNumber(r) = assemblyObjects(i).componentNumber;
        summaryMaterial(r) = assemblyObjects(i).material;
        summaryDeg_s(r) = rotationSweep_deg_s(j);
        summaryRPM(r) = rotationSweep_RPM(j);
        summaryFrequency_Hz(r) = rotationSweep_RPM(j)/60;
        summaryMaxK(r) = maxTemperature_K(i,j);
        summaryMaxC(r) = maxTemperature_K(i,j)-273.15;
        summaryFinalK(r) = finalTemperature_K(i,j);
        summaryFinalC(r) = finalTemperature_K(i,j)-273.15;
        summaryTimeMax_s(r) = timeOfMaximum_s(i,j);
        summaryMaxLaserPower_W(r) = maxLaserPower_W(i,j);
        summaryMeanLaserPower_W(r) = meanLaserPower_W(i,j);
        summaryLaserStd_W(r) = laserPowerStd_W(i,j);
        summaryLaserEnergy_J(r) = totalLaserEnergy_J(i,j);
        summaryAverageH(r) = averageConvection_W_m2K(i,j);
        summaryAveragePropAirflow(r) = averagePropellerAirflow_m_s(i,j);
        summaryMaxPropAirflow(r) = maxPropellerAirflow_m_s(i,j);
        summaryTermination(r) = terminationReason(i,j);
    end
end

summaryTable = table(...
    summaryObject,summaryType,summaryNumber,summaryMaterial,...
    summaryDeg_s,summaryRPM,summaryFrequency_Hz,...
    summaryMaxK,summaryMaxC,summaryFinalK,summaryFinalC,...
    summaryTimeMax_s,summaryMaxLaserPower_W,summaryMeanLaserPower_W,...
    summaryLaserStd_W,summaryLaserEnergy_J,summaryAverageH,...
    summaryAveragePropAirflow,summaryMaxPropAirflow,summaryTermination,...
    'VariableNames',{'ObjectName','ComponentType','ComponentNumber','Material',...
    'Rotation_deg_s','Equivalent_RPM','Frequency_Hz',...
    'MaximumTemperature_K','MaximumTemperature_C','FinalTemperature_K','FinalTemperature_C',...
    'TimeOfMaximum_s','MaximumLaserPower_W','MeanLaserPower_W','LaserPowerStd_W',...
    'TotalLaserEnergy_J','AverageConvection_W_m2K','AveragePropellerAirflow_m_s',...
    'MaximumPropellerAirflow_m_s','Termination'});

% 14. PRINT FINAL SUMMARY
fprintf('\n');
fprintf('============================================================\n');
fprintf('FINAL ROTATION SWEEP SUMMARY\n');
fprintf('============================================================\n');

for j = 1:nRotation
    fprintf('\nRotation = %.0f deg/s (%.0f RPM)\n',...
        rotationSweep_deg_s(j),rotationSweep_RPM(j));
    fprintf('  Mean propeller airflow = %.4f m/s\n',mean(averagePropellerAirflow_m_s(:,j)));
    fprintf('  Mean convection h      = %.4f W/(m^2 K)\n',mean(averageConvection_W_m2K(:,j)));
    fprintf('  Maximum drone temp     = %.3f C\n',max(maxTemperature_K(:,j),[],'omitnan')-273.15);
    fprintf('  Final maximum temp     = %.3f C\n',max(finalTemperature_K(:,j),[],'omitnan')-273.15);
    fprintf('  Maximum laser power    = %.4f W\n',max(maxLaserPower_W(:,j),[],'omitnan'));
    fprintf('  Total laser energy     = %.4f J\n',sum(totalLaserEnergy_J(:,j),'omitnan'));
end

% 15. PLOT
% Maximum temperature vs rotation
figure('Name','Maximum Temperature vs Rotation');
hold on;
for i = 1:numObjects
    plot(rotationSweep_deg_s,maxTemperature_K(i,:)-273.15,'-o',...
        'LineWidth',1.2,'DisplayName',sprintf('Object %d - %s %d',...
        i,assemblyObjects(i).componentType,assemblyObjects(i).componentNumber));
end
grid on;
xlabel('Rotation speed [deg/s]');
ylabel('Maximum temperature [C]');
title('Maximum Temperature vs Rotation Speed');
legend('Location','eastoutside');
exportgraphics(gcf,fullfile(outputFolder,'maximum_temperature_vs_rotation.png'));

% Convection vs rotation
figure('Name','Convection vs Rotation');
plot(rotationSweep_deg_s,mean(averageConvection_W_m2K,1),'-o','LineWidth',2);
grid on;
xlabel('Rotation speed [deg/s]');
ylabel('Mean h [W/(m^2 K)]');
title('Convection Coefficient vs Rotation Speed');
exportgraphics(gcf,fullfile(outputFolder,'convection_vs_rotation.png'));

% Propeller airflow vs rotation
figure('Name','Propeller Airflow vs Rotation');
plot(rotationSweep_deg_s,mean(averagePropellerAirflow_m_s,1),'-o','LineWidth',2);
grid on;
xlabel('Rotation speed [deg/s]');
ylabel('Mean induced airflow [m/s]');
title('Propeller-Induced Airflow vs Rotation Speed');
exportgraphics(gcf,fullfile(outputFolder,'propeller_airflow_vs_rotation.png'));

% Laser power vs rotation
figure('Name','Laser Power vs Rotation');
plot(rotationSweep_deg_s,max(maxLaserPower_W,[],1),'-o','LineWidth',2);
grid on;
xlabel('Rotation speed [deg/s]');
ylabel('Maximum absorbed laser power [W]');
title('Maximum Absorbed Laser Power vs Rotation Speed');
exportgraphics(gcf,fullfile(outputFolder,'laser_power_vs_rotation.png'));

% 16. SAVE RESULTS
summaryFile = fullfile(outputFolder,'thermal_lumped_rotation_summary.csv');
writetable(summaryTable,summaryFile);

matFile = fullfile(outputFolder,'thermal_lumped_rotation_results.mat');
save(matFile,...
    'results','assemblyObjects','G','layer','summaryTable',...
    'rotationSweep_deg_s','rotationSweep_RPM','omegaSweep_rad_s',...
    'maxTemperature_K','finalTemperature_K','finalAverageTemperature_K',...
    'timeOfMaximum_s','maxLaserPower_W','meanLaserPower_W','laserPowerStd_W',...
    'totalLaserEnergy_J','averageConvection_W_m2K',...
    'averagePropellerAirflow_m_s','maxPropellerAirflow_m_s',...
    'terminationReason','assemblyCentroid','laserOrigin','laserAimPointWorld',...
    'laserDirection','w0','zR','r0','wTurb','Cn2',...
    'PROP_RPM_MULTIPLIER','PROP_THRUST_COEFFICIENT',...
    'USE_LASER_NOISE','LASER_NOISE_SIGMA_LOG','LASER_NOISE_TEMPORAL_TAU',...
    'LASER_NOISE_SPATIAL_SCALE','LASER_NOISE_SEED');

fprintf('\n');
fprintf('============================================================\n');
fprintf('SIMULATION COMPLETE\n');
fprintf('============================================================\n');
fprintf('Model type:       Lumped thermal network\n');
fprintf('FEA/FEM:          OFF\n');
fprintf('Rotation sweep:   0 to 360 deg/s\n');
fprintf('RPM sweep:        0 to 60 RPM\n');
fprintf('Time step:        %.3f s\n',dt);
fprintf('Final time:       %.1f s\n',t_final);
fprintf('Convection:       %s\n',logicalToOnOff(USE_CONVECTION));
fprintf('Prop airflow:     %s\n',logicalToOnOff(USE_PROPELLER_AIRFLOW));
fprintf('Radiation:        %s\n',logicalToOnOff(USE_RADIATION));
fprintf('Laser noise:      %s\n',logicalToOnOff(USE_LASER_NOISE));
fprintf('CSV: %s\n',summaryFile);
fprintf('MAT: %s\n',matFile);
fprintf('============================================================\n');

% % LOCAL FUNCTION: SOLVE ROTATION CASE
function [caseResult,stats] = solveRotationCase(...
    objects,G,materials,layer,targetObject,rotation_deg_s,omega_z,frequency_Hz,...
    T0,Tinf,t_final,dt,USE_CONVECTION,USE_PROPELLER_AIRFLOW,USE_RADIATION,...
    h_free,air_density,air_viscosity,air_conductivity,air_prandtl,L_characteristic,...
    emissivity,sigma,laserTarget,spilloverFraction,laserOrigin,assemblyCentroid,...
    P0,ALPHA_ATM,WAVELENGTH,D,M2,absorptivity,Cn2,USE_TURBULENCE,...
    USE_LASER_NOISE,noiseSigma,noiseTau,noiseSpatialScale,propRPM,inducedVelocity,...
    propSpreadFactor,noiseSeed,maxTAllowed,minTAllowed)

numObjects = numel(objects);
nSteps = floor(t_final/dt)+1;
timeHistory = zeros(nSteps,1);
maxTemperatureHistory = zeros(nSteps,1);
laserPowerHistory = zeros(nSteps,1);
convectionPowerHistory = zeros(nSteps,1);
radiationPowerHistory = zeros(nSteps,1);
conductionPowerHistory = zeros(nSteps,1);
propAirflowHistory = zeros(nSteps,1);
meanTemperatureHistory = zeros(nSteps,1);

T = T0*ones(numObjects,1);

rng(noiseSeed,'twister');
noiseState = 0;

maxT = T0;
maxTTime = 0;
termination = "time limit";

sumH = 0;
sumProp = 0;

for step = 1:nSteps
    currentTime = (step-1)*dt;
    timeHistory(step) = currentTime;

    laserPower = zeros(numObjects,1);
    hValues = zeros(numObjects,1);
    propAirflowValues = zeros(numObjects,1);

   
    % Laser power on each component
    for i = 1:numObjects
        isTarget = isTargetComponent(objects(i),laserTarget);
        if isTarget
            laserPower(i) = calculateComponentLaserPower(...
                objects(i),currentTime,omega_z,assemblyCentroid,laserOrigin,...
                P0,ALPHA_ATM,WAVELENGTH,D,M2,absorptivity,...
                Cn2,USE_TURBULENCE,USE_LASER_NOISE,noiseSigma,noiseTau,...
                noiseSpatialScale,noiseState);
        elseif spilloverFraction > 0
            laserPower(i) = spilloverFraction*calculateComponentLaserPower(...
                objects(i),currentTime,omega_z,assemblyCentroid,laserOrigin,...
                P0,ALPHA_ATM,WAVELENGTH,D,M2,absorptivity,...
                Cn2,USE_TURBULENCE,USE_LASER_NOISE,noiseSigma,noiseTau,...
                noiseSpatialScale,noiseState);
        end
    end

   
    % Mean-zero stochastic temporal state
    if USE_LASER_NOISE && noiseTau > 0
        rhoNoise = exp(-dt/noiseTau);
        noiseState = rhoNoise*noiseState + sqrt(max(1-rhoNoise^2,0))*randn;
    else
        noiseState = 0;
    end

   
    % Convection coefficient and propeller airflow
   
    for i = 1:numObjects
        [hValues(i),propAirflowValues(i)] = calculateAverageConvection(...
            objects(i),omega_z,assemblyCentroid,h_free,...
            air_density,air_viscosity,air_conductivity,air_prandtl,...
            L_characteristic,USE_PROPELLER_AIRFLOW,inducedVelocity,...
            propSpreadFactor);
    end

   
    % Energy balance
    dTdt = zeros(numObjects,1);
    netConduction = zeros(numObjects,1);
    convectionPower = zeros(numObjects,1);
    radiationPower = zeros(numObjects,1);

    for i = 1:numObjects
        for j = 1:numObjects
            if i ~= j && G(i,j) > 0
                netConduction(i) = netConduction(i) + G(i,j)*(T(j)-T(i));
            end
        end

        if USE_CONVECTION
            convectionPower(i) = hValues(i)*objects(i).area*(T(i)-Tinf);
        end

        if USE_RADIATION
            eps_i = emissivity;
            if isfield(materials.(objects(i).material),'emissivity')
                eps_i = materials.(objects(i).material).emissivity;
            end
            radiationPower(i) = eps_i*sigma*objects(i).area*(T(i)^4-Tinf^4);
        end

        dTdt(i) = (laserPower(i)+netConduction(i)-...
            convectionPower(i)-radiationPower(i))/objects(i).thermalCapacity;
    end

   
    % Explicit Euler update
   
    Tnext = T + dt*dTdt;
    Tnext = min(max(Tnext,minTAllowed),maxTAllowed);

    T = Tnext;

    currentMax = max(T);
    meanTemperatureHistory(step) = mean(T);
    maxTemperatureHistory(step) = currentMax;
    laserPowerHistory(step) = sum(laserPower);
    convectionPowerHistory(step) = sum(convectionPower);
    radiationPowerHistory(step) = sum(radiationPower);
    conductionPowerHistory(step) = sum(netConduction);
    propAirflowHistory(step) = mean(propAirflowValues);

    sumH = sumH + mean(hValues);
    sumProp = sumProp + mean(propAirflowValues);

    if currentMax > maxT
        maxT = currentMax;
        maxTTime = currentTime;
    end

    % Terminal progress
    if step == 1 || currentTime >= round((currentTime-dt)/1.0)+1
        if abs(currentTime-round(currentTime/1.0)*1.0) < dt/2 || currentTime == 0
            fprintf('  t = %6.2f s | Tmax = %9.3f C | laser = %9.3f W | conv = %9.3f W | rad = %9.3f W | prop V = %7.3f m/s\n',...
                currentTime,currentMax-273.15,sum(laserPower),sum(convectionPower),sum(radiationPower),mean(propAirflowValues));
        end
    end

    if currentMax >= maxTAllowed
        termination = "temperature safety limit";
        timeHistory = timeHistory(1:step);
        maxTemperatureHistory = maxTemperatureHistory(1:step);
        laserPowerHistory = laserPowerHistory(1:step);
        convectionPowerHistory = convectionPowerHistory(1:step);
        radiationPowerHistory = radiationPowerHistory(1:step);
        conductionPowerHistory = conductionPowerHistory(1:step);
        propAirflowHistory = propAirflowHistory(1:step);
        meanTemperatureHistory = meanTemperatureHistory(1:step);
        break;
    end
end

stats.maxTemperature_K = maxT;
stats.finalTemperature_K = max(T);
stats.finalAverageTemperature_K = mean(T);
stats.timeOfMaximum_s = maxTTime;
stats.maxLaserPower_W = max(laserPowerHistory);
stats.meanLaserPower_W = mean(laserPowerHistory);
stats.laserPowerStd_W = std(laserPowerHistory);
stats.totalLaserEnergy_J = trapz(timeHistory,laserPowerHistory);
stats.averageH = sumH/max(numel(timeHistory),1);
stats.averagePropellerAirflow = sumProp/max(numel(timeHistory),1);
stats.maxPropellerAirflow = max(propAirflowHistory);
stats.termination = termination;

caseResult.time_s = timeHistory;
caseResult.temperature_K = T;
caseResult.maxTemperatureHistory_K = maxTemperatureHistory;
caseResult.meanTemperatureHistory_K = meanTemperatureHistory;
caseResult.laserPowerHistory_W = laserPowerHistory;
caseResult.convectionPowerHistory_W = convectionPowerHistory;
caseResult.radiationPowerHistory_W = radiationPowerHistory;
caseResult.conductionPowerHistory_W = conductionPowerHistory;
caseResult.propellerAirflowHistory_m_s = propAirflowHistory;
caseResult.rotation_deg_s = rotation_deg_s;
caseResult.frequency_Hz = frequency_Hz;
caseResult.omega_rad_s = omega_z;
caseResult.propellerRPM = propRPM;
caseResult.targetObject = targetObject;
caseResult.thermalLayer = layer;
end

% % LOCAL FUNCTION: COMPONENT LASER POWER
function power = calculateComponentLaserPower(component,t,omega_z,assemblyCentroid,...
    laserOrigin,P0,alphaAtm,wavelength,D,M2,absorptivity,Cn2,...
    useTurbulence,useNoise,noiseSigma,noiseTau,noiseSpatialScale,noiseState)

centroids = component.geometry.triangleCentroids;
areas = component.geometry.triangleAreas;

if isempty(centroids)
    power = 0;
    return;
end

% The laser tracks a circular path around the drone's central waist.
dimensions = component.geometry.dimensions;
trajectoryRadius = 0.40*min(dimensions(1:2));

if ~isfinite(trajectoryRadius) || trajectoryRadius <= 0
    trajectoryRadius = 0;
end

theta = omega_z*t;
target = assemblyCentroid + [trajectoryRadius*cos(theta),...
    trajectoryRadius*sin(theta),0];

beamVector = target-laserOrigin;
range = norm(beamVector);
if range < 1e-12
    power = 0;
    return;
end
beamDirection = beamVector/range;

% Gaussian beam.
w0 = D/(pi*M2);
zR = pi*w0^2/(M2*wavelength);
w = w0*sqrt(1+(range/max(zR,eps))^2);

% Fried-parameter turbulence broadening.
if useTurbulence
    r0 = friedParameter(range,wavelength,Cn2);
    if isfinite(r0) && r0 > 0
        w = w*sqrt(1+(w/max(r0,eps))^2);
    end
end

% Atmospheric attenuation.
P_atm = P0*exp(-alphaAtm*range);

% Build beam-plane basis.
ref = [0 0 1];
if abs(dot(ref,beamDirection)) > 0.95
    ref = [0 1 0];
end

e1 = cross(beamDirection,ref);
e1 = e1/max(norm(e1),eps);
e2 = cross(beamDirection,e1);
e2 = e2/max(norm(e2),eps);

relative = centroids-target;
X = relative*e1.';
Y = relative*e2.';
r2 = X.^2+Y.^2;

% Deterministic Gaussian intensity.
I = (2*P_atm/(pi*w^2))*exp(-2*r2/w^2);

% Mean-preserving stochastic beam fluctuation.
if useNoise
    % Spatially coherent perturbation.  The temporal OU state supplies
    % slowly varying beam-power jitter while the spatial term gives local
    % speckle/turbulence structure.
    localScale = max(noiseSpatialScale*w,1e-9);
    spatialNoise = noiseSigma*randn(size(I));
    spatialNoise = spatialNoise.*exp(-r2/(2*localScale^2));
    totalNoise = spatialNoise + 0.35*noiseState;
    I = I.*exp(totalNoise-0.5*noiseSigma^2);
end

q = absorptivity*I;
power = sum(q.*areas);
power = max(power,0);

% Never allow the stochastic realization to create more total optical
% power than the available attenuated laser power by an arbitrary amount.
% The beam integration itself remains geometry dependent.
if power > absorptivity*P_atm
    power = absorptivity*P_atm;
end

end

% % LOCAL FUNCTION: CONVECTION INCLUDING PROPELLER AIRFLOW
 
function [hAverage,airflow] = calculateAverageConvection(component,omega_z,...
    rotationAxisCenter,hFree,rho,mu,kAir,Pr,L,USE_PROPELLER_AIRFLOW,...
    inducedVelocity,propSpreadFactor)

r = norm(component.centroid(1:2)-rotationAxisCenter(1:2));
componentRadius = 0.5*max(component.geometry.dimensions(1:2));
rEffective = max(r,componentRadius);

rotationalVelocity = abs(omega_z)*rEffective;

% Components farther from the propeller disk receive less induced flow.
radialFactor = exp(-r/(max(propSpreadFactor*0.25,1e-6)));
if USE_PROPELLER_AIRFLOW
    airflow = inducedVelocity*radialFactor;
else
    airflow = 0;
end

% Combine rotation-driven and propeller-induced flow.
Veff = sqrt(rotationalVelocity^2+airflow^2);

Re = rho*Veff*L/max(mu,eps);
Re = max(Re,1);

if Re < 5e5
    Nu = 0.664*sqrt(Re)*Pr^(1/3);
else
    Nu = (0.037*Re^0.8-871)*Pr^(1/3);
end

hForced = Nu*kAir/max(L,eps);
hAverage = max(hFree+hForced,1e-6);

end

% % LOCAL FUNCTION: FRIED PARAMETER
 
function r0 = friedParameter(range,wavelength,Cn2)

if range <= 0 || wavelength <= 0 || Cn2 <= 0
    r0 = Inf;
    return;
end

k = 2*pi/wavelength;
r0 = (0.423*k^2*Cn2*range)^(-3/5);

end

% % LOCAL FUNCTION: LASER AIM POINT
 
function point = getLaserAimPoint(objects,bodyIndex,target,assemblyCentroid)

target = lower(string(target));

if target == "body"
    point = objects(bodyIndex).centroid;
elseif target == "assembly" || target == "all"
    point = assemblyCentroid;
else
    point = assemblyCentroid;
end

end

% % LOCAL FUNCTION: TARGET COMPONENT
function tf = isTargetComponent(component,target)

target = lower(string(target));
type = lower(string(component.componentType));
number = component.componentNumber;

tf = false;

if target == "none"
    return;
elseif target == "all"
    tf = true;
elseif target == "body"
    tf = type == "body";
elseif startsWith(target,"motor")
    n = str2double(extractAfter(target,"motor"));
    tf = type == "motor" && number == n;
elseif startsWith(target,"prop")
    n = str2double(extractAfter(target,"prop"));
    tf = type == "prop" && number == n;
end

end

% % LOCAL FUNCTION: ESTIMATE PROPELLER DIAMETER
function Dprop = estimatePropellerDiameter(objects,propIndices,minDiameter)

values = zeros(numel(propIndices),1);

for k = 1:numel(propIndices)
    i = propIndices(k);
    dims = objects(i).geometry.dimensions;
    values(k) = max(dims(1:2));
end

values = values(isfinite(values) & values > minDiameter);

if isempty(values)
    Dprop = minDiameter;
else
    Dprop = median(values);
end

end

% % LOCAL FUNCTION: READ 3MF
 
function [objects,allPoints,allTriangles] = readDrone3MF(filename)

tempFolder = tempname;
mkdir(tempFolder);
cleanupObj = onCleanup(@()cleanup3MF(tempFolder)); %#ok<NASGU>

try
    unzip(filename,tempFolder);
catch ME
    error('Could not unzip the 3MF file.\n%s',ME.message);
end

modelFile = fullfile(tempFolder,'3D','3dmodel.model');
if ~isfile(modelFile)
    error('3MF model file was not found: %s',modelFile);
end

try
    xmlDoc = xmlread(modelFile);
catch ME
    error('Could not parse 3MF XML.\n%s',ME.message);
end

root = xmlDoc.getDocumentElement();
unitName = lower(string(char(root.getAttribute('unit'))));
if strlength(unitName) == 0
    unitName = "millimeter";
end

switch unitName
    case "millimeter"
        unitScale = 1e-3;
    case "centimeter"
        unitScale = 1e-2;
    case "meter"
        unitScale = 1;
    case "inch"
        unitScale = 0.0254;
    case "micron"
        unitScale = 1e-6;
    otherwise
        warning('Unknown 3MF unit %s. Assuming millimeters.',unitName);
        unitScale = 1e-3;
end

fprintf('3MF units: %s\n',unitName);

objectNodes = root.getElementsByTagName('object');
numObjects = objectNodes.getLength;
if numObjects == 0
    error('No mesh objects were found in the 3MF file.');
end

objects = struct('name',{},'points',{},'triangles',{},...
    'geometry',{},'material',{},'componentType',{},'componentNumber',{},...
    'centroid',{},'area',{},'volume',{},'mass',{},'thermalCapacity',{},...
    'temperature',{});
allPoints = zeros(0,3);
allTriangles = zeros(0,3);

for objectIndex = 0:numObjects-1
    objectNode = objectNodes.item(objectIndex);
    objectName = string(char(objectNode.getAttribute('name')));
    if strlength(objectName) == 0
        objectName = "Object"+string(objectIndex+1);
    end

    meshNodes = objectNode.getElementsByTagName('mesh');
    if meshNodes.getLength == 0
        continue;
    end
    meshNode = meshNodes.item(0);

    vertexNodes = meshNode.getElementsByTagName('vertex');
    nV = vertexNodes.getLength;
    points = zeros(nV,3);

    for v = 0:nV-1
        vertex = vertexNodes.item(v);
        points(v+1,:) = [str2double(char(vertex.getAttribute('x'))),...
            str2double(char(vertex.getAttribute('y'))),...
            str2double(char(vertex.getAttribute('z')))]*unitScale;
    end

    triangleNodes = meshNode.getElementsByTagName('triangle');
    nT = triangleNodes.getLength;
    triangles = zeros(nT,3);

    for t = 0:nT-1
        triangleNode = triangleNodes.item(t);
        triangles(t+1,:) = [str2double(char(triangleNode.getAttribute('v1'))),...
            str2double(char(triangleNode.getAttribute('v2'))),...
            str2double(char(triangleNode.getAttribute('v3')))] + 1;
    end

    newObject.name = objectName;
    newObject.points = points;
    newObject.triangles = triangles;
    newObject.geometry = [];
    newObject.material = "";
    newObject.componentType = "other";
    newObject.componentNumber = 0;
    newObject.centroid = [0 0 0];
    newObject.area = 0;
    newObject.volume = 0;
    newObject.mass = 0;
    newObject.thermalCapacity = 0;
    newObject.temperature = 298.15;
    objects(end+1) = newObject; %#ok<AGROW>

    vertexOffset = size(allPoints,1);
    allPoints = [allPoints;points]; %#ok<AGROW>
    allTriangles = [allTriangles;triangles+vertexOffset]; %#ok<AGROW>
end

if isempty(allTriangles)
    error('3MF contains no triangles.');
end

usedVertices = unique(allTriangles(:));
vertexMap = zeros(size(allPoints,1),1);
vertexMap(usedVertices) = 1:numel(usedVertices);
allPoints = allPoints(usedVertices,:);
allTriangles = vertexMap(allTriangles);

sortedTriangles = sort(allTriangles,2);
[~,uniqueIndices] = unique(sortedTriangles,'rows','stable');
allTriangles = allTriangles(uniqueIndices,:);

end

% % LOCAL FUNCTION: BOOLEAN TO ON/OFF
 
function result = logicalToOnOff(value)
if value
    result = "ON";
else
    result = "OFF";
end
end

% % LOCAL FUNCTION: CLEANUP
function cleanup3MF(folder)
if isfolder(folder)
    try
        rmdir(folder,'s');
    catch
    end
end
end
